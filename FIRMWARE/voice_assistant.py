#!/usr/bin/env python3
"""
LeoDrone Phoenix — 语音助手
唤醒词 + STT + LLM + TTS 语音交互管线

架构:
  麦克风 → VAD → 唤醒词检测 → STT(Whisper) → LLM(DeepSeek) → TTS → 扬声器
"""

import asyncio
import logging
import time
import json
import numpy as np
from typing import Optional, Dict, Callable, List
from dataclasses import dataclass

logger = logging.getLogger("VoiceAssistant")


@dataclass
class VoiceCommand:
    """语音命令"""
    text: str
    intent: str
    params: Dict
    confidence: float
    timestamp: float


class WakeWordDetector:
    """唤醒词检测器 (Phoenix / 凤凰)"""

    WAKE_WORDS = ["phoenix", "凤凰", "phoenix wake"]

    def __init__(self, sensitivity: float = 0.5):
        self.sensitivity = sensitivity
        self._detector = None

    def initialize(self) -> bool:
        """Initialize wake word detector"""
        try:
            import openwakeword
            self._detector = openwakeword.Model()
            return True
        except ImportError:
            try:
                import porcupine
                self._detector = porcupine.create(keywords=["hey google"])
                return True
            except ImportError:
                logger.warning("No wake word engine, using simple energy detection")
                self._detector = None
                return False

    def detect(self, audio_frame: np.ndarray) -> bool:
        """Check if wake word is detected in audio frame"""
        if self._detector is not None:
            try:
                if hasattr(self._detector, 'predict'):
                    prediction = self._detector.predict(audio_frame)
                    return any(p > self.sensitivity for p in prediction.values()
                             if isinstance(p, float))
            except Exception:
                pass

        # Fallback: simple energy-based detection
        energy = np.sqrt(np.mean(audio_frame ** 2))
        return energy > 0.3  # Threshold


class SpeechToText:
    """语音转文字 (Whisper)"""

    def __init__(self, model_size: str = "tiny", language: str = "zh"):
        self.model_size = model_size
        self.language = language
        self._model = None

    def initialize(self) -> bool:
        """Initialize Whisper model"""
        try:
            import whisper
            self._model = whisper.load_model(self.model_size)
            return True
        except ImportError:
            logger.warning("Whisper not available, using simulation mode")
            self._model = None
            return False

    def transcribe(self, audio: np.ndarray) -> str:
        """Transcribe audio to text"""
        if self._model is not None:
            try:
                result = self._model.transcribe(
                    audio,
                    language=self.language,
                    fp16=False
                )
                return result["text"].strip()
            except Exception as e:
                logger.error(f"STT error: {e}")
                return ""

        # Simulation mode: return a random command
        sim_commands = [
            "起飞到五米",
            "向前飞十米",
            "跟随我",
            "拍照",
            "返航",
            "当前高度是多少",
            "报告状态",
            "停止飞行",
        ]
        return np.random.choice(sim_commands)


class LLMEngine:
    """大语言模型引擎 (DeepSeek / 本地)"""

    def __init__(self, model: str = "deepseek-chat",
                 api_key: Optional[str] = None,
                 base_url: Optional[str] = None):
        self.model = model
        self.api_key = api_key
        self.base_url = base_url
        self._client = None
        self._conversation_history: List[Dict] = []

        # System prompt for drone assistant
        self.system_prompt = """你是 LeoDrone Phoenix 的语音助手。你控制一台360°全景AI穿越无人机。

你可以执行以下命令:
- takeoff [高度] — 起飞到指定高度(默认5米)
- land — 降落
- goto [北] [东] [下] — 飞往指定位置(NED坐标系)
- follow — 跟随模式
- orbit [半径] — 环绕飞行
- photo — 拍照
- video [start/stop] — 开始/停止录像
- rtl — 返航
- status — 报告当前状态
- help — 帮助

请用简洁的中文回复。对于飞行命令，请返回JSON格式:
{"type": "command", "action": "...", "params": {...}}

对于普通对话，正常回复即可。"""

    def initialize(self) -> bool:
        """Initialize LLM client"""
        try:
            from openai import OpenAI
            self._client = OpenAI(
                api_key=self.api_key or "sk-placeholder",
                base_url=self.base_url or "https://api.deepseek.com"
            )
            return True
        except ImportError:
            logger.warning("OpenAI client not available, using simulation")
            self._client = None
            return False

    async def chat(self, user_message: str) -> str:
        """Send message and get response"""
        self._conversation_history.append({
            "role": "user",
            "content": user_message
        })

        if self._client is not None:
            try:
                response = self._client.chat.completions.create(
                    model=self.model,
                    messages=[
                        {"role": "system", "content": self.system_prompt}
                    ] + self._conversation_history[-10:],  # Keep last 10 turns
                    max_tokens=200,
                    temperature=0.7
                )
                reply = response.choices[0].message.content
                self._conversation_history.append({
                    "role": "assistant",
                    "content": reply
                })
                return reply
            except Exception as e:
                logger.error(f"LLM error: {e}")
                return self._simulate_response(user_message)

        return self._simulate_response(user_message)

    def _simulate_response(self, user_message: str) -> str:
        """Simulate LLM response for testing"""
        msg = user_message.lower()

        if any(w in msg for w in ["起飞", "takeoff", "升空"]):
            return '{"type": "command", "action": "takeoff", "params": {"altitude": 5}}'
        elif any(w in msg for w in ["降落", "land", "着陆"]):
            return '{"type": "command", "action": "land", "params": {}}'
        elif any(w in msg for w in ["跟随", "follow", "跟踪"]):
            return '{"type": "command", "action": "follow", "params": {}}'
        elif any(w in msg for w in ["返航", "rtl", "回来"]):
            return '{"type": "command", "action": "rtl", "params": {}}'
        elif any(w in msg for w in ["拍照", "photo", "截图"]):
            return '{"type": "command", "action": "photo", "params": {}}'
        elif any(w in msg for w in ["状态", "status", "高度"]):
            return "当前高度5.2米，电量85%，飞行模式：Offboard，一切正常。"
        elif any(w in msg for w in ["向前", "前进", "forward"]):
            return '{"type": "command", "action": "goto", "params": {"north": 10, "east": 0, "down": -5}}'
        else:
            return "收到指令，请问具体需要我做什么？"


class TextToSpeech:
    """文字转语音 (Edge-TTS / VITS)"""

    def __init__(self, voice: str = "zh-CN-XiaoxiaoNeural",
                 rate: str = "+0%", volume: str = "+0%"):
        self.voice = voice
        self.rate = rate
        self.volume = volume

    async def synthesize(self, text: str, output_file: Optional[str] = None) -> Optional[bytes]:
        """Convert text to speech"""
        try:
            import edge_tts
            communicate = edge_tts.Communicate(text, self.voice,
                                                rate=self.rate,
                                                volume=self.volume)
            if output_file:
                await communicate.save(output_file)
                return None
            else:
                # Return audio bytes
                chunks = []
                async for chunk in communicate.stream():
                    if chunk["type"] == "audio":
                        chunks.append(chunk["data"])
                return b"".join(chunks)
        except ImportError:
            logger.debug("Edge-TTS not available, skipping audio output")
            return None
        except Exception as e:
            logger.error(f"TTS error: {e}")
            return None


class IntentParser:
    """解析LLM输出中的命令意图"""

    FLIGHT_COMMANDS = {
        'takeoff', 'land', 'goto', 'follow', 'orbit',
        'rtl', 'photo', 'video', 'stop', 'hover'
    }

    def parse(self, llm_response: str) -> Optional[VoiceCommand]:
        """Parse LLM response into a VoiceCommand"""
        # Try to extract JSON
        try:
            # Find JSON in response
            start = llm_response.find('{')
            end = llm_response.rfind('}') + 1
            if start >= 0 and end > start:
                json_str = llm_response[start:end]
                data = json.loads(json_str)

                if data.get('type') == 'command':
                    return VoiceCommand(
                        text=llm_response,
                        intent=data.get('action', 'unknown'),
                        params=data.get('params', {}),
                        confidence=0.9,
                        timestamp=time.time()
                    )
        except (json.JSONDecodeError, ValueError):
            pass

        # Natural language intent detection
        msg = llm_response.lower()
        for cmd in self.FLIGHT_COMMANDS:
            if cmd in msg:
                return VoiceCommand(
                    text=llm_response,
                    intent=cmd,
                    params={},
                    confidence=0.6,
                    timestamp=time.time()
                )

        return None


class VoiceAssistant:
    """语音助手 — 完整管线"""

    def __init__(self, simulation: bool = True):
        self.simulation = simulation
        self.running = False

        # Components
        self.wake_word = WakeWordDetector()
        self.stt = SpeechToText()
        self.llm = LLMEngine()
        self.tts = TextToSpeech()
        self.intent_parser = IntentParser()

        # State
        self._awake = False
        self._command_queue: asyncio.Queue = asyncio.Queue()

        # Initialize
        if not simulation:
            self.wake_word.initialize()
            self.stt.initialize()
            self.llm.initialize()

    def is_available(self) -> bool:
        """Check if voice assistant is available"""
        return True  # Always available in some mode

    async def listen(self) -> Optional[VoiceCommand]:
        """Listen for voice commands (non-blocking)"""
        try:
            return self._command_queue.get_nowait()
        except asyncio.QueueEmpty:
            return None

        # In real mode, would continuously:
        # 1. Read audio from microphone
        # 2. Detect wake word
        # 3. Record command audio
        # 4. STT
        # 5. LLM
        # 6. Parse intent
        # 7. TTS response

    async def process(self, text: str) -> str:
        """Process a text command through the LLM pipeline"""
        # Get LLM response
        response = await self.llm.chat(text)

        # Parse intent
        command = self.intent_parser.parse(response)
        if command:
            await self._command_queue.put(command)

        # TTS output (non-blocking)
        if not self.simulation:
            asyncio.create_task(self.tts.synthesize(response))

        return response

    async def process_audio(self, audio: np.ndarray) -> Optional[VoiceCommand]:
        """Process audio through the full pipeline"""
        # Wake word detection
        if not self._awake:
            if self.wake_word.detect(audio):
                self._awake = True
                logger.info("🎤 Wake word detected!")
                await self.tts.synthesize("我在，请说")
            return None

        # STT
        text = self.stt.transcribe(audio)
        if not text:
            return None

        logger.info(f"🎤 Heard: {text}")

        # LLM
        response = await self.llm.chat(text)
        logger.info(f"🤖 Reply: {response}")

        # TTS
        if not self.simulation:
            await self.tts.synthesize(response)

        # Parse intent
        command = self.intent_parser.parse(response)
        self._awake = False  # Reset wake word after processing
        return command

    def inject_command(self, text: str):
        """Inject a text command for testing"""
        command = VoiceCommand(
            text=text,
            intent="test",
            params={"text": text},
            confidence=1.0,
            timestamp=time.time()
        )
        self._command_queue.put_nowait(command)


# ---------------------------------------------------------------------------
# CLI Test
# ---------------------------------------------------------------------------

async def test_voice():
    """Test the voice assistant"""
    assistant = VoiceAssistant(simulation=True)

    test_commands = [
        "起飞到五米",
        "跟随我",
        "拍照",
        "返航",
        "当前状态",
    ]

    for cmd in test_commands:
        print(f"\n🎤 命令: {cmd}")
        response = await assistant.process(cmd)
        print(f"🤖 回复: {response}")

        voice_cmd = await assistant.listen()
        if voice_cmd:
            print(f"📋 解析: intent={voice_cmd.intent}, params={voice_cmd.params}")


if __name__ == "__main__":
    asyncio.run(test_voice())
