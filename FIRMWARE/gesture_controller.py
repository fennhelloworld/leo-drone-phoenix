#!/usr/bin/env python3
"""
LeoDrone Phoenix — 手势控制器
MediaPipe 手势识别 → MAVLink 飞行指令

手势集:
  👆 指向上 → 上升    👇 指向下 → 下降
  ✋ 张开手掌 → 悬停   ✊ 握拳 → 拍照
  👈 左滑 → 左移      👉 右滑 → 右移
  🤏 捏合 → 变焦      👋 挥手 → 跟随模式
"""

import logging
import time
import numpy as np
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass
from enum import Enum, auto

logger = logging.getLogger("GestureController")


class Gesture(Enum):
    """手势类型"""
    NONE = auto()
    POINT_UP = auto()      # 👆 上升
    POINT_DOWN = auto()    # 👇 下降
    OPEN_PALM = auto()     # ✋ 悬停
    FIST = auto()          # ✊ 拍照
    SWIPE_LEFT = auto()    # 👈 左移
    SWIPE_RIGHT = auto()   # 👉 右移
    PINCH = auto()         # 🤏 变焦
    WAVE = auto()          # 👋 跟随模式
    THUMBS_UP = auto()     # 👍 确认
    PEACE = auto()         # ✌️ 测量


@dataclass
class HandLandmarks:
    """手部21关键点"""
    positions: np.ndarray  # (21, 3) x, y, z
    handedness: str  # "Left" or "Right"
    confidence: float

    # Key landmark indices
    WRIST = 0
    THUMB_TIP = 4
    INDEX_TIP = 8
    MIDDLE_TIP = 12
    RING_TIP = 16
    PINKY_TIP = 20
    THUMB_MCP = 2
    INDEX_MCP = 5
    MIDDLE_MCP = 9
    RING_MCP = 13
    PINKY_MCP = 17


class MediaPipeDetector:
    """MediaPipe 手部检测"""

    def __init__(self, max_hands: int = 2, min_detection_confidence: float = 0.7):
        self.max_hands = max_hands
        self.min_confidence = min_detection_confidence
        self._mp_hands = None
        self._detector = None

    def initialize(self) -> bool:
        """Initialize MediaPipe Hands"""
        try:
            import mediapipe as mp
            self._mp_hands = mp.solutions.hands
            self._detector = self._mp_hands.Hands(
                static_image_mode=False,
                max_num_hands=self.max_hands,
                min_detection_confidence=self.min_confidence,
                min_tracking_confidence=0.5
            )
            return True
        except ImportError:
            logger.warning("MediaPipe not available, using simulation")
            self._detector = None
            return False

    def detect(self, frame: np.ndarray) -> List[HandLandmarks]:
        """Detect hands in frame"""
        if self._detector is None:
            return self._simulated_detect()

        try:
            results = self._detector.process(frame)
            landmarks_list = []

            if results.multi_hand_landmarks:
                for hand_lm, hand_class in zip(
                        results.multi_hand_landmarks,
                        results.multi_handedness):
                    positions = np.array([[lm.x, lm.y, lm.z]
                                         for lm in hand_lm.landmark])
                    handedness = hand_class.classification[0].label
                    confidence = hand_class.classification[0].score

                    landmarks_list.append(HandLandmarks(
                        positions=positions,
                        handedness=handedness,
                        confidence=confidence
                    ))

            return landmarks_list
        except Exception as e:
            logger.error(f"Hand detection error: {e}")
            return []

    def _simulated_detect(self) -> List[HandLandmarks]:
        """Generate simulated hand landmarks"""
        # Simulate a right hand in open palm position
        positions = np.zeros((21, 3))
        # Wrist at center
        positions[0] = [0.5, 0.5, 0]
        # Spread fingers for open palm
        for i in range(1, 5):  # Thumb to pinky
            tip_idx = i * 4
            mcp_idx = i * 4 - 2
            positions[mcp_idx] = [0.3 + i * 0.1, 0.5, 0]
            positions[tip_idx] = [0.3 + i * 0.1, 0.3, 0]

        return [HandLandmarks(
            positions=positions,
            handedness="Right",
            confidence=0.95
        )]


class GestureClassifier:
    """手势分类器 — 从关键点序列识别手势"""

    # Tracking state for dynamic gestures
    def __init__(self):
        self._prev_wrist_pos: Optional[np.ndarray] = None
        self._prev_time: float = 0
        self._wave_count: int = 0
        self._swipe_start: Optional[np.ndarray] = None
        self._swipe_start_time: float = 0

    def classify(self, landmarks: HandLandmarks) -> Gesture:
        """Classify hand landmarks into a gesture"""
        lm = landmarks.positions

        # Calculate finger states (extended or curled)
        fingers_extended = self._check_fingers_extended(lm)

        # Check for specific gestures

        # ✋ Open Palm - all fingers extended
        if sum(fingers_extended) >= 4:
            # Check for wave (palm moving left-right)
            if self._detect_wave(landmarks):
                return Gesture.WAVE
            return Gesture.OPEN_PALM

        # ✊ Fist - no fingers extended
        if sum(fingers_extended) == 0:
            return Gesture.FIST

        # 👆 Point Up - only index extended, pointing up
        if fingers_extended[1] and not fingers_extended[2]:
            index_tip = lm[HandLandmarks.INDEX_TIP]
            index_mcp = lm[HandLandmarks.INDEX_MCP]
            if index_tip[1] < index_mcp[1]:  # Tip above MCP
                return Gesture.POINT_UP
            else:
                return Gesture.POINT_DOWN

        # 👇 Point Down - only index extended, pointing down
        if fingers_extended[1] and index_tip[1] > index_mcp[1]:
            return Gesture.POINT_DOWN

        # 🤏 Pinch - thumb and index close together
        thumb_tip = lm[HandLandmarks.THUMB_TIP]
        index_tip = lm[HandLandmarks.INDEX_TIP]
        pinch_dist = np.linalg.norm(thumb_tip - index_tip)
        if pinch_dist < 0.05:
            return Gesture.PINCH

        # 👍 Thumbs Up - only thumb extended
        if fingers_extended[0] and sum(fingers_extended[1:]) == 0:
            return Gesture.THUMBS_UP

        # ✌️ Peace - index and middle extended
        if fingers_extended[1] and fingers_extended[2] and sum(fingers_extended) == 2:
            return Gesture.PEACE

        # Check for swipe gestures
        swipe = self._detect_swipe(landmarks)
        if swipe:
            return swipe

        return Gesture.NONE

    def _check_fingers_extended(self, lm: np.ndarray) -> List[bool]:
        """Check which fingers are extended"""
        extended = []

        # Thumb (check x distance from palm center)
        thumb_tip = lm[4]
        thumb_ip = lm[3]
        wrist = lm[0]
        thumb_mcp = lm[2]
        extended.append(abs(thumb_tip[0] - wrist[0]) > abs(thumb_mcp[0] - wrist[0]))

        # Other fingers (check y position: tip above PIP = extended)
        finger_tips = [8, 12, 16, 20]
        finger_pips = [6, 10, 14, 18]
        for tip, pip in zip(finger_tips, finger_pips):
            extended.append(lm[tip][1] < lm[pip][1])

        return extended

    def _detect_wave(self, landmarks: HandLandmarks) -> bool:
        """Detect wave gesture (palm moving left-right)"""
        wrist = landmarks.positions[0].copy()
        current_time = time.time()

        if self._prev_wrist_pos is not None:
            dx = wrist[0] - self._prev_wrist_pos[0]
            dt = current_time - self._prev_time

            if dt > 0 and abs(dx / dt) > 0.5:  # Fast horizontal movement
                self._wave_count += 1
                if self._wave_count >= 3:  # 3 oscillations = wave
                    self._wave_count = 0
                    return True

        self._prev_wrist_pos = wrist
        self._prev_time = current_time
        return False

    def _detect_swipe(self, landmarks: HandLandmarks) -> Optional[Gesture]:
        """Detect swipe left/right gesture"""
        wrist = landmarks.positions[0].copy()
        current_time = time.time()

        if self._swipe_start is None:
            self._swipe_start = wrist
            self._swipe_start_time = current_time
            return None

        dt = current_time - self._swipe_start_time
        if dt > 1.0:  # Reset after 1 second
            self._swipe_start = wrist
            self._swipe_start_time = current_time
            return None

        dx = wrist[0] - self._swipe_start[0]
        if abs(dx) > 0.15:  # Significant horizontal movement
            self._swipe_start = None
            if dx > 0:
                return Gesture.SWIPE_RIGHT
            else:
                return Gesture.SWIPE_LEFT

        return None


class GestureController:
    """手势控制器 — MediaPipe手势 → MAVLink指令"""

    # Gesture to MAVLink command mapping
    GESTURE_COMMANDS = {
        Gesture.POINT_UP: {'type': 'goto', 'params': {'north': 0, 'east': 0, 'down': -1}},
        Gesture.POINT_DOWN: {'type': 'goto', 'params': {'north': 0, 'east': 0, 'down': 1}},
        Gesture.OPEN_PALM: {'type': 'hover', 'params': {}},
        Gesture.FIST: {'type': 'photo', 'params': {}},
        Gesture.SWIPE_LEFT: {'type': 'goto', 'params': {'north': 0, 'east': -2, 'down': 0}},
        Gesture.SWIPE_RIGHT: {'type': 'goto', 'params': {'north': 0, 'east': 2, 'down': 0}},
        Gesture.PINCH: {'type': 'zoom', 'params': {'level': 2}},
        Gesture.WAVE: {'type': 'follow', 'params': {}},
        Gesture.THUMBS_UP: {'type': 'confirm', 'params': {}},
        Gesture.PEACE: {'type': 'measure', 'params': {}},
    }

    def __init__(self, simulation: bool = True):
        self.simulation = simulation
        self._detector = MediaPipeDetector()
        self._classifier = GestureClassifier()
        self._last_gesture = Gesture.NONE
        self._gesture_cooldown: Dict[Gesture, float] = {}
        self._cooldown_duration = 1.0  # seconds

        if not simulation:
            self._detector.initialize()

    def detect(self, frame: Optional[np.ndarray] = None) -> Optional[Gesture]:
        """Detect gesture from frame"""
        if frame is None and self.simulation:
            landmarks = self._detector._simulated_detect()
        elif frame is not None:
            landmarks = self._detector.detect(frame)
        else:
            return None

        if not landmarks:
            return None

        # Classify first hand
        gesture = self._classifier.classify(landmarks[0])

        # Cooldown check
        if gesture != Gesture.NONE:
            now = time.time()
            last_time = self._gesture_cooldown.get(gesture, 0)
            if now - last_time < self._cooldown_duration:
                return None
            self._gesture_cooldown[gesture] = now

        self._last_gesture = gesture
        return gesture

    def gesture_to_mavlink(self, gesture: Gesture) -> Optional[Dict]:
        """Convert gesture to MAVLink command"""
        command = self.GESTURE_COMMANDS.get(gesture)
        if command:
            logger.info(f"🖐️ Gesture: {gesture.name} → {command}")
        return command

    def get_last_gesture(self) -> Gesture:
        """Get the last detected gesture"""
        return self._last_gesture

    def set_gesture_callback(self, callback):
        """Set callback for gesture events"""
        self._callback = callback


# ---------------------------------------------------------------------------
# CLI Test
# ---------------------------------------------------------------------------

if __name__ == "__main__":
    controller = GestureController(simulation=True)

    print("Testing gesture classification...")

    # Test simulated detection
    gesture = controller.detect()
    if gesture:
        print(f"Detected: {gesture.name}")
        cmd = controller.gesture_to_mavlink(gesture)
        print(f"Command: {cmd}")

    # Test all gesture types
    print("\nGesture → Command mapping:")
    for g in Gesture:
        cmd = controller.gesture_to_mavlink(g)
        if cmd:
            print(f"  {g.name:15s} → {cmd}")
