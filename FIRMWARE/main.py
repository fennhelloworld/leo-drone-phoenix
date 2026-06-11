#!/usr/bin/env python3
"""
LeoDrone Phoenix — 主伴飞计算机脚本
MAVSDK + 相机 + AI Pipeline 主循环

架构:
  传感器 → 感知 → 融合 → 认知 → 协调 → 控制
    ↑                                    ↓
  安全层 ◄───────────────────────────────┘
"""

import asyncio
import logging
import signal
import sys
import time
import json
import numpy as np
from dataclasses import dataclass, field
from typing import Optional, Dict, List, Any
from enum import Enum, auto

# MAVSDK for drone control
try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
    HAS_MAVSDK = True
except ImportError:
    HAS_MAVSDK = False

# omni-perception-fusion imports
try:
    from omni_perception_fusion.ekf import ExtendedKalmanFilter
    from omni_perception_fusion.slam import VINSFusion
    from omni_perception_fusion.detection import YOLODetector
    from omni_perception_fusion.causal import CausalGraph
    from omni_perception_fusion.temporal_kg import TemporalKnowledgeGraph
    from omni_perception_fusion.world_model import WorldModel
    from omni_perception_fusion.moe import MixtureOfExperts
    from omni_perception_fusion.mapping import CollaborativeMapper
    HAS_OPF = True
except ImportError:
    HAS_OPF = False

# Local modules
from sensor_node import SensorNode, SensorReading
from voice_assistant import VoiceAssistant
from gesture_controller import GestureController
from swarm_coordinator import SwarmCoordinator
from video_editor import VideoEditor

# ---------------------------------------------------------------------------
# Configuration
# ---------------------------------------------------------------------------

LOG_FORMAT = "%(asctime)s [%(levelname)s] %(name)s: %(message)s"
logging.basicConfig(level=logging.INFO, format=LOG_FORMAT)
logger = logging.getLogger("Phoenix")


class FlightMode(Enum):
    """飞行模式"""
    DISARMED = auto()
    MANUAL = auto()
    STABILIZED = auto()
    OFFBOARD = auto()
    FOLLOW = auto()
    ORBIT = auto()
    RETURN_TO_LAUNCH = auto()


class SafetyStatus(Enum):
    """安全状态"""
    SAFE = "safe"
    WARNING = "warning"
    CRITICAL = "critical"
    EMERGENCY = "emergency"


@dataclass
class DroneState:
    """无人机完整状态"""
    # Position (NED frame)
    position_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity_ned: np.ndarray = field(default_factory=lambda: np.zeros(3))
    attitude_euler: np.ndarray = field(default_factory=lambda: np.zeros(3))  # roll, pitch, yaw

    # Flight status
    mode: FlightMode = FlightMode.DISARMED
    armed: bool = False
    battery_voltage: float = 16.8
    battery_percent: float = 100.0

    # GPS
    gps_lat: float = 0.0
    gps_lon: float = 0.0
    gps_alt: float = 0.0
    gps_satellites: int = 0
    gps_fix: int = 0

    # Sensors
    imu_accel: np.ndarray = field(default_factory=lambda: np.zeros(3))
    imu_gyro: np.ndarray = field(default_factory=lambda: np.zeros(3))
    baro_alt: float = 0.0
    temperature: float = 25.0
    humidity: float = 50.0
    pressure: float = 1013.25

    # Perception
    tracked_targets: List[Dict] = field(default_factory=list)
    slam_position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    slam_map_ready: bool = False

    # Safety
    safety_status: SafetyStatus = SafetyStatus.SAFE
    geofence_ok: bool = True
    obstacles: List[Dict] = field(default_factory=list)
    weather_ok: bool = True

    # Timestamp
    timestamp: float = field(default_factory=time.time)


class PhoenixController:
    """Phoenix 主控制器 — 9层架构协调器"""

    def __init__(self, connection_url: str = "udp://:14540",
                 simulation: bool = True):
        self.connection_url = connection_url
        self.simulation = simulation
        self.running = False
        self.state = DroneState()

        # Drone connection
        self.drone: Optional[System] = None

        # Subsystems
        self.sensor_node = SensorNode(simulation=simulation)
        self.voice_assistant = VoiceAssistant(simulation=simulation)
        self.gesture_controller = GestureController(simulation=simulation)
        self.swarm_coordinator = SwarmCoordinator(simulation=simulation)
        self.video_editor = VideoEditor(simulation=simulation)

        # Perception modules
        self.ekf = ExtendedKalmanFilter() if HAS_OPF else self._make_ekf()
        self.slam = VINSFusion() if HAS_OPF else None
        self.detector = YOLODetector() if HAS_OPF else None
        self.causal_graph = CausalGraph() if HAS_OPF else None
        self.temporal_kg = TemporalKnowledgeGraph() if HAS_OPF else None
        self.world_model = WorldModel() if HAS_OPF else None
        self.moe = MixtureOfExperts() if HAS_OPF else None
        self.mapper = CollaborativeMapper() if HAS_OPF else None

        # Task references
        self._tasks: List[asyncio.Task] = []

    # ------------------------------------------------------------------
    # Fallback implementations (no hardware/GPU needed)
    # ------------------------------------------------------------------

    @staticmethod
    def _make_ekf():
        """Lightweight EKF fallback using pure NumPy"""
        class SimpleEKF:
            def __init__(self):
                self.state = np.zeros(9)  # pos(3) + vel(3) + att(3)
                self.P = np.eye(9) * 0.1
                self.Q = np.eye(9) * 0.01
                self.R_gps = np.eye(3) * 2.5
                self.R_baro = np.eye(1) * 1.0

            def predict(self, accel, gyro, dt):
                """EKF predict step"""
                F = np.eye(9)
                F[0:3, 3:6] = np.eye(3) * dt
                self.state = F @ self.state
                self.state[3:6] += accel * dt
                self.P = F @ self.P @ F.T + self.Q

            def update_gps(self, gps_pos):
                """EKF update with GPS"""
                H = np.zeros((3, 9))
                H[0:3, 0:3] = np.eye(3)
                y = gps_pos - H @ self.state
                S = H @ self.P @ H.T + self.R_gps
                K = self.P @ H.T @ np.linalg.inv(S)
                self.state += K @ y
                self.P = (np.eye(9) - K @ H) @ self.P

            def update_baro(self, baro_alt):
                """EKF update with barometer"""
                H = np.zeros((1, 9))
                H[0, 2] = 1.0
                y = np.array([baro_alt]) - H @ self.state
                S = H @ self.P @ H.T + self.R_baro
                K = self.P @ H.T @ np.linalg.inv(S)
                self.state += (K @ y).flatten()
                self.P = (np.eye(9) - K @ H) @ self.P

            def get_position(self):
                return self.state[0:3].copy()

            def get_velocity(self):
                return self.state[3:6].copy()
        return SimpleEKF()

    # ------------------------------------------------------------------
    # Connection
    # ------------------------------------------------------------------

    async def connect(self):
        """Connect to the drone via MAVSDK"""
        if not HAS_MAVSDK:
            logger.warning("MAVSDK not available, running in offline mode")
            return

        self.drone = System()
        await self.drone.connect(system_address=self.connection_url)

        logger.info(f"Connecting to {self.connection_url}...")
        async for state in self.drone.core.connection_state():
            if state.is_connected:
                logger.info("✅ Drone connected!")
                break

    # ------------------------------------------------------------------
    # Telemetry loop
    # ------------------------------------------------------------------

    async def telemetry_loop(self):
        """Continuously read telemetry data"""
        if not self.drone:
            # Simulated telemetry
            while self.running:
                await asyncio.sleep(0.05)
                self._update_simulated_telemetry()
            return

        # Real MAVSDK telemetry
        async for position in self.drone.telemetry.position():
            self.state.position_ned = np.array([
                position.latitude_deg,
                position.longitude_deg,
                position.relative_altitude_m
            ])

        async for velocity in self.drone.telemetry.velocity_ned():
            self.state.velocity_ned = np.array([
                velocity.north_m_s,
                velocity.east_m_s,
                velocity.down_m_s
            ])

        async for battery in self.drone.telemetry.battery():
            self.state.battery_voltage = battery.voltage_v
            self.state.battery_percent = battery.remaining_percent * 100

    def _update_simulated_telemetry(self):
        """Update state with simulated telemetry data"""
        reading = self.sensor_node.read_all()

        self.state.imu_accel = reading.get('accel', np.zeros(3))
        self.state.imu_gyro = reading.get('gyro', np.zeros(3))
        self.state.baro_alt = reading.get('altitude', 0.0)
        self.state.temperature = reading.get('temperature', 25.0)
        self.state.humidity = reading.get('humidity', 50.0)
        self.state.pressure = reading.get('pressure', 1013.25)
        self.state.gps_satellites = reading.get('num_satellites', 0)
        self.state.gps_fix = reading.get('fix_type', 0)

        # EKF update
        dt = 0.05
        self.ekf.predict(self.state.imu_accel, self.state.imu_gyro, dt)
        if self.state.gps_fix >= 3:
            self.ekf.update_gps(np.array([
                reading.get('lat', 0.0),
                reading.get('lon', 0.0),
                reading.get('alt', 0.0)
            ]))
        self.ekf.update_baro(self.state.baro_alt)

        self.state.position_ned = self.ekf.get_position()
        self.state.velocity_ned = self.ekf.get_velocity()
        self.state.timestamp = time.time()

    # ------------------------------------------------------------------
    # Safety monitoring
    # ------------------------------------------------------------------

    async def safety_loop(self):
        """Continuous safety monitoring"""
        while self.running:
            await asyncio.sleep(0.1)
            self._check_safety()

    def _check_safety(self):
        """Check all safety conditions"""
        issues = []

        # Battery check
        if self.state.battery_percent < 10:
            issues.append("CRITICAL: Battery < 10%")
            self.state.safety_status = SafetyStatus.EMERGENCY
        elif self.state.battery_percent < 20:
            issues.append("WARNING: Battery < 20%")

        # Geofence check
        dist = np.linalg.norm(self.state.position_ned[:2])
        if dist > 300:
            issues.append("CRITICAL: Outside geofence")
            self.state.geofence_ok = False
        elif dist > 250:
            issues.append("WARNING: Near geofence boundary")
            self.state.geofence_ok = True

        # Altitude check
        if abs(self.state.position_ned[2]) > 120:
            issues.append("CRITICAL: Above max altitude")

        # Weather check
        if self.state.humidity > 95 or self.state.pressure < 980:
            issues.append("WARNING: Bad weather conditions")
            self.state.weather_ok = False

        # Update safety status
        if any("CRITICAL" in i for i in issues):
            self.state.safety_status = SafetyStatus.CRITICAL
        elif any("WARNING" in i for i in issues):
            self.state.safety_status = SafetyStatus.WARNING
        else:
            self.state.safety_status = SafetyStatus.SAFE

        for issue in issues:
            logger.warning(f"⚠️ {issue}")

    # ------------------------------------------------------------------
    # Perception loop
    # ------------------------------------------------------------------

    async def perception_loop(self):
        """Run perception pipeline"""
        while self.running:
            await asyncio.sleep(0.033)  # ~30fps
            await self._run_perception()

    async def _run_perception(self):
        """Single perception cycle"""
        # In simulation, generate synthetic camera data
        if self.simulation:
            frames = self._generate_sim_frames()
        else:
            frames = []  # Real camera capture here

        # YOLO detection
        if self.detector and frames:
            detections = self.detector.detect(frames[0])
            self.state.tracked_targets = detections

        # SLAM update
        if self.slam:
            self.slam.update(
                frames[0] if frames else None,
                self.state.imu_accel,
                self.state.imu_gyro
            )
            self.state.slam_position = self.slam.get_position()
            self.state.slam_map_ready = True

    def _generate_sim_frames(self):
        """Generate simulated camera frames"""
        return [np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                for _ in range(4)]

    # ------------------------------------------------------------------
    # Fusion loop
    # ------------------------------------------------------------------

    async def fusion_loop(self):
        """Run sensor fusion"""
        while self.running:
            await asyncio.sleep(0.05)  # 20Hz
            self._run_fusion()

    def _run_fusion(self):
        """Single fusion cycle"""
        # Causal graph update
        if self.causal_graph:
            obs = {
                'temperature': self.state.temperature,
                'humidity': self.state.humidity,
                'pressure': self.state.pressure,
                'wind_speed': np.linalg.norm(self.state.velocity_ned[:2]),
                'altitude': abs(self.state.position_ned[2])
            }
            self.causal_graph.update(obs)

        # Temporal KG update
        if self.temporal_kg:
            event = {
                'timestamp': self.state.timestamp,
                'position': self.state.position_ned.tolist(),
                'targets': len(self.state.tracked_targets),
                'safety': self.state.safety_status.value
            }
            self.temporal_kg.add_event(event)

    # ------------------------------------------------------------------
    # Cognition loop
    # ------------------------------------------------------------------

    async def cognition_loop(self):
        """Run cognitive reasoning"""
        while self.running:
            await asyncio.sleep(0.2)  # 5Hz
            self._run_cognition()

    def _run_cognition(self):
        """Single cognition cycle"""
        # World model prediction
        if self.world_model:
            current_state = {
                'position': self.state.position_ned.tolist(),
                'velocity': self.state.velocity_ned.tolist(),
                'attitude': self.state.attitude_euler.tolist(),
            }
            prediction = self.world_model.predict(current_state)
            # Use prediction for planning

        # MoE routing
        if self.moe:
            task_embedding = np.random.randn(128)  # Simplified
            expert_output = self.moe.route(task_embedding)

    # ------------------------------------------------------------------
    # Voice interaction loop
    # ------------------------------------------------------------------

    async def voice_loop(self):
        """Voice assistant loop"""
        if not self.voice_assistant.is_available():
            logger.info("Voice assistant not available in simulation mode")
            return

        while self.running:
            await asyncio.sleep(0.1)
            command = await self.voice_assistant.listen()
            if command:
                response = await self.voice_assistant.process(command)
                logger.info(f"🎤 Voice: {command} → {response}")

    # ------------------------------------------------------------------
    # Gesture loop
    # ------------------------------------------------------------------

    async def gesture_loop(self):
        """Gesture recognition loop"""
        while self.running:
            await asyncio.sleep(0.033)  # 30fps
            if self.simulation:
                continue
            gesture = self.gesture_controller.detect()
            if gesture:
                cmd = self.gesture_controller.gesture_to_mavlink(gesture)
                if cmd and self.drone:
                    await self._send_mavlink_command(cmd)

    # ------------------------------------------------------------------
    # Flight control
    # ------------------------------------------------------------------

    async def takeoff(self, altitude: float = 5.0):
        """Takeoff to specified altitude"""
        if not self.drone:
            logger.info(f"[SIM] Takeoff to {altitude}m")
            self.state.position_ned[2] = -altitude
            self.state.mode = FlightMode.OFFBOARD
            return

        logger.info(f"Taking off to {altitude}m...")
        await self.drone.action.arm()

        await self.drone.offboard.set_position_ned(
            PositionNedYaw(0, 0, -altitude, 0)
        )

        try:
            await self.drone.offboard.start()
        except OffboardError as e:
            logger.error(f"Offboard start failed: {e}")
            await self.drone.action.disarm()
            return

        self.state.mode = FlightMode.OFFBOARD
        await asyncio.sleep(3)
        logger.info("✅ Takeoff complete")

    async def goto(self, north: float, east: float, down: float, yaw: float = 0.0):
        """Go to position (NED frame)"""
        if not self.drone:
            logger.info(f"[SIM] Goto N={north} E={east} D={down}")
            self.state.position_ned = np.array([north, east, down])
            return

        await self.drone.offboard.set_position_ned(
            PositionNedYaw(north, east, down, yaw)
        )
        await asyncio.sleep(2)

    async def land(self):
        """Land the drone"""
        if not self.drone:
            logger.info("[SIM] Landing")
            self.state.position_ned[2] = 0.0
            self.state.mode = FlightMode.DISARMED
            return

        try:
            await self.drone.offboard.stop()
        except OffboardError:
            pass
        await self.drone.action.land()
        self.state.mode = FlightMode.DISARMED
        logger.info("✅ Landing initiated")

    async def return_to_launch(self):
        """Return to launch point"""
        if not self.drone:
            logger.info("[SIM] RTL")
            self.state.position_ned = np.zeros(3)
            return

        try:
            await self.drone.offboard.stop()
        except OffboardError:
            pass
        await self.drone.action.return_to_launch()
        self.state.mode = FlightMode.RETURN_TO_LAUNCH
        logger.info("✅ RTL initiated")

    async def _send_mavlink_command(self, cmd: Dict):
        """Send a MAVLink command from gesture/voice"""
        cmd_type = cmd.get('type')
        if cmd_type == 'takeoff':
            await self.takeoff(cmd.get('altitude', 5.0))
        elif cmd_type == 'land':
            await self.land()
        elif cmd_type == 'goto':
            await self.goto(**cmd.get('params', {}))
        elif cmd_type == 'rtl':
            await self.return_to_launch()

    # ------------------------------------------------------------------
    # Follow mode
    # ------------------------------------------------------------------

    async def follow_loop(self):
        """Follow tracked target"""
        while self.running and self.state.mode == FlightMode.FOLLOW:
            await asyncio.sleep(0.1)
            if not self.state.tracked_targets:
                continue

            target = self.state.tracked_targets[0]
            target_pos = np.array(target.get('position', [5, 0, -3]))
            offset = np.array([3, 0, -2])  # 3m behind, 2m above
            desired = target_pos + offset

            if self.drone:
                await self.drone.offboard.set_position_ned(
                    PositionNedYaw(desired[0], desired[1], desired[2], 0)
                )
            else:
                self.state.position_ned = desired

    # ------------------------------------------------------------------
    # Main run loop
    # ------------------------------------------------------------------

    async def run(self):
        """Start all subsystems"""
        self.running = True
        logger.info("🔥 Phoenix starting...")

        # Connect to drone
        await self.connect()

        # Start sensor node
        self.sensor_node.start()

        # Start all loops
        self._tasks = [
            asyncio.create_task(self.telemetry_loop()),
            asyncio.create_task(self.safety_loop()),
            asyncio.create_task(self.perception_loop()),
            asyncio.create_task(self.fusion_loop()),
            asyncio.create_task(self.cognition_loop()),
            asyncio.create_task(self.voice_loop()),
            asyncio.create_task(self.gesture_loop()),
            asyncio.create_task(self.follow_loop()),
        ]

        logger.info("✅ All subsystems running")

        # Wait for shutdown signal
        try:
            while self.running:
                await asyncio.sleep(1)
        except asyncio.CancelledError:
            pass

        await self.shutdown()

    async def shutdown(self):
        """Gracefully shutdown all subsystems"""
        logger.info("🛑 Phoenix shutting down...")
        self.running = False

        # Cancel all tasks
        for task in self._tasks:
            task.cancel()

        # Stop sensor node
        self.sensor_node.stop()

        # Land if armed
        if self.state.armed and self.drone:
            await self.land()

        logger.info("✅ Phoenix shutdown complete")


# ---------------------------------------------------------------------------
# CLI Entry Point
# ---------------------------------------------------------------------------

async def main():
    """Main entry point"""
    import argparse

    parser = argparse.ArgumentParser(description="LeoDrone Phoenix")
    parser.add_argument("--connection", default="udp://:14540",
                       help="MAVLink connection URL")
    parser.add_argument("--sim", action="store_true", default=True,
                       help="Run in simulation mode")
    parser.add_argument("--log-level", default="INFO",
                       choices=["DEBUG", "INFO", "WARNING", "ERROR"])
    args = parser.parse_args()

    logging.getLogger().setLevel(getattr(logging, args.log_level))

    controller = PhoenixController(
        connection_url=args.connection,
        simulation=args.sim
    )

    # Handle signals
    loop = asyncio.get_event_loop()
    for sig in (signal.SIGINT, signal.SIGTERM):
        loop.add_signal_handler(sig, lambda: asyncio.create_task(
            controller.shutdown()))

    await controller.run()


if __name__ == "__main__":
    asyncio.run(main())
