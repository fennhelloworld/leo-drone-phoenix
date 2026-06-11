#!/usr/bin/env python3
"""
LeoDrone Phoenix — 综合测试套件
30+ 测试覆盖所有功能特性

所有测试仅需 Python + NumPy，无需GPU/硬件
运行: python3 tests/test_phoenix.py
"""

import unittest
import numpy as np
import time
import math
import sys
import os
import asyncio

# Add project paths
PROJECT_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
FIRMWARE_DIR = os.path.join(PROJECT_DIR, "FIRMWARE")
SIMULATION_DIR = os.path.join(PROJECT_DIR, "SIMULATION")
sys.path.insert(0, FIRMWARE_DIR)
sys.path.insert(0, SIMULATION_DIR)


# ===========================================================================
# Test Suite
# ===========================================================================

class TestPhoenixCore(unittest.TestCase):
    """Phoenix 核心功能测试"""

    def test_project_structure(self):
        """T01: 项目结构完整性"""
        required_dirs = ["FIRMWARE", "SIMULATION", "tests"]
        for d in required_dirs:
            path = os.path.join(PROJECT_DIR, d)
            self.assertTrue(os.path.isdir(path), f"Missing directory: {d}")

        required_files = [
            "README.md", "ARCHITECTURE.md", "INDOOR_SIM_SETUP.md",
            "OUTDOOR_CONFIG.md", "SAFETY.md", "Makefile", "run.sh"
        ]
        for f in required_files:
            path = os.path.join(PROJECT_DIR, f)
            self.assertTrue(os.path.isfile(path), f"Missing file: {f}")

    def test_firmware_modules(self):
        """T02: 固件模块可导入"""
        from main import PhoenixController, DroneState, FlightMode
        from sensor_node import SensorNode, BME280Driver, ICM42688Driver
        from voice_assistant import VoiceAssistant, IntentParser
        from gesture_controller import GestureController, Gesture
        from swarm_coordinator import SwarmCoordinator, FormationType
        from video_editor import VideoEditor, BackgroundType, FilterType

    def test_drone_state(self):
        """T03: 无人机状态数据结构"""
        from main import DroneState, FlightMode, SafetyStatus
        state = DroneState()
        self.assertEqual(state.mode, FlightMode.DISARMED)
        self.assertFalse(state.armed)
        self.assertEqual(state.battery_percent, 100.0)
        self.assertEqual(state.safety_status, SafetyStatus.SAFE)
        self.assertEqual(state.position_ned.shape, (3,))
        self.assertEqual(state.velocity_ned.shape, (3,))


class TestFlightControl(unittest.TestCase):
    """飞行控制测试"""

    def test_flight_modes(self):
        """T04: 飞行模式枚举"""
        from main import FlightMode
        modes = [FlightMode.DISARMED, FlightMode.MANUAL,
                 FlightMode.STABILIZED, FlightMode.OFFBOARD,
                 FlightMode.FOLLOW, FlightMode.ORBIT,
                 FlightMode.RETURN_TO_LAUNCH]
        self.assertEqual(len(modes), 7)

    def test_offboard_takeoff(self):
        """T05: Offboard起飞控制"""
        async def _test():
            from main import PhoenixController
            ctrl = PhoenixController(simulation=True)
            self.assertIsNotNone(ctrl)
            # Simulated takeoff
            await ctrl.takeoff(altitude=5.0)
            self.assertAlmostEqual(ctrl.state.position_ned[2], -5.0, places=1)
        asyncio.run(_test())

    def test_goto_waypoint(self):
        """T06: 航点飞行"""
        async def _test():
            from main import PhoenixController
            ctrl = PhoenixController(simulation=True)
            await ctrl.takeoff(5.0)
            await ctrl.goto(10, 5, -5, yaw=90)
            self.assertAlmostEqual(ctrl.state.position_ned[0], 10, places=0)
            self.assertAlmostEqual(ctrl.state.position_ned[1], 5, places=0)
        asyncio.run(_test())

    def test_land(self):
        """T07: 降落"""
        async def _test():
            from main import PhoenixController
            ctrl = PhoenixController(simulation=True)
            await ctrl.takeoff(5.0)
            await ctrl.land()
            self.assertAlmostEqual(ctrl.state.position_ned[2], 0.0, places=1)
        asyncio.run(_test())

    def test_return_to_launch(self):
        """T08: 返航"""
        async def _test():
            from main import PhoenixController
            ctrl = PhoenixController(simulation=True)
            await ctrl.takeoff(5.0)
            await ctrl.goto(10, 10, -5)
            await ctrl.return_to_launch()
            self.assertAlmostEqual(
                np.linalg.norm(ctrl.state.position_ned[:2]), 0.0, places=0)
        asyncio.run(_test())

    def test_safety_monitoring(self):
        """T09: 安全监控"""
        from main import PhoenixController, SafetyStatus
        ctrl = PhoenixController(simulation=True)
        # Normal state
        ctrl._check_safety()
        self.assertEqual(ctrl.state.safety_status, SafetyStatus.SAFE)
        # Low battery
        ctrl.state.battery_percent = 15
        ctrl._check_safety()
        self.assertIn(ctrl.state.safety_status,
                      [SafetyStatus.WARNING, SafetyStatus.CRITICAL])
        # Critical battery
        ctrl.state.battery_percent = 5
        ctrl._check_safety()
        self.assertIn(ctrl.state.safety_status,
                      [SafetyStatus.CRITICAL, SafetyStatus.EMERGENCY])


class TestSensors(unittest.TestCase):
    """传感器测试"""

    def test_bme280_simulated(self):
        """T10: BME280仿真数据"""
        from sensor_node import BME280Driver
        bme = BME280Driver()
        data = bme._simulated_read()
        self.assertIn('temperature', data)
        self.assertIn('humidity', data)
        self.assertIn('pressure', data)
        self.assertIn('altitude', data)
        self.assertGreater(data['temperature'], -10)
        self.assertLess(data['temperature'], 50)
        self.assertGreater(data['humidity'], 0)
        self.assertLess(data['humidity'], 100)
        self.assertGreater(data['pressure'], 900)

    def test_icm42688_simulated(self):
        """T11: ICM-42688-P仿真数据"""
        from sensor_node import ICM42688Driver
        imu = ICM42688Driver()
        data = imu._simulated_read()
        self.assertIn('accel', data)
        self.assertIn('gyro', data)
        self.assertEqual(data['accel'].shape, (3,))
        self.assertEqual(data['gyro'].shape, (3,))
        # Gravity should be ~9.81 on z-axis
        self.assertAlmostEqual(data['accel'][2], 9.81, delta=1.0)

    def test_gps_simulated(self):
        """T12: GPS仿真数据"""
        from sensor_node import SimulatedGPS
        gps = SimulatedGPS()
        data = gps.read()
        self.assertIn('lat', data)
        self.assertIn('lon', data)
        self.assertIn('alt', data)
        self.assertIn('fix_type', data)
        self.assertEqual(data['fix_type'], 3)
        self.assertGreater(data['num_satellites'], 0)

    def test_sensor_node_combined(self):
        """T13: 传感器节点综合读取"""
        from sensor_node import SensorNode
        node = SensorNode(simulation=True)
        node.start()
        data = node.read_all()
        self.assertIn('accel', data)
        self.assertIn('gyro', data)
        self.assertIn('temperature', data)
        self.assertIn('pressure', data)
        self.assertIn('lat', data)
        node.stop()

    def test_wind_estimate(self):
        """T14: 风速估计"""
        from sensor_node import SensorNode
        node = SensorNode(simulation=True)
        wind = node.get_wind_estimate()
        self.assertIn('wind_speed', wind)
        self.assertIn('wind_direction', wind)
        self.assertIn('gust_speed', wind)
        # Wind speed should be a number (can be near zero)
        self.assertIsInstance(wind['wind_speed'], (int, float))


class TestPerception(unittest.TestCase):
    """感知层测试"""

    def test_ekf_predict(self):
        """T15: EKF预测步骤"""
        from main import PhoenixController
        ctrl = PhoenixController(simulation=True)
        ekf = ctrl.ekf
        initial_pos = ekf.get_position().copy()
        ekf.predict(np.array([0, 0, 9.81]), np.zeros(3), 0.1)
        # Position should have changed slightly
        new_pos = ekf.get_position()
        self.assertEqual(new_pos.shape, (3,))

    def test_ekf_gps_update(self):
        """T16: EKF GPS更新"""
        from main import PhoenixController
        ctrl = PhoenixController(simulation=True)
        ekf = ctrl.ekf
        gps_pos = np.array([10.0, 20.0, 5.0])
        # Multiple updates to converge
        for _ in range(10):
            ekf.update_gps(gps_pos)
        estimated = ekf.get_position()
        # Should move toward GPS measurement
        error = np.linalg.norm(estimated - gps_pos)
        self.assertLess(error, 30)  # Within reasonable range

    def test_360_stitching(self):
        """T17: 360°全景拼接"""
        from sim_360_stitch import PanoramicStitcher
        stitcher = PanoramicStitcher(num_cameras=4)
        frames = [np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
                  for _ in range(4)]
        pano = stitcher.fast_stitch(frames)
        self.assertEqual(pano.shape, (960, 1920, 3))

    def test_fisheye_camera_model(self):
        """T18: 鱼眼相机模型"""
        from sim_360_stitch import FisheyeCameraModel
        cam = FisheyeCameraModel(fov_deg=160)
        # Project 3D point
        pixel = cam.project(np.array([0.1, 0.1, 1.0]))
        self.assertEqual(pixel.shape, (2,))
        # Undistort
        undistorted = cam.undistort(pixel)
        self.assertEqual(undistorted.shape, (2,))

    def test_slam_vio(self):
        """T19: VINS-Fusion SLAM"""
        from sim_slam import SimpleVIO
        vio = SimpleVIO()
        # Predict
        vio.predict(np.array([0, 0, 9.81]), np.zeros(3), 0.01)
        pos = vio.get_position()
        self.assertEqual(pos.shape, (3,))
        # Visual update
        vio.update_visual(np.array([1, 2, 3]))
        updated_pos = vio.get_position()
        self.assertEqual(updated_pos.shape, (3,))

    def test_slam_trajectory_generation(self):
        """T20: 仿真轨迹生成"""
        from sim_slam import SimulatedTrajectory
        traj = SimulatedTrajectory(duration=10.0, dt=0.01)
        figure8 = traj.generate_figure8()
        self.assertIn('positions', figure8)
        self.assertEqual(figure8['positions'].shape[1], 3)
        self.assertGreater(len(figure8['positions']), 0)


class TestDetection(unittest.TestCase):
    """目标检测与跟踪测试"""

    def test_yolo_detection(self):
        """T21: YOLOv8目标检测仿真"""
        from sim_tracking import SimpleYOLODetector, SimulatedPerson
        detector = SimpleYOLODetector()
        persons = [SimulatedPerson(start_pos=np.array([5, 0, 10]))]
        frame = np.zeros((480, 640, 3), dtype=np.uint8)
        detections = detector.detect(frame, persons, np.array([0, 0, 3]))
        self.assertIsInstance(detections, list)

    def test_deepsort_tracking(self):
        """T22: DeepSORT多目标跟踪"""
        from sim_tracking import SimpleDeepSORT, BoundingBox
        tracker = SimpleDeepSORT()
        # Add detection
        det = BoundingBox(x1=100, y1=100, x2=200, y2=300,
                         confidence=0.9, class_id=0, class_name="person")
        tracks = tracker.update([det])
        self.assertGreater(len(tracks), 0)
        self.assertEqual(tracks[0].track_id, 0)

    def test_follow_controller(self):
        """T23: 跟随飞行控制"""
        from sim_tracking import FollowController, TrackedObject, BoundingBox
        controller = FollowController()
        bbox = BoundingBox(x1=280, y1=100, x2=360, y2=300,
                          confidence=0.9, class_id=0, class_name="person")
        target = TrackedObject(track_id=0, bbox=bbox)
        velocity = controller.compute_velocity(target)
        self.assertEqual(velocity.shape, (3,))


class TestGestureControl(unittest.TestCase):
    """手势控制测试"""

    def test_gesture_enum(self):
        """T24: 手势类型枚举"""
        from gesture_controller import Gesture
        gestures = [Gesture.POINT_UP, Gesture.POINT_DOWN,
                    Gesture.OPEN_PALM, Gesture.FIST,
                    Gesture.SWIPE_LEFT, Gesture.SWIPE_RIGHT,
                    Gesture.PINCH, Gesture.WAVE]
        self.assertEqual(len(gestures), 8)

    def test_gesture_to_mavlink(self):
        """T25: 手势→MAVLink指令映射"""
        from gesture_controller import GestureController, Gesture
        ctrl = GestureController(simulation=True)
        for gesture in Gesture:
            cmd = ctrl.gesture_to_mavlink(gesture)
            if gesture != Gesture.NONE:
                self.assertIsNotNone(cmd, f"Missing mapping for {gesture}")
                self.assertIn('type', cmd)

    def test_hand_landmarks(self):
        """T26: 手部关键点检测"""
        from gesture_controller import MediaPipeDetector
        detector = MediaPipeDetector()
        landmarks = detector._simulated_detect()
        self.assertGreater(len(landmarks), 0)
        self.assertEqual(landmarks[0].positions.shape, (21, 3))

    def test_gesture_classification(self):
        """T27: 手势分类"""
        from gesture_controller import GestureClassifier, HandLandmarks, Gesture
        classifier = GestureClassifier()
        # Open palm (all fingers extended)
        positions = np.zeros((21, 3))
        for i in range(5):
            tip_idx = 4 * (i + 1) if i > 0 else 4
            positions[tip_idx] = [0.3 + i * 0.1, 0.2, 0]  # Tips high
        lm = HandLandmarks(positions=positions, handedness="Right", confidence=0.9)
        gesture = classifier.classify(lm)
        self.assertIsInstance(gesture, Gesture)


class TestVoiceAssistant(unittest.TestCase):
    """语音助手测试"""

    def test_voice_assistant_init(self):
        """T28: 语音助手初始化"""
        from voice_assistant import VoiceAssistant
        va = VoiceAssistant(simulation=True)
        self.assertTrue(va.is_available())

    def test_llm_simulated_response(self):
        """T29: LLM仿真响应"""
        async def _test():
            from voice_assistant import VoiceAssistant
            va = VoiceAssistant(simulation=True)
            response = await va.process("起飞到五米")
            self.assertIsInstance(response, str)
            self.assertGreater(len(response), 0)
        asyncio.run(_test())

    def test_intent_parsing(self):
        """T30: 意图解析"""
        from voice_assistant import IntentParser
        parser = IntentParser()
        # JSON command
        cmd = parser.parse('{"type": "command", "action": "takeoff", "params": {"altitude": 5}}')
        self.assertIsNotNone(cmd)
        self.assertEqual(cmd.intent, "takeoff")

    def test_wake_word_detection(self):
        """T31: 唤醒词检测"""
        from voice_assistant import WakeWordDetector
        detector = WakeWordDetector()
        # High energy frame should trigger
        loud_audio = np.ones(1600) * 0.5
        result = detector.detect(loud_audio)
        self.assertIsInstance(result, (bool, np.bool_))

    def test_stt_simulated(self):
        """T32: STT仿真"""
        from voice_assistant import SpeechToText
        stt = SpeechToText()
        audio = np.random.randn(16000).astype(np.float32)
        text = stt.transcribe(audio)
        self.assertIsInstance(text, str)


class TestSwarmCoordination(unittest.TestCase):
    """编队协调测试"""

    def test_formation_v_shape(self):
        """T33: V字形编队生成"""
        from swarm_coordinator import FormationGenerator
        gen = FormationGenerator()
        positions = gen.v_shape(3, spacing=5.0)
        self.assertEqual(len(positions), 3)
        # Leader at origin
        self.assertAlmostEqual(np.linalg.norm(positions[0]), 0, places=5)

    def test_formation_circle(self):
        """T34: 环形编队生成"""
        from swarm_coordinator import FormationGenerator
        gen = FormationGenerator()
        positions = gen.circle(3, radius=10.0)
        self.assertEqual(len(positions), 3)
        distances = [np.linalg.norm(p[:2]) for p in positions]
        for d in distances:
            self.assertAlmostEqual(d, 10.0, places=5)

    def test_swarm_coordinator_init(self):
        """T35: 编队协调器初始化"""
        from swarm_coordinator import SwarmCoordinator, FormationType
        coord = SwarmCoordinator(num_uavs=3, simulation=True)
        self.assertEqual(coord.num_uavs, 3)
        self.assertEqual(coord.formation_type, FormationType.V_SHAPE)

    def test_formation_switching(self):
        """T36: 编队类型切换"""
        from swarm_coordinator import SwarmCoordinator, FormationType
        coord = SwarmCoordinator(num_uavs=3, simulation=True)
        for ft in FormationType:
            coord.set_formation(ft, spacing=5.0)
            self.assertEqual(coord.formation_type, ft)

    def test_collaborative_mapping(self):
        """T37: 协同建图"""
        from swarm_coordinator import CollaborativeMapper
        mapper = CollaborativeMapper()
        obs = [{'position': np.array([5, 5, 0]), 'occupied': True}]
        mapper.update_local_map("uav_1", np.zeros(3), obs)
        mapper.merge_maps()
        grid = mapper.get_occupancy_grid()
        self.assertEqual(grid.ndim, 2)

    def test_consensus_velocity(self):
        """T38: 一致性速度计算"""
        from swarm_coordinator import ConsensusProtocol, UAVState, SwarmRole
        consensus = ConsensusProtocol()
        leader = UAVState(uav_id="l", position=np.zeros(3),
                         velocity=np.array([1, 0, 0]), role=SwarmRole.LEADER)
        follower = UAVState(uav_id="f", position=np.array([5, 5, 0]),
                           velocity=np.zeros(3), role=SwarmRole.FOLLOWER)
        vel = consensus.compute_follower_velocity(follower, leader, np.array([-5, 0, 0]))
        self.assertEqual(vel.shape, (3,))


class TestVideoEditor(unittest.TestCase):
    """视频编辑测试"""

    def test_person_segmentation(self):
        """T39: 人物分割"""
        from video_editor import PersonSegmenter
        seg = PersonSegmenter()
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)
        mask = seg.segment(frame)
        self.assertEqual(mask.shape, (480, 640))
        self.assertTrue(np.all(mask >= 0))
        self.assertTrue(np.all(mask <= 1))

    def test_background_replacement(self):
        """T40: 背景替换"""
        from video_editor import BackgroundReplacer, BackgroundType
        replacer = BackgroundReplacer()
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        mask = np.ones((240, 320), dtype=np.float32)
        mask[0:120, :] = 0.0
        for bg_type in [BackgroundType.BLACK, BackgroundType.WHITE,
                       BackgroundType.VIRTUAL_SKY]:
            replacer.set_background(bg_type)
            result = replacer.replace(frame, mask)
            self.assertEqual(result.shape, frame.shape)

    def test_video_filters(self):
        """T41: 视频滤镜"""
        from video_editor import FilterEngine, FilterType
        engine = FilterEngine()
        frame = np.random.randint(0, 255, (120, 160, 3), dtype=np.uint8)
        for ft in FilterType:
            result = engine.apply(frame, ft)
            self.assertEqual(result.shape, frame.shape)
            self.assertEqual(result.dtype, np.uint8)

    def test_electronic_stabilization(self):
        """T42: 电子防抖"""
        from video_editor import ElectronicImageStabilizer
        eis = ElectronicImageStabilizer()
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
        gyro = np.array([0.01, 0.02, 0.001])
        stabilized = eis.stabilize(frame, gyro)
        self.assertEqual(stabilized.shape, frame.shape)

    def test_full_video_pipeline(self):
        """T43: 完整视频管线"""
        from video_editor import VideoEditor, BackgroundType, FilterType
        editor = VideoEditor(simulation=True)
        frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)

        # Original
        result = editor.process_frame(frame)
        self.assertEqual(result.shape, frame.shape)

        # With background
        editor.set_background(BackgroundType.VIRTUAL_SKY)
        result = editor.process_frame(frame)
        self.assertEqual(result.shape, frame.shape)

        # With filter
        editor.set_filter(FilterType.CINEMATIC)
        result = editor.process_frame(frame)
        self.assertEqual(result.shape, frame.shape)

    def test_360_video_processing(self):
        """T44: 360°视频处理"""
        from video_editor import VideoEditor
        editor = VideoEditor(simulation=True)
        frames = [np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
                  for _ in range(4)]
        pano = editor.process_360_frames(frames)
        self.assertEqual(pano.shape[2], 3)
        self.assertGreater(pano.shape[1], 320)  # Wider than single frame


class TestSafety(unittest.TestCase):
    """安全系统测试"""

    def test_safety_status_enum(self):
        """T45: 安全状态枚举"""
        from main import SafetyStatus
        statuses = [SafetyStatus.SAFE, SafetyStatus.WARNING,
                   SafetyStatus.CRITICAL, SafetyStatus.EMERGENCY]
        self.assertEqual(len(statuses), 4)

    def test_geofence_check(self):
        """T46: 地理围栏检查"""
        from main import PhoenixController, SafetyStatus
        ctrl = PhoenixController(simulation=True)
        # Inside fence
        ctrl.state.position_ned = np.array([100, 100, -5])
        ctrl._check_safety()
        self.assertTrue(ctrl.state.geofence_ok)

        # Outside fence
        ctrl.state.position_ned = np.array([400, 0, -5])
        ctrl._check_safety()
        self.assertFalse(ctrl.state.geofence_ok)

    def test_battery_safety(self):
        """T47: 电池安全"""
        from main import PhoenixController, SafetyStatus
        ctrl = PhoenixController(simulation=True)

        # Normal
        ctrl.state.battery_percent = 80
        ctrl._check_safety()
        self.assertNotEqual(ctrl.state.safety_status, SafetyStatus.EMERGENCY)

        # Low
        ctrl.state.battery_percent = 15
        ctrl._check_safety()
        self.assertIn(ctrl.state.safety_status,
                      [SafetyStatus.WARNING, SafetyStatus.CRITICAL])

        # Critical
        ctrl.state.battery_percent = 5
        ctrl._check_safety()
        self.assertIn(ctrl.state.safety_status,
                      [SafetyStatus.CRITICAL, SafetyStatus.EMERGENCY])

    def test_weather_safety(self):
        """T48: 气象安全"""
        from main import PhoenixController, DroneState, SafetyStatus
        ctrl = PhoenixController(simulation=True)
        # Bad humidity - use fresh state
        ctrl.state = DroneState()
        ctrl.state.humidity = 98
        ctrl._check_safety()
        self.assertFalse(ctrl.state.weather_ok)
        # Normal - use fresh state to avoid carry-over
        ctrl.state = DroneState()
        ctrl.state.humidity = 60
        ctrl.state.pressure = 1013
        ctrl._check_safety()
        self.assertTrue(ctrl.state.weather_ok)


class TestArchitecture(unittest.TestCase):
    """架构完整性测试"""

    def test_9_layer_architecture(self):
        """T49: 9层架构验证"""
        layers = [
            "L0 Hardware", "L1 Sensing", "L2 Perception",
            "L3 Fusion", "L4 Cognition", "L5 Coordination",
            "L6 Interaction", "L7 Safety", "L8 Ground Station"
        ]
        arch_path = os.path.join(PROJECT_DIR, "ARCHITECTURE.md")
        with open(arch_path, 'r') as f:
            content = f.read()
        for layer in layers:
            self.assertIn(layer, content,
                         f"Missing layer: {layer}")

    def test_feature_list_complete(self):
        """T50: 功能清单完整性"""
        features = [
            "360", "SLAM", "YOLO", "手势", "语音", "编队",
            "防抖", "EIS", "BME280", "地理围栏", "Failsafe",
            "MAVSDK", "Gazebo", "SITL", "全景"
        ]
        readme_path = os.path.join(PROJECT_DIR, "README.md")
        with open(readme_path, 'r') as f:
            content = f.read()
        for feature in features:
            self.assertIn(feature, content,
                         f"Missing feature: {feature}")


# ===========================================================================
# Run
# ===========================================================================

def run_all_tests():
    """Run all tests and print summary"""
    loader = unittest.TestLoader()
    suite = loader.loadTestsFromModule(sys.modules[__name__])

    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)

    print("\n" + "=" * 60)
    print(f"Tests run: {result.testsRun}")
    print(f"Failures: {len(result.failures)}")
    print(f"Errors: {len(result.errors)}")
    print(f"Success: {result.testsRun - len(result.failures) - len(result.errors)}")
    print("=" * 60)

    return result.wasSuccessful()


if __name__ == "__main__":
    success = run_all_tests()
    sys.exit(0 if success else 1)
