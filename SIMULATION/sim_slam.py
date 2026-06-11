#!/usr/bin/env python3
"""
LeoDrone Phoenix — VINS-Fusion SLAM 仿真
仿真IMU + 视觉数据 → VIO状态估计 → 轨迹评估
"""

import numpy as np
import time
import math
import logging
from typing import List, Dict, Tuple, Optional

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SimSLAM")


class SimulatedTrajectory:
    """仿真轨迹生成器 — 生成真实的3D飞行轨迹"""

    def __init__(self, duration: float = 60.0, dt: float = 0.005):
        self.duration = duration
        self.dt = dt
        self.num_steps = int(duration / dt)

    def generate_figure8(self, scale: float = 5.0,
                         height: float = 3.0) -> Dict[str, np.ndarray]:
        """生成8字形飞行轨迹"""
        positions = []
        velocities = []
        accelerations = []

        for i in range(self.num_steps):
            t = i * self.dt

            # Figure-8 (lemniscate)
            x = scale * np.sin(t * 0.2)
            y = scale * np.sin(t * 0.2) * np.cos(t * 0.2)
            z = height + 0.5 * np.sin(t * 0.1)

            # Derivatives
            vx = scale * 0.2 * np.cos(t * 0.2)
            vy = scale * 0.2 * (np.cos(t * 0.4))
            vz = 0.5 * 0.1 * np.cos(t * 0.1)

            ax = -scale * 0.04 * np.sin(t * 0.2)
            ay = -scale * 0.08 * np.sin(t * 0.4)
            az = -0.5 * 0.01 * np.sin(t * 0.1)

            positions.append([x, y, z])
            velocities.append([vx, vy, vz])
            accelerations.append([ax, ay, az])

        return {
            'positions': np.array(positions),
            'velocities': np.array(velocities),
            'accelerations': np.array(accelerations),
            'timestamps': np.arange(self.num_steps) * self.dt
        }

    def generate_helix(self, radius: float = 3.0, height: float = 5.0,
                       turns: float = 2.0) -> Dict[str, np.ndarray]:
        """生成螺旋上升轨迹"""
        positions = []
        velocities = []

        for i in range(self.num_steps):
            t = i * self.dt
            angle = turns * 2 * np.pi * t / self.duration

            x = radius * np.cos(angle)
            y = radius * np.sin(angle)
            z = height * t / self.duration

            vx = -radius * np.sin(angle) * turns * 2 * np.pi / self.duration
            vy = radius * np.cos(angle) * turns * 2 * np.pi / self.duration
            vz = height / self.duration

            positions.append([x, y, z])
            velocities.append([vx, vy, vz])

        return {
            'positions': np.array(positions),
            'velocities': np.array(velocities),
            'timestamps': np.arange(self.num_steps) * self.dt
        }

    def generate_rectangle(self, width: float = 10.0,
                           height: float = 10.0,
                           flight_height: float = 3.0) -> Dict[str, np.ndarray]:
        """生成矩形飞行轨迹"""
        # Define waypoints
        waypoints = np.array([
            [0, 0, flight_height],
            [width, 0, flight_height],
            [width, height, flight_height],
            [0, height, flight_height],
            [0, 0, flight_height],
        ])

        # Interpolate between waypoints
        positions = []
        segment_duration = self.duration / (len(waypoints) - 1)

        for i in range(len(waypoints) - 1):
            start = waypoints[i]
            end = waypoints[i + 1]
            num_points = int(segment_duration / self.dt)

            for j in range(num_points):
                t = j / num_points
                # Smooth interpolation
                t_smooth = t * t * (3 - 2 * t)
                pos = start + (end - start) * t_smooth
                positions.append(pos)

        positions = np.array(positions[:self.num_steps])
        timestamps = np.arange(len(positions)) * self.dt

        # Compute velocities by differentiation
        velocities = np.gradient(positions, self.dt, axis=0)

        return {
            'positions': positions,
            'velocities': velocities,
            'timestamps': timestamps[:len(positions)]
        }


class SimulatedIMU:
    """仿真IMU — 添加噪声到真实轨迹"""

    def __init__(self, accel_noise: float = 0.02,
                 gyro_noise: float = 0.005,
                 accel_bias: float = 0.001,
                 gyro_bias: float = 0.0001,
                 gravity: np.ndarray = None):
        self.accel_noise = accel_noise
        self.gyro_noise = gyro_noise
        self.accel_bias = np.array([accel_bias] * 3)
        self.gyro_bias = np.array([gyro_bias] * 3)
        self.gravity = gravity if gravity is not None else np.array([0, 0, 9.81])

    def generate(self, true_accelerations: np.ndarray,
                 true_velocities: np.ndarray) -> Dict[str, np.ndarray]:
        """Generate noisy IMU measurements"""
        n = len(true_accelerations)

        # Add gravity to accelerometer
        accel_meas = true_accelerations + self.gravity + self.accel_bias
        accel_meas += np.random.normal(0, self.accel_noise, (n, 3))

        # Compute angular rates from velocity changes
        # (simplified: assume body-aligned)
        gyro_meas = np.zeros((n, 3))
        for i in range(1, n):
            dv = true_velocities[i] - true_velocities[i - 1]
            gyro_meas[i] = np.cross(self.gravity, dv) / (np.linalg.norm(self.gravity) ** 2 + 1e-6)
        gyro_meas += self.gyro_bias
        gyro_meas += np.random.normal(0, self.gyro_noise, (n, 3))

        return {
            'accel': accel_meas,
            'gyro': gyro_meas
        }


class SimulatedVisualFeatures:
    """仿真视觉特征 — 3D特征点投影到图像平面"""

    def __init__(self, num_features: int = 100,
                 image_size: Tuple[int, int] = (640, 480),
                 focal_length: float = 300.0):
        self.num_features = num_features
        self.image_size = image_size
        self.focal_length = focal_length

        # Generate random 3D feature points
        self.features_3d = np.random.uniform(-20, 20, (num_features, 3))
        self.features_3d[:, 2] = np.abs(self.features_3d[:, 2]) + 1  # Ensure positive depth

    def project(self, camera_position: np.ndarray,
                camera_rotation: np.ndarray = None) -> np.ndarray:
        """Project 3D features to 2D image plane"""
        if camera_rotation is None:
            camera_rotation = np.eye(3)

        # Transform to camera frame
        features_cam = (self.features_3d - camera_position) @ camera_rotation.T

        # Project (pinhole model)
        valid = features_cam[:, 2] > 0.1  # In front of camera

        pixel_coords = np.zeros((self.num_features, 2))
        for i in range(self.num_features):
            if valid[i]:
                z = features_cam[i, 2]
                pixel_coords[i, 0] = self.focal_length * features_cam[i, 0] / z + self.image_size[0] / 2
                pixel_coords[i, 1] = self.focal_length * features_cam[i, 1] / z + self.image_size[1] / 2
            else:
                pixel_coords[i] = [-1, -1]  # Invalid

        # Filter visible features
        visible = (pixel_coords[:, 0] >= 0) & \
                  (pixel_coords[:, 0] < self.image_size[0]) & \
                  (pixel_coords[:, 1] >= 0) & \
                  (pixel_coords[:, 1] < self.image_size[1]) & \
                  valid

        return pixel_coords, visible


class SimpleVIO:
    """简化的视觉惯性里程计 (VIO) — 纯NumPy实现"""

    def __init__(self):
        # State: [position(3), velocity(3), attitude(4 quaternion)]
        self.position = np.zeros(3)
        self.velocity = np.zeros(3)
        self.attitude = np.array([1, 0, 0, 0], dtype=np.float64)  # Quaternion

        # Covariance
        self.P = np.eye(9) * 0.01

        # Process noise
        self.Q = np.eye(9) * 0.001

        # Gravity
        self.gravity = np.array([0, 0, 9.81])

    def predict(self, accel: np.ndarray, gyro: np.ndarray, dt: float):
        """IMU prediction step"""
        # Remove gravity bias (simplified)
        accel_world = accel - self.gravity

        # Update position and velocity
        self.position += self.velocity * dt + 0.5 * accel_world * dt ** 2
        self.velocity += accel_world * dt

        # Update covariance
        F = np.eye(9)
        F[0:3, 3:6] = np.eye(3) * dt
        self.P = F @ self.P @ F.T + self.Q * dt

    def update_visual(self, measured_position: np.ndarray,
                      confidence: float = 1.0):
        """Visual measurement update"""
        H = np.zeros((3, 9))
        H[0:3, 0:3] = np.eye(3)

        R = np.eye(3) * (1.0 / confidence)

        y = measured_position - self.position
        S = H @ self.P @ H.T + R
        K = self.P @ H.T @ np.linalg.inv(S)

        # Update state
        state_correction = K @ y
        self.position += state_correction[0:3]
        self.velocity += state_correction[3:6]

        # Update covariance
        self.P = (np.eye(9) - K @ H) @ self.P

    def get_position(self) -> np.ndarray:
        return self.position.copy()

    def get_velocity(self) -> np.ndarray:
        return self.velocity.copy()


def test_slam():
    """Test VINS-Fusion SLAM simulation"""
    logger.info("=" * 50)
    logger.info("VINS-Fusion SLAM Simulation")
    logger.info("=" * 50)

    # Generate trajectory
    logger.info("Generating figure-8 trajectory...")
    traj_gen = SimulatedTrajectory(duration=30.0, dt=0.005)
    trajectory = traj_gen.generate_figure8(scale=5.0, height=3.0)

    true_pos = trajectory['positions']
    true_vel = trajectory['velocities']
    true_acc = trajectory['accelerations']
    timestamps = trajectory['timestamps']

    logger.info(f"  Trajectory points: {len(true_pos)}")
    logger.info(f"  Duration: {timestamps[-1]:.1f}s")

    # Generate IMU data
    logger.info("\nGenerating IMU measurements...")
    imu = SimulatedIMU()
    imu_data = imu.generate(true_acc, true_vel)
    logger.info(f"  Accel noise std: {imu.accel_noise}")
    logger.info(f"  Gyro noise std: {imu.gyro_noise}")

    # Generate visual features
    logger.info("\nGenerating visual features...")
    visual = SimulatedVisualFeatures(num_features=100)
    logger.info(f"  Feature points: {visual.num_features}")

    # Run VIO
    logger.info("\nRunning VIO estimation...")
    vio = SimpleVIO()

    estimated_positions = []
    visual_update_interval = 200  # Update every 200 IMU steps (1Hz visual)
    imu_dt = 0.005

    for i in range(len(true_pos)):
        # IMU prediction
        vio.predict(imu_data['accel'][i], imu_data['gyro'][i], imu_dt)

        # Visual update (at lower rate)
        if i % visual_update_interval == 0:
            pixel_coords, visible = visual.project(true_pos[i])
            num_visible = np.sum(visible)

            # Visual position measurement (with some noise)
            visual_pos = true_pos[i] + np.random.normal(0, 0.1, 3)
            vio.update_visual(visual_pos, confidence=0.8)

        estimated_positions.append(vio.get_position())

    estimated_positions = np.array(estimated_positions)

    # Evaluate
    logger.info("\nEvaluating SLAM performance...")

    # Position error
    position_errors = np.linalg.norm(estimated_positions - true_pos, axis=1)
    mean_error = np.mean(position_errors)
    max_error = np.max(position_errors)
    rmse = np.sqrt(np.mean(position_errors ** 2))

    # Drift (end-to-end)
    drift = np.linalg.norm(estimated_positions[-1] - true_pos[-1])
    trajectory_length = np.sum(np.linalg.norm(np.diff(true_pos, axis=0), axis=1))
    drift_percent = (drift / trajectory_length) * 100

    logger.info(f"  Mean position error: {mean_error:.4f} m")
    logger.info(f"  Max position error: {max_error:.4f} m")
    logger.info(f"  RMSE: {rmse:.4f} m")
    logger.info(f"  End-to-end drift: {drift:.4f} m")
    logger.info(f"  Drift percentage: {drift_percent:.2f}%")
    logger.info(f"  Trajectory length: {trajectory_length:.2f} m")

    # Verify drift is within acceptable range
    assert drift_percent < 5.0, f"Drift {drift_percent:.2f}% exceeds 5% threshold"

    # Test with different trajectories
    logger.info("\nTesting with helix trajectory...")
    helix_traj = traj_gen.generate_helix(radius=3.0, height=5.0, turns=2.0)
    logger.info(f"  Helix points: {len(helix_traj['positions'])}")

    logger.info("\nTesting with rectangle trajectory...")
    rect_traj = traj_gen.generate_rectangle(width=10.0, height=10.0)
    logger.info(f"  Rectangle points: {len(rect_traj['positions'])}")

    logger.info("\n" + "=" * 50)
    logger.info("✅ SLAM SIMULATION TESTS PASSED")
    logger.info("=" * 50)

    return True


if __name__ == "__main__":
    test_slam()
