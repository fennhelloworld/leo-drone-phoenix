#!/usr/bin/env python3
"""
LeoDrone Phoenix — 360°全景拼接仿真
4路鱼眼相机仿真 + 去畸变 + 拼接 + 等距柱状投影
"""

import numpy as np
import time
import math
import logging

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("Sim360Stitch")


class FisheyeCameraModel:
    """鱼眼相机模型 (IMX219 + 鱼眼镜头)"""

    def __init__(self, resolution=(640, 480), fov_deg=160,
                 focal_length=1.5, distortion=None):
        self.resolution = resolution
        self.fov = np.radians(fov_deg)
        self.focal_length = focal_length
        self.cx = resolution[0] / 2
        self.cy = resolution[1] / 2

        # Kannala-Brandt distortion model coefficients
        self.distortion = distortion or np.array([0.2, 0.05, -0.01, 0.005])

    def project(self, point_3d: np.ndarray) -> np.ndarray:
        """Project 3D point to 2D pixel using fisheye model"""
        x, y, z = point_3d
        r = np.sqrt(x ** 2 + y ** 2)
        theta = np.arctan2(r, z)

        # Distortion
        theta_d = theta * (1 + self.distortion[0] * theta ** 2 +
                          self.distortion[1] * theta ** 4 +
                          self.distortion[2] * theta ** 6 +
                          self.distortion[3] * theta ** 8)

        # Project to image
        if r > 1e-6:
            px = self.focal_length * theta_d * x / r + self.cx
            py = self.focal_length * theta_d * y / r + self.cy
        else:
            px = self.cx
            py = self.cy

        return np.array([px, py])

    def undistort(self, pixel: np.ndarray) -> np.ndarray:
        """Undistort pixel coordinates"""
        # Inverse of the distortion model (simplified)
        x = (pixel[0] - self.cx) / self.focal_length
        y = (pixel[1] - self.cy) / self.focal_length

        r = np.sqrt(x ** 2 + y ** 2)
        theta = np.arctan(r)

        # Invert distortion
        theta_d = theta
        for _ in range(5):  # Newton iterations
            f = theta_d * (1 + self.distortion[0] * theta_d ** 2 +
                          self.distortion[1] * theta_d ** 4) - theta
            fp = 1 + 3 * self.distortion[0] * theta_d ** 2 + \
                 5 * self.distortion[1] * theta_d ** 4
            theta_d -= f / fp

        if r > 1e-6:
            scale = np.tan(theta_d) / r
            x_undist = x * scale
            y_undist = y * scale
        else:
            x_undist = x
            y_undist = y

        return np.array([x_undist * self.focal_length + self.cx,
                         y_undist * self.focal_length + self.cy])

    def generate_test_image(self, scene_id: int = 0,
                            yaw_offset: float = 0.0) -> np.ndarray:
        """Generate a test image with calibration pattern"""
        w, h = self.resolution
        image = np.zeros((h, w, 3), dtype=np.uint8)

        # Add grid pattern
        for i in range(0, w, 40):
            image[:, i, :] = 128
        for j in range(0, h, 40):
            image[j, :, :] = 128

        # Add camera ID marker
        colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]
        color = colors[scene_id % 4]
        image[20:60, 20:60] = color

        # Add direction indicator
        center_x, center_y = w // 2, h // 2
        angle = yaw_offset
        end_x = int(center_x + 50 * np.cos(angle))
        end_y = int(center_y + 50 * np.sin(angle))
        # Draw line
        for t in np.linspace(0, 1, 50):
            px = int(center_x + t * (end_x - center_x))
            py = int(center_y + t * (end_y - center_y))
            if 0 <= px < w and 0 <= py < h:
                image[py, px] = [255, 255, 255]

        return image


class PanoramicStitcher:
    """360°全景拼接器"""

    def __init__(self, num_cameras: int = 4, camera_fov: float = 160):
        self.num_cameras = num_cameras
        self.camera_fov = camera_fov

        # Camera models (front, right, back, left)
        self.cameras = []
        self.yaw_offsets = [0, np.pi/2, np.pi, 3*np.pi/2]

        for i in range(num_cameras):
            camera = FisheyeCameraModel(fov_deg=camera_fov)
            self.cameras.append(camera)

        # Output panoramic size
        self.pano_width = 1920
        self.pano_height = 960

    def undistort_frame(self, frame: np.ndarray,
                        camera_id: int) -> np.ndarray:
        """Undistort a single camera frame"""
        camera = self.cameras[camera_id]
        h, w = frame.shape[:2]
        undistorted = np.zeros_like(frame)

        # Remap each pixel
        map_x = np.zeros((h, w), dtype=np.float32)
        map_y = np.zeros((h, w), dtype=np.float32)

        for y in range(h):
            for x in range(w):
                undist_pixel = camera.undistort(np.array([x, y]))
                src_x = int(np.clip(undist_pixel[0], 0, w - 1))
                src_y = int(np.clip(undist_pixel[1], 0, h - 1))
                map_x[y, x] = src_x
                map_y[y, x] = src_y

        # Apply remap
        for y in range(h):
            for x in range(w):
                undistorted[y, x] = frame[int(map_y[y, x]), int(map_x[y, x])]

        return undistorted

    def stitch(self, frames: list) -> np.ndarray:
        """Stitch multiple camera frames into a panorama

        Args:
            frames: List of (H, W, 3) images from each camera

        Returns:
            Equirectangular panoramic image (pano_h, pano_w, 3)
        """
        pano = np.zeros((self.pano_height, self.pano_width, 3), dtype=np.uint8)

        for cam_id, frame in enumerate(frames):
            if frame is None:
                continue

            yaw = self.yaw_offsets[cam_id]
            h, w = frame.shape[:2]

            # Map each pixel in panorama to camera frame
            for py in range(self.pano_height):
                for px in range(self.pano_width // self.num_cameras):
                    # Equirectangular projection
                    lon = (px / (self.pano_width // self.num_cameras) - 0.5) * \
                          np.radians(self.camera_fov) + yaw
                    lat = (py / self.pano_height - 0.5) * np.pi

                    # Spherical to Cartesian
                    x = np.cos(lat) * np.sin(lon)
                    y = np.cos(lat) * np.cos(lon)
                    z = np.sin(lat)

                    # Project to camera
                    point_3d = np.array([x, y, z])
                    pixel = self.cameras[cam_id].project(point_3d)

                    # Sample from frame
                    src_x = int(np.clip(pixel[0], 0, w - 1))
                    src_y = int(np.clip(pixel[1], 0, h - 1))

                    # Map to panorama position
                    pano_px = int((lon / (2 * np.pi) + 0.5) * self.pano_width) % self.pano_width
                    pano[py, pano_px] = frame[src_y, src_x]

        return pano

    def fast_stitch(self, frames: list) -> np.ndarray:
        """Fast stitching using vectorized operations"""
        pano = np.zeros((self.pano_height, self.pano_width, 3), dtype=np.uint8)

        section_width = self.pano_width // self.num_cameras

        for cam_id, frame in enumerate(frames):
            if frame is None:
                continue

            h, w = frame.shape[:2]
            # Simple horizontal placement for fast simulation
            start_x = cam_id * section_width
            end_x = min(start_x + section_width, self.pano_width)

            # Resize frame to section
            target_w = end_x - start_x
            y_indices = np.linspace(0, h - 1, self.pano_height).astype(int)
            x_indices = np.linspace(0, w - 1, target_w).astype(int)

            pano[:, start_x:end_x] = frame[np.ix_(y_indices, x_indices)]

        return pano


def test_360_stitch():
    """Test 360° panoramic stitching"""
    logger.info("=" * 50)
    logger.info("360° Panoramic Stitching Simulation")
    logger.info("=" * 50)

    # Create stitcher
    stitcher = PanoramicStitcher(num_cameras=4, camera_fov=160)

    # Generate test frames from 4 cameras
    logger.info("Generating test frames...")
    frames = []
    for i in range(4):
        camera = stitcher.cameras[i]
        frame = camera.generate_test_image(scene_id=i,
                                           yaw_offset=stitcher.yaw_offsets[i])
        frames.append(frame)
        logger.info(f"  Camera {i}: frame shape={frame.shape}, "
                    f"yaw={np.degrees(stitcher.yaw_offsets[i]):.0f}°")

    # Test undistortion
    logger.info("\nTesting undistortion...")
    for i in range(4):
        undistorted = stitcher.undistort_frame(frames[i], i)
        logger.info(f"  Camera {i}: undistorted shape={undistorted.shape}")
        assert undistorted.shape == frames[i].shape

    # Test fast stitching
    logger.info("\nTesting fast stitching...")
    start_time = time.time()
    pano = stitcher.fast_stitch(frames)
    elapsed = time.time() - start_time
    logger.info(f"  Panorama shape: {pano.shape}")
    logger.info(f"  Stitching time: {elapsed:.3f}s")
    assert pano.shape == (960, 1920, 3), f"Expected (960,1920,3), got {pano.shape}"

    # Verify panorama content
    logger.info("\nVerifying panorama content...")
    # Each camera section should have its color marker
    section_width = 1920 // 4
    for i in range(4):
        section = pano[:, i * section_width:(i + 1) * section_width]
        assert section.mean() > 0, f"Section {i} is empty"

    # Test full stitching (slower but more accurate)
    logger.info("\nTesting full stitching...")
    start_time = time.time()
    pano_full = stitcher.stitch(frames)
    elapsed = time.time() - start_time
    logger.info(f"  Full panorama shape: {pano_full.shape}")
    logger.info(f"  Full stitching time: {elapsed:.3f}s")

    logger.info("\n" + "=" * 50)
    logger.info("✅ 360° STITCHING TESTS PASSED")
    logger.info("=" * 50)

    return True


if __name__ == "__main__":
    test_360_stitch()
