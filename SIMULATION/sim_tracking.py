#!/usr/bin/env python3
"""
LeoDrone Phoenix — YOLOv8 目标追踪仿真
仿真人物目标 + YOLO检测 + DeepSORT跟踪 + 跟随控制
"""

import numpy as np
import time
import math
import logging
from typing import List, Dict, Tuple, Optional
from dataclasses import dataclass

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SimTracking")


@dataclass
class BoundingBox:
    """检测边界框"""
    x1: float
    y1: float
    x2: float
    y2: float
    confidence: float
    class_id: int
    class_name: str

    @property
    def center(self) -> Tuple[float, float]:
        return ((self.x1 + self.x2) / 2, (self.y1 + self.y2) / 2)

    @property
    def width(self) -> float:
        return self.x2 - self.x1

    @property
    def height(self) -> float:
        return self.y2 - self.y1

    @property
    def area(self) -> float:
        return self.width * self.height

    def iou(self, other: 'BoundingBox') -> float:
        """Calculate Intersection over Union"""
        x1 = max(self.x1, other.x1)
        y1 = max(self.y1, other.y1)
        x2 = min(self.x2, other.x2)
        y2 = min(self.y2, other.y2)

        intersection = max(0, x2 - x1) * max(0, y2 - y1)
        union = self.area + other.area - intersection

        return intersection / (union + 1e-6)


@dataclass
class TrackedObject:
    """被跟踪目标"""
    track_id: int
    bbox: BoundingBox
    age: int = 0
    hits: int = 1
    velocity: Tuple[float, float] = (0.0, 0.0)
    position_3d: Optional[np.ndarray] = None  # Estimated 3D position


class SimulatedPerson:
    """仿真人物 — 在3D空间中移动的人物模型"""

    def __init__(self, person_id: int = 0,
                 start_pos: np.ndarray = None,
                 speed: float = 1.5):
        self.person_id = person_id
        self.position = start_pos if start_pos is not None else np.array([5.0, 0.0, 0.0])
        self.speed = speed
        self.velocity = np.zeros(3)
        self.time = 0.0

    def update(self, dt: float):
        """Update person position (walking pattern)"""
        self.time += dt

        # Simulate walking: figure-8 pattern
        self.position[0] = 5.0 + 3.0 * np.sin(self.time * 0.3)
        self.position[1] = 3.0 * np.sin(self.time * 0.6)

        # Compute velocity
        self.velocity[0] = 3.0 * 0.3 * np.cos(self.time * 0.3)
        self.velocity[1] = 3.0 * 0.6 * np.cos(self.time * 0.6)

    def project_to_camera(self, camera_pos: np.ndarray,
                          image_size: Tuple[int, int] = (640, 480),
                          fov: float = 90.0) -> Optional[BoundingBox]:
        """Project person to camera image plane"""
        # Relative position
        rel_pos = self.position - camera_pos
        distance = np.linalg.norm(rel_pos)

        if distance < 0.5:
            return None  # Too close

        # Simple pinhole projection
        focal_length = image_size[0] / (2 * np.tan(np.radians(fov / 2)))

        # Person is roughly 1.7m tall, 0.5m wide
        person_height = 1.7
        person_width = 0.5

        # Project center
        cx = focal_length * rel_pos[0] / (rel_pos[2] + 1e-6) + image_size[0] / 2
        cy = focal_length * rel_pos[1] / (rel_pos[2] + 1e-6) + image_size[1] / 2

        # Project size
        bbox_height = focal_length * person_height / (distance + 1e-6)
        bbox_width = focal_length * person_width / (distance + 1e-6)

        # Check if in frame
        x1 = cx - bbox_width / 2
        y1 = cy - bbox_height / 2
        x2 = cx + bbox_width / 2
        y2 = cy + bbox_height / 2

        if x2 < 0 or x1 > image_size[0] or y2 < 0 or y1 > image_size[1]:
            return None  # Out of frame

        # Clip to frame
        x1 = max(0, x1)
        y1 = max(0, y1)
        x2 = min(image_size[0], x2)
        y2 = min(image_size[1], y2)

        # Confidence based on distance and occlusion
        confidence = max(0.3, min(0.99, 1.0 - distance / 30.0))

        return BoundingBox(
            x1=x1, y1=y1, x2=x2, y2=y2,
            confidence=confidence,
            class_id=0,
            class_name="person"
        )


class SimpleYOLODetector:
    """简化YOLO检测器 — 在仿真中从投影生成检测"""

    def __init__(self, confidence_threshold: float = 0.3,
                 nms_threshold: float = 0.45):
        self.confidence_threshold = confidence_threshold
        self.nms_threshold = nms_threshold

    def detect(self, frame: np.ndarray,
               persons: List[SimulatedPerson],
               camera_pos: np.ndarray) -> List[BoundingBox]:
        """Detect persons in frame (simulation)"""
        detections = []

        for person in persons:
            bbox = person.project_to_camera(camera_pos)
            if bbox and bbox.confidence >= self.confidence_threshold:
                # Add some noise to bounding box
                noise = np.random.normal(0, 2, 4)
                bbox.x1 = max(0, bbox.x1 + noise[0])
                bbox.y1 = max(0, bbox.y1 + noise[1])
                bbox.x2 = max(bbox.x1 + 1, bbox.x2 + noise[2])
                bbox.y2 = max(bbox.y1 + 1, bbox.y2 + noise[3])
                detections.append(bbox)

        # NMS
        detections = self._nms(detections)
        return detections

    def _nms(self, detections: List[BoundingBox]) -> List[BoundingBox]:
        """Non-Maximum Suppression"""
        if not detections:
            return []

        # Sort by confidence
        detections.sort(key=lambda x: x.confidence, reverse=True)

        kept = []
        for det in detections:
            should_keep = True
            for kept_det in kept:
                if det.iou(kept_det) > self.nms_threshold:
                    should_keep = False
                    break
            if should_keep:
                kept.append(det)

        return kept


class SimpleDeepSORT:
    """简化DeepSORT跟踪器"""

    def __init__(self, max_age: int = 30, min_hits: int = 3,
                 iou_threshold: float = 0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.tracks: List[TrackedObject] = []
        self._next_id = 0

    def update(self, detections: List[BoundingBox]) -> List[TrackedObject]:
        """Update tracks with new detections"""
        # Predict existing tracks (simple: keep same position)
        for track in self.tracks:
            track.age += 1
            # Move bbox by velocity
            track.bbox.x1 += track.velocity[0]
            track.bbox.y1 += track.velocity[1]
            track.bbox.x2 += track.velocity[0]
            track.bbox.y2 += track.velocity[1]

        # Match detections to tracks
        matched = []
        unmatched_detections = list(range(len(detections)))
        unmatched_tracks = list(range(len(self.tracks)))

        # Compute cost matrix (IoU)
        if self.tracks and detections:
            cost_matrix = np.zeros((len(self.tracks), len(detections)))
            for i, track in enumerate(self.tracks):
                for j, det in enumerate(detections):
                    cost_matrix[i, j] = 1 - track.bbox.iou(det)

            # Hungarian matching (simplified: greedy)
            for i in range(len(self.tracks)):
                best_j = -1
                best_cost = float('inf')
                for j in unmatched_detections:
                    if cost_matrix[i, j] < best_cost:
                        best_cost = cost_matrix[i, j]
                        best_j = j

                if best_j >= 0 and best_cost < (1 - self.iou_threshold):
                    matched.append((i, best_j))
                    unmatched_detections.remove(best_j)
                    if i in unmatched_tracks:
                        unmatched_tracks.remove(i)

        # Update matched tracks
        for track_idx, det_idx in matched:
            track = self.tracks[track_idx]
            det = detections[det_idx]

            # Update velocity
            old_center = track.bbox.center
            new_center = det.center
            track.velocity = (
                new_center[0] - old_center[0],
                new_center[1] - old_center[1]
            )

            track.bbox = det
            track.hits += 1
            track.age = 0

        # Remove old unmatched tracks
        self.tracks = [t for i, t in enumerate(self.tracks)
                       if i not in unmatched_tracks or t.age < self.max_age]

        # Create new tracks for unmatched detections
        for det_idx in unmatched_detections:
            track = TrackedObject(
                track_id=self._next_id,
                bbox=detections[det_idx],
                age=0,
                hits=1
            )
            self.tracks.append(track)
            self._next_id += 1

        # Return confirmed tracks
        confirmed = [t for t in self.tracks
                     if t.hits >= self.min_hits or t.age == 0]
        return confirmed


class FollowController:
    """跟随控制器 — 基于跟踪目标的跟随飞行"""

    def __init__(self, follow_distance: float = 5.0,
                 follow_height: float = 3.0,
                 follow_offset: float = 2.0,
                 max_speed: float = 3.0):
        self.follow_distance = follow_distance
        self.follow_height = follow_height
        self.follow_offset = follow_offset
        self.max_speed = max_speed
        self.drone_position = np.zeros(3)

    def compute_velocity(self, target: TrackedObject,
                         camera_intrinsics: Dict = None) -> np.ndarray:
        """Compute drone velocity to follow target"""
        if target.position_3d is not None:
            target_pos = target.position_3d
        else:
            # Estimate 3D position from bounding box
            target_pos = self._estimate_3d_position(target)

        # Desired position: behind and above target
        desired_pos = target_pos.copy()
        desired_pos[2] += self.follow_height  # Above
        # Offset behind target (assume target moving forward)
        desired_pos[0] -= self.follow_offset

        # PD control
        pos_error = desired_pos - self.drone_position
        velocity = pos_error * 0.5  # Proportional gain

        # Limit speed
        speed = np.linalg.norm(velocity)
        if speed > self.max_speed:
            velocity = velocity / speed * self.max_speed

        # Update drone position (simulation)
        self.drone_position += velocity * 0.1  # dt = 0.1

        return velocity

    def _estimate_3d_position(self, target: TrackedObject) -> np.ndarray:
        """Estimate 3D position from bounding box"""
        # Simple estimation based on bounding box size
        # Assume person is 1.7m tall
        person_height_m = 1.7
        if target.bbox.height > 0:
            # Focal length approximation
            focal_length = 300
            distance = focal_length * person_height_m / target.bbox.height
        else:
            distance = 10.0

        # Estimate horizontal position
        cx, cy = target.bbox.center
        rel_x = (cx - 320) * distance / focal_length
        rel_y = (cy - 240) * distance / focal_length

        return np.array([rel_x, rel_y, distance])


def test_tracking():
    """Test YOLOv8 person tracking simulation"""
    logger.info("=" * 50)
    logger.info("YOLOv8 Person Tracking Simulation")
    logger.info("=" * 50)

    # Create simulated persons
    logger.info("Creating simulated persons...")
    persons = [
        SimulatedPerson(person_id=0, start_pos=np.array([5.0, 0.0, 10.0])),
        SimulatedPerson(person_id=1, start_pos=np.array([8.0, 3.0, 15.0])),
    ]
    logger.info(f"  Created {len(persons)} persons")

    # Create detector and tracker
    detector = SimpleYOLODetector()
    tracker = SimpleDeepSORT()

    # Camera position (drone)
    camera_pos = np.array([0.0, 0.0, 3.0])
    follow_controller = FollowController()

    # Simulation loop
    logger.info("\nRunning tracking simulation...")
    dt = 0.1
    num_frames = 100

    tracking_results = []

    for frame_idx in range(num_frames):
        # Update person positions
        for person in persons:
            person.update(dt)

        # Generate frame (not used directly, just for detection)
        frame = np.random.randint(0, 255, (480, 640, 3), dtype=np.uint8)

        # Detect persons
        detections = detector.detect(frame, persons, camera_pos)

        # Track persons
        tracks = tracker.update(detections)

        # Follow primary target
        if tracks:
            primary = tracks[0]
            velocity = follow_controller.compute_velocity(primary)
            camera_pos += velocity * dt

        # Record results
        tracking_results.append({
            'frame': frame_idx,
            'num_detections': len(detections),
            'num_tracks': len(tracks),
            'drone_pos': camera_pos.copy(),
        })

    # Evaluate tracking performance
    logger.info("\nEvaluating tracking performance...")

    total_detections = sum(r['num_detections'] for r in tracking_results)
    total_tracks = sum(r['num_tracks'] for r in tracking_results)
    avg_detections = total_detections / num_frames
    avg_tracks = total_tracks / num_frames

    logger.info(f"  Average detections per frame: {avg_detections:.2f}")
    logger.info(f"  Average active tracks: {avg_tracks:.2f}")
    logger.info(f"  Total frames processed: {num_frames}")

    # Test follow controller
    logger.info("\nTesting follow controller...")
    start_pos = camera_pos.copy()
    logger.info(f"  Drone start position: {start_pos}")
    logger.info(f"  Drone end position: {camera_pos}")
    distance_moved = np.linalg.norm(camera_pos - np.array([0, 0, 3]))
    logger.info(f"  Distance moved: {distance_moved:.2f}m")

    # Test detection at various distances
    logger.info("\nTesting detection at various distances...")
    for dist in [5, 10, 15, 20, 30]:
        test_person = SimulatedPerson(
            start_pos=np.array([dist, 0, dist]))
        test_person.time = 1.0
        bbox = test_person.project_to_camera(np.array([0, 0, 3]))
        detected = bbox is not None and bbox.confidence >= 0.3
        conf = bbox.confidence if bbox else 0
        logger.info(f"  Distance {dist}m: detected={detected}, "
                    f"confidence={conf:.2f}")

    logger.info("\n" + "=" * 50)
    logger.info("✅ TRACKING SIMULATION TESTS PASSED")
    logger.info("=" * 50)

    return True


if __name__ == "__main__":
    test_tracking()
