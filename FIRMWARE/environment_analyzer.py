#!/usr/bin/env python3
"""LeoDrone Phoenix - Environment Perception & Analysis
Multi-modal environment understanding: obstacles, terrain, hazards
"""
import numpy as np
import time
import logging
from dataclasses import dataclass
from typing import List, Dict

logger = logging.getLogger(__name__)

@dataclass
class Obstacle:
    position: np.ndarray   # (3,) NED
    size: np.ndarray       # (3,) meters
    velocity: np.ndarray   # (3,) m/s
    confidence: float
    obs_type: str          # "static", "dynamic", "terrain"

@dataclass
class TerrainInfo:
    elevation_map: np.ndarray
    roughness: float
    slope_deg: float
    land_cover: str
    timestamp: float

class EnvironmentAnalyzer:
    """Multi-modal environment perception"""
    def __init__(self, detection_range_m=50.0, sim_mode=True):
        self.detection_range = detection_range_m
        self.sim_mode = sim_mode
        self._obstacles: List[Obstacle] = []
        self._terrain: Optional[TerrainInfo] = None
    
    def detect_obstacles(self, point_cloud=None, radar_data=None) -> List[Obstacle]:
        """Detect obstacles from sensor fusion"""
        if self.sim_mode:
            n_obs = np.random.choice([0, 1, 2, 3], p=[0.5, 0.3, 0.15, 0.05])
            self._obstacles = []
            for i in range(n_obs):
                self._obstacles.append(Obstacle(
                    position=np.random.randn(3) * 10,
                    size=np.random.rand(3) * 3 + 0.5,
                    velocity=np.random.randn(3) * 0.5,
                    confidence=0.7 + np.random.rand() * 0.3,
                    obs_type="dynamic" if np.random.random() > 0.5 else "static"
                ))
        return self._obstacles
    
    def analyze_terrain(self, depth_map=None) -> TerrainInfo:
        """Analyze terrain from depth data"""
        t = time.time()
        if self.sim_mode:
            grid = np.random.rand(20, 20) * 5  # 20x20 grid, 0-5m elevation
            self._terrain = TerrainInfo(
                elevation_map=grid,
                roughness=float(np.std(grid)),
                slope_deg=float(np.random.rand() * 30),
                land_cover=np.random.choice(["grass", "concrete", "water", "forest"]),
                timestamp=t
            )
        return self._terrain
    
    def compute_risk_map(self) -> np.ndarray:
        """Compute risk map for path planning"""
        if self.sim_mode:
            return np.random.rand(20, 20)  # 0=safe, 1=dangerous
        risk = np.zeros((20, 20))
        for obs in self._obstacles:
            # Inflate obstacle by safety margin
            idx = np.clip((obs.position[:2] + 50).astype(int) // 5, 0, 19)
            risk[idx[0], idx[1]] = 1.0
        return risk
    
    def get_hazard_level(self) -> str:
        """Get overall hazard assessment"""
        n_dynamic = sum(1 for o in self._obstacles if o.obs_type == "dynamic")
        if n_dynamic > 2:
            return "HIGH"
        elif n_dynamic > 0 or len(self._obstacles) > 3:
            return "MEDIUM"
        return "LOW"
