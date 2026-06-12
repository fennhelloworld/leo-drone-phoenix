#!/usr/bin/env python3
"""LeoDrone Phoenix - Early Warning System
Multi-source threat detection and alert generation
"""
import numpy as np
import time
import logging
from dataclasses import dataclass
from typing import List, Dict
from enum import Enum, auto

logger = logging.getLogger(__name__)

class WarningLevel(Enum):
    INFO = 0
    CAUTION = 1
    WARNING = 2
    CRITICAL = 3
    EMERGENCY = 4

@dataclass
class Warning:
    level: WarningLevel
    source: str
    message: str
    action: str
    timestamp: float
    confidence: float

class EarlyWarningSystem:
    """Multi-source early warning for drone safety"""
    def __init__(self):
        self._active_warnings: List[Warning] = []
        self._warning_history: List[Warning] = []
        self._thresholds = {
            "battery_low_pct": 20.0,
            "battery_critical_pct": 10.0,
            "altitude_max_m": 120.0,
            "distance_max_m": 500.0,
            "wind_max_ms": 12.0,
            "visibility_min_km": 1.0,
            "collision_distance_m": 3.0,
            "signal_quality_min": 0.3,
        }
    
    def check_battery(self, pct: float) -> List[Warning]:
        warnings = []
        t = time.time()
        if pct < self._thresholds["battery_critical_pct"]:
            warnings.append(Warning(WarningLevel.EMERGENCY, "battery",
                f"Battery critical: {pct:.0f}%", "LAND IMMEDIATELY", t, 1.0))
        elif pct < self._thresholds["battery_low_pct"]:
            warnings.append(Warning(WarningLevel.WARNING, "battery",
                f"Battery low: {pct:.0f}%", "Return to launch", t, 0.9))
        return warnings
    
    def check_geofence(self, distance_m: float) -> List[Warning]:
        warnings = []
        t = time.time()
        if distance_m > self._thresholds["distance_max_m"]:
            warnings.append(Warning(WarningLevel.CRITICAL, "geofence",
                f"Beyond geofence: {distance_m:.0f}m", "Return immediately", t, 1.0))
        elif distance_m > self._thresholds["distance_max_m"] * 0.9:
            warnings.append(Warning(WarningLevel.CAUTION, "geofence",
                f"Approaching geofence: {distance_m:.0f}m", "Turn back", t, 0.8))
        return warnings
    
    def check_weather(self, wind_ms: float, vis_km: float) -> List[Warning]:
        warnings = []
        t = time.time()
        if wind_ms > self._thresholds["wind_max_ms"]:
            warnings.append(Warning(WarningLevel.WARNING, "weather",
                f"High wind: {wind_ms:.1f}m/s", "Land or reduce altitude", t, 0.85))
        if vis_km < self._thresholds["visibility_min_km"]:
            warnings.append(Warning(WarningLevel.CRITICAL, "weather",
                f"Low visibility: {vis_km:.1f}km", "Land immediately", t, 0.9))
        return warnings
    
    def check_collision(self, nearest_obstacle_m: float) -> List[Warning]:
        warnings = []
        t = time.time()
        if nearest_obstacle_m < self._thresholds["collision_distance_m"]:
            warnings.append(Warning(WarningLevel.EMERGENCY, "collision",
                f"Collision imminent: {nearest_obstacle_m:.1f}m", "EVASIVE ACTION", t, 0.95))
        elif nearest_obstacle_m < self._thresholds["collision_distance_m"] * 2:
            warnings.append(Warning(WarningLevel.WARNING, "collision",
                f"Obstacle nearby: {nearest_obstacle_m:.1f}m", "Slow down", t, 0.8))
        return warnings
    
    def check_all(self, battery_pct: float, distance_m: float,
                  wind_ms: float, vis_km: float, 
                  nearest_obstacle_m: float) -> List[Warning]:
        """Run all safety checks and return aggregated warnings"""
        all_warnings = []
        all_warnings.extend(self.check_battery(battery_pct))
        all_warnings.extend(self.check_geofence(distance_m))
        all_warnings.extend(self.check_weather(wind_ms, vis_km))
        all_warnings.extend(self.check_collision(nearest_obstacle_m))
        
        self._active_warnings = all_warnings
        self._warning_history.extend(all_warnings)
        if len(self._warning_history) > 1000:
            self._warning_history = self._warning_history[-1000:]
        
        return all_warnings
    
    def get_max_level(self) -> WarningLevel:
        if not self._active_warnings:
            return WarningLevel.INFO
        return max(w.level for w in self._active_warnings)
    
    def get_active_warnings(self) -> List[Warning]:
        return self._active_warnings.copy()
