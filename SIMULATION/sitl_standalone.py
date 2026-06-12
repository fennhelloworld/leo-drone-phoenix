#!/usr/bin/env python3
"""LeoDrone Phoenix - Standalone SITL Simulation (No Docker needed)
Pure Python simulation of PX4 SITL + MAVLink offboard control
"""
import sys
import time
import numpy as np

class SimulatedDrone:
    """Simplified drone dynamics for SITL simulation"""
    def __init__(self):
        self.pos = np.array([0.0, 0.0, 0.0])  # NED frame
        self.vel = np.array([0.0, 0.0, 0.0])
        self.att = np.array([0.0, 0.0, 0.0])  # roll, pitch, yaw (rad)
        self.armed = False
        self.mode = "STABILIZED"
        self.battery = 100.0  # %
        
    def arm(self):
        self.armed = True
        self.mode = "OFFBOARD"
        print("  [MAVLink] ARMED → OFFBOARD mode")
        
    def takeoff(self, altitude=2.0):
        if not self.armed:
            self.arm()
        target = np.array([self.pos[0], self.pos[1], -altitude])  # NED: down is positive
        print(f"  [MAVLink] TAKEOFF to {altitude}m")
        for step in range(50):
            self.pos = self.pos * 0.9 + target * 0.1
            self.vel = (target - self.pos) * 0.5
            self.battery -= 0.01
            time.sleep(0.02)
        self.pos = target.copy()
        print(f"  [MAVLink] At altitude: {altitude}m ✓")
        
    def goto(self, x, y, z):
        target = np.array([x, y, -z])
        dist = np.linalg.norm(target - self.pos)
        print(f"  [MAVLink] GOTO ({x:.1f}, {y:.1f}, {z:.1f}) dist={dist:.1f}m")
        for step in range(int(dist * 20)):
            self.pos = self.pos * 0.95 + target * 0.05
            self.vel = (target - self.pos) * 0.3
            self.battery -= 0.005
            time.sleep(0.01)
        self.pos = target.copy()
        print(f"  [MAVLink] Reached waypoint ✓")
        
    def land(self):
        print("  [MAVLink] LANDING")
        target = np.array([self.pos[0], self.pos[1], 0.0])
        for step in range(30):
            self.pos = self.pos * 0.9 + target * 0.1
            self.battery -= 0.005
            time.sleep(0.02)
        self.pos = target.copy()
        self.armed = False
        print("  [MAVLink] LANDED ✓ DISARMED")
        
    def rtl(self):
        print("  [MAVLink] RETURN TO LAUNCH")
        self.goto(0, 0, 2.0)
        self.land()


def run_sitl():
    print()
    print("=" * 50)
    print("  LeoDrone Phoenix — SITL Simulation")
    print("=" * 50)
    print()
    
    drone = SimulatedDrone()
    
    # Phase 1: Preflight check
    print("Phase 1: Pre-flight Check")
    print(f"  Battery: {drone.battery:.0f}%")
    print(f"  GPS: 3D Fix (12 sats)")
    print(f"  IMU: Calibrated ✓")
    print(f"  BME280: T=25.1°C H=50.2% ✓")
    print()
    
    # Phase 2: Takeoff
    print("Phase 2: Takeoff")
    drone.takeoff(2.0)
    print()
    
    # Phase 3: Waypoint navigation
    print("Phase 3: Waypoint Navigation")
    drone.goto(5.0, 0.0, 2.0)   # East 5m
    drone.goto(5.0, 5.0, 2.0)   # North 5m
    drone.goto(0.0, 5.0, 2.0)   # West 5m
    drone.goto(0.0, 0.0, 2.0)   # South 5m (back to start)
    print()
    
    # Phase 4: Sensor scan
    print("Phase 4: Sensor Scan at Waypoints")
    for i, (x, y) in enumerate([(3, 0), (3, 3), (0, 3)]):
        drone.goto(x, y, 2.0)
        # Simulate BME280 readings
        temp = 25.0 + np.random.normal(0, 0.5)
        hum = 50.0 + np.random.normal(0, 2)
        print(f"  BME280 @ ({x},{y}): T={temp:.1f}°C H={hum:.1f}%")
    print()
    
    # Phase 5: RTL
    print("Phase 5: Return To Launch")
    drone.rtl()
    print()
    
    # Summary
    print("=" * 50)
    print(f"  SITL Simulation COMPLETE")
    print(f"  Battery remaining: {drone.battery:.1f}%")
    print(f"  Final pos: ({drone.pos[0]:.1f}, {drone.pos[1]:.1f}, {-drone.pos[2]:.1f})")
    print("=" * 50)


if __name__ == "__main__":
    run_sitl()
