#!/usr/bin/env python3
"""
LeoDrone Phoenix — 仿真飞行测试
起飞 → 悬停 → 航点飞行 → 返航 → 降落

使用 MAVSDK Python 连接 PX4 SITL
"""

import asyncio
import logging
import sys
import time
import numpy as np

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SimTestFlight")

# Try MAVSDK import
try:
    from mavsdk import System
    from mavsdk.offboard import OffboardError, PositionNedYaw, VelocityNedYaw
    HAS_MAVSDK = True
except ImportError:
    HAS_MAVSDK = False
    logger.warning("MAVSDK not installed, running in pure simulation mode")


class SimulatedDrone:
    """纯Python仿真无人机 (无需MAVSDK)"""

    def __init__(self):
        self.position = np.zeros(3)  # NED
        self.velocity = np.zeros(3)
        self.yaw = 0.0
        self.armed = False
        self.in_air = False
        self.target_position = np.zeros(3)

    async def arm(self):
        logger.info("[SIM] Arming...")
        self.armed = True
        await asyncio.sleep(0.5)

    async def takeoff(self, altitude: float = 5.0):
        logger.info(f"[SIM] Taking off to {altitude}m...")
        self.armed = True
        self.in_air = True
        target = np.array([0, 0, -altitude])
        await self._move_to(target, speed=2.0)
        logger.info("[SIM] ✅ Takeoff complete")

    async def goto(self, north: float, east: float, down: float,
                   yaw: float = 0.0):
        logger.info(f"[SIM] Going to N={north}, E={east}, D={down}")
        target = np.array([north, east, down])
        self.yaw = yaw
        await self._move_to(target, speed=3.0)
        logger.info("[SIM] ✅ Waypoint reached")

    async def hover(self, duration: float = 3.0):
        logger.info(f"[SIM] Hovering for {duration}s...")
        await asyncio.sleep(duration)

    async def land(self):
        logger.info("[SIM] Landing...")
        target = np.array([self.position[0], self.position[1], 0])
        await self._move_to(target, speed=1.5)
        self.in_air = False
        self.armed = False
        logger.info("[SIM] ✅ Landed")

    async def return_to_launch(self):
        logger.info("[SIM] Return to launch...")
        # First go up
        target = np.array([0, 0, self.position[2]])
        await self._move_to(target, speed=3.0)
        # Then to home
        target = np.array([0, 0, -5])
        await self._move_to(target, speed=3.0)
        # Then land
        await self.land()

    async def _move_to(self, target: np.ndarray, speed: float = 2.0):
        """Smooth movement to target position"""
        distance = np.linalg.norm(target - self.position)
        duration = max(distance / speed, 0.1)

        start = self.position.copy()
        start_time = time.time()

        while time.time() - start_time < duration:
            progress = min((time.time() - start_time) / duration, 1.0)
            # Smooth easing
            progress = progress * progress * (3 - 2 * progress)
            self.position = start + (target - start) * progress
            self.velocity = (target - start) / duration
            await asyncio.sleep(0.02)

        self.position = target.copy()
        self.velocity = np.zeros(3)


async def test_flight_mavsdk():
    """Test flight using real MAVSDK connection"""
    drone = System()
    await drone.connect(system_address="udp://:14540")

    logger.info("Waiting for drone connection...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            logger.info("✅ Drone connected!")
            break

    # Arm
    logger.info("Arming...")
    await drone.action.arm()
    await asyncio.sleep(1)

    # Set initial offboard position
    logger.info("Setting offboard position...")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, 0, 0))

    # Start offboard
    logger.info("Starting offboard mode...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        logger.error(f"Offboard start failed: {e}")
        await drone.action.disarm()
        return False

    # Takeoff
    logger.info("Taking off to 5m...")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -5, 0))
    await asyncio.sleep(5)

    # Hover
    logger.info("Hovering for 3 seconds...")
    await asyncio.sleep(3)

    # Waypoint 1
    logger.info("Flying to waypoint 1 (N=10, E=0)...")
    await drone.offboard.set_position_ned(PositionNedYaw(10, 0, -5, 0))
    await asyncio.sleep(5)

    # Waypoint 2
    logger.info("Flying to waypoint 2 (N=10, E=10)...")
    await drone.offboard.set_position_ned(PositionNedYaw(10, 10, -5, 90))
    await asyncio.sleep(5)

    # Waypoint 3
    logger.info("Flying to waypoint 3 (N=0, E=10)...")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 10, -5, 180))
    await asyncio.sleep(5)

    # Return to home position
    logger.info("Returning to launch...")
    await drone.offboard.set_position_ned(PositionNedYaw(0, 0, -5, 270))
    await asyncio.sleep(5)

    # Stop offboard
    logger.info("Stopping offboard mode...")
    try:
        await drone.offboard.stop()
    except OffboardError as e:
        logger.warning(f"Offboard stop error: {e}")

    # Land
    logger.info("Landing...")
    await drone.action.land()
    await asyncio.sleep(5)

    logger.info("✅ Flight test complete!")
    return True


async def test_flight_simulated():
    """Test flight using pure Python simulation"""
    drone = SimulatedDrone()

    # Arm
    await drone.arm()

    # Takeoff
    await drone.takeoff(altitude=5.0)

    # Hover
    await drone.hover(3.0)

    # Waypoint flight pattern (square)
    waypoints = [
        (10, 0, -5, 0),
        (10, 10, -5, 90),
        (0, 10, -5, 180),
        (0, 0, -5, 270),
    ]

    for i, (n, e, d, y) in enumerate(waypoints):
        logger.info(f"Waypoint {i+1}/{len(waypoints)}")
        await drone.goto(n, e, d, y)

    # Return to launch
    await drone.return_to_launch()

    # Verify final state
    assert not drone.in_air, "Drone should be on ground"
    assert not drone.armed, "Drone should be disarmed"
    assert np.linalg.norm(drone.position) < 1.0, "Drone should be near origin"

    logger.info("✅ Simulated flight test complete!")
    return True


async def main():
    """Main entry point"""
    use_mavsdk = HAS_MAVSDK and "--sim" not in sys.argv

    if use_mavsdk:
        logger.info("Running MAVSDK flight test...")
        success = await test_flight_mavsdk()
    else:
        logger.info("Running simulated flight test (no MAVSDK needed)...")
        success = await test_flight_simulated()

    if success:
        logger.info("=" * 50)
        logger.info("✅ ALL FLIGHT TESTS PASSED")
        logger.info("=" * 50)
    else:
        logger.error("❌ FLIGHT TEST FAILED")
        sys.exit(1)


if __name__ == "__main__":
    asyncio.run(main())
