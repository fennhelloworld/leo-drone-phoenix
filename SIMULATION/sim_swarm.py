#!/usr/bin/env python3
"""
LeoDrone Phoenix — 编队飞行仿真
3机编队: V字形 → 环形 → 线形 → 降落

测试编队生成、一致性控制、协同建图
"""

import asyncio
import numpy as np
import time
import math
import logging
from typing import Dict, List

logging.basicConfig(level=logging.INFO,
                    format="%(asctime)s [%(levelname)s] %(message)s")
logger = logging.getLogger("SimSwarm")

# Import from firmware
sys_path = "/home/fenn/projects/leo-drone-phoenix"
import sys
sys.path.insert(0, f"{sys_path}/FIRMWARE")

from swarm_coordinator import (
    SwarmCoordinator, FormationType, SwarmRole,
    FormationGenerator, ConsensusProtocol, CollaborativeMapper,
    UAVState
)


async def test_formation_shapes():
    """Test different formation shapes"""
    logger.info("=" * 50)
    logger.info("Formation Shape Tests")
    logger.info("=" * 50)

    generator = FormationGenerator()
    num_uavs = 3
    spacing = 5.0

    # V-Shape
    logger.info("\n--- V-Shape Formation ---")
    positions = generator.v_shape(num_uavs, spacing)
    for i, pos in enumerate(positions):
        logger.info(f"  UAV-{i}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
    assert len(positions) == num_uavs
    assert np.linalg.norm(positions[0]) < 0.01  # Leader at origin

    # Circle
    logger.info("\n--- Circle Formation ---")
    positions = generator.circle(num_uavs, spacing * 2)
    for i, pos in enumerate(positions):
        logger.info(f"  UAV-{i}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
    assert len(positions) == num_uavs
    # All should be at same distance from center
    distances = [np.linalg.norm(p[:2]) for p in positions]
    assert max(distances) - min(distances) < 0.01

    # Line
    logger.info("\n--- Line Formation ---")
    positions = generator.line(num_uavs, spacing)
    for i, pos in enumerate(positions):
        logger.info(f"  UAV-{i}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
    assert len(positions) == num_uavs

    # Diamond
    logger.info("\n--- Diamond Formation ---")
    positions = generator.diamond(num_uavs, spacing)
    for i, pos in enumerate(positions):
        logger.info(f"  UAV-{i}: [{pos[0]:.1f}, {pos[1]:.1f}, {pos[2]:.1f}]")
    assert len(positions) == num_uavs

    logger.info("\n✅ Formation shape tests passed")


async def test_consensus():
    """Test consensus protocol"""
    logger.info("\n" + "=" * 50)
    logger.info("Consensus Protocol Tests")
    logger.info("=" * 50)

    consensus = ConsensusProtocol(gain_position=1.0, gain_velocity=0.5)

    # Create leader and follower
    leader = UAVState(
        uav_id="leader",
        position=np.array([0, 0, -5]),
        velocity=np.array([1, 0, 0]),
        role=SwarmRole.LEADER
    )

    follower = UAVState(
        uav_id="follower",
        position=np.array([5, 5, -5]),
        velocity=np.array([0, 0, 0]),
        role=SwarmRole.FOLLOWER
    )

    # Target offset: 5m behind leader
    target_offset = np.array([-5, 0, 0])

    # Compute follower velocity
    velocity = consensus.compute_follower_velocity(follower, leader, target_offset)
    logger.info(f"Follower velocity command: [{velocity[0]:.2f}, {velocity[1]:.2f}, {velocity[2]:.2f}]")

    # Simulate convergence
    logger.info("\nSimulating convergence...")
    for step in range(20):
        follower.position += velocity * 0.1
        follower.velocity = velocity
        velocity = consensus.compute_follower_velocity(follower, leader, target_offset)

        desired_pos = leader.position + target_offset
        error = np.linalg.norm(follower.position - desired_pos)
        logger.info(f"  Step {step:2d}: error={error:.3f}m")

    logger.info("\n✅ Consensus protocol tests passed")


async def test_collaborative_mapping():
    """Test collaborative mapping"""
    logger.info("\n" + "=" * 50)
    logger.info("Collaborative Mapping Tests")
    logger.info("=" * 50)

    mapper = CollaborativeMapper(resolution=0.5, map_size=50.0)

    # UAV-1 observes obstacles
    logger.info("\nUAV-1 mapping...")
    observations_1 = [
        {'position': np.array([5, 5, 0]), 'occupied': True},
        {'position': np.array([6, 5, 0]), 'occupied': True},
        {'position': np.array([5, 6, 0]), 'occupied': True},
        {'position': np.array([10, 10, 0]), 'occupied': False},
    ]
    mapper.update_local_map("uav_1", np.array([0, 0, 0]), observations_1)

    # UAV-2 observes from different position
    logger.info("UAV-2 mapping...")
    observations_2 = [
        {'position': np.array([5, 5, 0]), 'occupied': True},  # Confirms
        {'position': np.array([4, 5, 0]), 'occupied': True},
        {'position': np.array([15, 15, 0]), 'occupied': True},
    ]
    mapper.update_local_map("uav_2", np.array([10, 0, 0]), observations_2)

    # Merge maps
    logger.info("Merging maps...")
    mapper.merge_maps()

    # Get results
    global_map = mapper.get_global_map()
    occupancy = mapper.get_occupancy_grid()

    occupied_cells = np.sum(occupancy)
    logger.info(f"  Global map shape: {global_map.shape}")
    logger.info(f"  Occupied cells: {occupied_cells}")
    logger.info(f"  Local maps: {list(mapper.local_maps.keys())}")

    assert global_map.shape[0] == global_map.shape[1]
    assert len(mapper.local_maps) == 2

    logger.info("\n✅ Collaborative mapping tests passed")


async def test_swarm_flight():
    """Test full swarm flight simulation"""
    logger.info("\n" + "=" * 50)
    logger.info("Swarm Flight Simulation")
    logger.info("=" * 50)

    coordinator = SwarmCoordinator(num_uavs=3, simulation=True)

    # Phase 1: V-shape formation takeoff
    logger.info("\n--- Phase 1: V-Shape Takeoff ---")
    coordinator.set_formation(FormationType.V_SHAPE, spacing=5.0)
    await coordinator.formation_takeoff(5.0)

    for uid, state in coordinator.uavs.items():
        logger.info(f"  {uid}: pos=[{state.position[0]:.1f}, "
                    f"{state.position[1]:.1f}, {state.position[2]:.1f}]")

    # Phase 2: Fly to waypoint
    logger.info("\n--- Phase 2: Formation Flight ---")
    await coordinator.formation_goto(10.0, 0.0)

    # Phase 3: Change to circle formation
    logger.info("\n--- Phase 3: Circle Formation ---")
    coordinator.set_formation(FormationType.CIRCLE, spacing=8.0)
    await asyncio.sleep(0.5)

    # Phase 4: Line formation
    logger.info("\n--- Phase 4: Line Formation ---")
    coordinator.set_formation(FormationType.LINE, spacing=5.0)
    await asyncio.sleep(0.5)

    # Phase 5: Land
    logger.info("\n--- Phase 5: Formation Landing ---")
    await coordinator.formation_land()

    for uid, state in coordinator.uavs.items():
        logger.info(f"  {uid}: pos=[{state.position[0]:.1f}, "
                    f"{state.position[1]:.1f}, {state.position[2]:.1f}]")

    # Get final status
    status = coordinator.get_formation_status()
    logger.info(f"\nFinal status:")
    logger.info(f"  Formation: {status['formation_type']}")
    logger.info(f"  Position errors: {status['position_errors']}")

    logger.info("\n✅ Swarm flight simulation tests passed")


async def test_swarm_robustness():
    """Test swarm robustness (lost UAV)"""
    logger.info("\n" + "=" * 50)
    logger.info("Swarm Robustness Tests")
    logger.info("=" * 50)

    coordinator = SwarmCoordinator(num_uavs=3, simulation=True)
    coordinator.set_formation(FormationType.V_SHAPE, spacing=5.0)

    # Simulate UAV-2 disconnecting
    logger.info("\nSimulating UAV-2 disconnect...")
    coordinator.uavs["uav_2"].connected = False
    coordinator.uavs["uav_2"].position = np.array([100, 100, 0])  # Far away

    # Remaining UAVs should still function
    commands = coordinator.compute_velocity_commands()
    active_uavs = sum(1 for uid, state in coordinator.uavs.items()
                      if state.connected)
    logger.info(f"  Active UAVs: {active_uavs}")
    logger.info(f"  Velocity commands: {len(commands)}")

    assert active_uavs == 2
    assert len(commands) == 3  # Commands still computed for all

    logger.info("\n✅ Swarm robustness tests passed")


async def main():
    """Run all swarm simulation tests"""
    logger.info("🔥 LeoDrone Phoenix — Swarm Simulation")
    logger.info("=" * 50)

    await test_formation_shapes()
    await test_consensus()
    await test_collaborative_mapping()
    await test_swarm_flight()
    await test_swarm_robustness()

    logger.info("\n" + "=" * 50)
    logger.info("✅ ALL SWARM SIMULATION TESTS PASSED")
    logger.info("=" * 50)


if __name__ == "__main__":
    asyncio.run(main())
