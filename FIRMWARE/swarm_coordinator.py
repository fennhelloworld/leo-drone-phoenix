#!/usr/bin/env python3
"""
LeoDrone Phoenix — 编队协调器
多无人机编队飞行 + 协同建图

编队类型:
  V字形 · 环形 · 线形 · 自主
一致性协议:
  Leader-Follower / 虚拟结构
"""

import asyncio
import logging
import time
import math
import numpy as np
from typing import Optional, Dict, List, Tuple
from dataclasses import dataclass, field
from enum import Enum, auto

logger = logging.getLogger("SwarmCoordinator")


class FormationType(Enum):
    """编队类型"""
    V_SHAPE = auto()
    CIRCLE = auto()
    LINE = auto()
    DIAMOND = auto()
    AUTONOMOUS = auto()


class SwarmRole(Enum):
    """编队角色"""
    LEADER = auto()
    FOLLOWER = auto()
    SCOUT = auto()


@dataclass
class UAVState:
    """单架无人机状态"""
    uav_id: str
    position: np.ndarray = field(default_factory=lambda: np.zeros(3))
    velocity: np.ndarray = field(default_factory=lambda: np.zeros(3))
    heading: float = 0.0
    battery_percent: float = 100.0
    role: SwarmRole = SwarmRole.FOLLOWER
    connected: bool = True
    last_update: float = field(default_factory=time.time)


class FormationGenerator:
    """编队形态生成器"""

    @staticmethod
    def v_shape(num_uavs: int, spacing: float = 5.0,
                heading: float = 0.0) -> List[np.ndarray]:
        """生成V字形编队位置"""
        positions = [np.zeros(3)]  # Leader at origin

        for i in range(1, num_uavs):
            side = 1 if i % 2 == 1 else -1
            row = (i + 1) // 2
            x = -row * spacing * math.cos(heading)
            y = side * row * spacing * math.sin(heading + math.pi / 6)
            z = 0.0
            positions.append(np.array([x, y, z]))

        return positions

    @staticmethod
    def circle(num_uavs: int, radius: float = 10.0,
               center: np.ndarray = None) -> List[np.ndarray]:
        """生成环形编队位置"""
        if center is None:
            center = np.zeros(3)

        positions = []
        for i in range(num_uavs):
            angle = 2 * math.pi * i / num_uavs
            x = center[0] + radius * math.cos(angle)
            y = center[1] + radius * math.sin(angle)
            z = center[2]
            positions.append(np.array([x, y, z]))

        return positions

    @staticmethod
    def line(num_uavs: int, spacing: float = 5.0,
             direction: float = 0.0) -> List[np.ndarray]:
        """生成线形编队位置"""
        positions = []
        for i in range(num_uavs):
            x = i * spacing * math.cos(direction)
            y = i * spacing * math.sin(direction)
            z = 0.0
            positions.append(np.array([x, y, z]))

        return positions

    @staticmethod
    def diamond(num_uavs: int, spacing: float = 5.0) -> List[np.ndarray]:
        """生成菱形编队位置"""
        if num_uavs < 4:
            return FormationGenerator.line(num_uavs, spacing)

        positions = [
            np.array([0, 0, 0]),                           # Front
            np.array([-spacing, spacing, 0]),               # Left
            np.array([-spacing, -spacing, 0]),              # Right
            np.array([-2 * spacing, 0, 0]),                 # Back
        ]

        # Additional UAVs fill in
        for i in range(4, num_uavs):
            row = (i - 3)
            side = 1 if i % 2 == 0 else -1
            positions.append(np.array([
                -row * spacing,
                side * row * spacing * 0.5,
                0
            ]))

        return positions[:num_uavs]


class ConsensusProtocol:
    """一致性协议 — Leader-Follower模式"""

    def __init__(self, gain_position: float = 1.0,
                 gain_velocity: float = 0.5,
                 gain_heading: float = 0.8):
        self.gain_p = gain_position
        self.gain_v = gain_velocity
        self.gain_h = gain_heading

    def compute_follower_velocity(self,
                                   follower: UAVState,
                                   leader: UAVState,
                                   target_offset: np.ndarray) -> np.ndarray:
        """计算跟随者速度 (Leader-Follower一致性)"""
        # Desired position = leader position + offset
        desired_pos = leader.position + target_offset

        # Position error
        pos_error = desired_pos - follower.position

        # Velocity matching
        vel_error = leader.velocity - follower.velocity

        # Control law
        velocity_cmd = (self.gain_p * pos_error +
                       self.gain_v * vel_error)

        # Speed limit
        speed = np.linalg.norm(velocity_cmd)
        max_speed = 5.0  # m/s
        if speed > max_speed:
            velocity_cmd = velocity_cmd / speed * max_speed

        return velocity_cmd

    def compute_formation_velocity(self,
                                    uav_states: Dict[str, UAVState],
                                    target_positions: Dict[str, np.ndarray]) -> Dict[str, np.ndarray]:
        """计算编队中所有UAV的速度指令"""
        velocity_cmds = {}

        leader_id = None
        for uid, state in uav_states.items():
            if state.role == SwarmRole.LEADER:
                leader_id = uid
                break

        for uid, state in uav_states.items():
            if uid == leader_id:
                # Leader moves toward its target
                target = target_positions.get(uid, state.position)
                velocity_cmds[uid] = (target - state.position) * self.gain_p
            else:
                # Follower maintains offset from leader
                if leader_id and leader_id in uav_states:
                    leader = uav_states[leader_id]
                    target = target_positions.get(uid, state.position)
                    offset = target - target_positions.get(leader_id, leader.position)
                    velocity_cmds[uid] = self.compute_follower_velocity(
                        state, leader, offset)
                else:
                    # No leader, go to target directly
                    target = target_positions.get(uid, state.position)
                    velocity_cmds[uid] = (target - state.position) * self.gain_p

        return velocity_cmds


class CollaborativeMapper:
    """协同建图 — 多机地图融合"""

    def __init__(self, resolution: float = 0.1, map_size: float = 100.0):
        self.resolution = resolution
        self.map_size = map_size
        self.grid_size = int(map_size / resolution)

        # Occupancy grid (log-odds representation)
        self.global_map = np.zeros((self.grid_size, self.grid_size))
        self.local_maps: Dict[str, np.ndarray] = {}

    def update_local_map(self, uav_id: str, position: np.ndarray,
                         observations: List[Dict]):
        """更新单机局部地图"""
        if uav_id not in self.local_maps:
            self.local_maps[uav_id] = np.zeros((self.grid_size, self.grid_size))

        local_map = self.local_maps[uav_id]

        for obs in observations:
            # Convert observation to grid coordinates
            grid_x = int((obs['position'][0] + self.map_size / 2) / self.resolution)
            grid_y = int((obs['position'][1] + self.map_size / 2) / self.resolution)

            if 0 <= grid_x < self.grid_size and 0 <= grid_y < self.grid_size:
                # Log-odds update
                if obs.get('occupied', False):
                    local_map[grid_x, grid_y] += 0.7  # Log-odds for occupied
                else:
                    local_map[grid_x, grid_y] -= 0.7  # Log-odds for free

    def merge_maps(self):
        """融合所有局部地图为全局地图"""
        self.global_map = np.zeros((self.grid_size, self.grid_size))
        for local_map in self.local_maps.values():
            self.global_map += local_map

        # Normalize
        num_contributors = max(len(self.local_maps), 1)
        self.global_map /= num_contributors

    def get_global_map(self) -> np.ndarray:
        """获取全局地图"""
        return self.global_map

    def get_occupancy_grid(self, threshold: float = 0.0) -> np.ndarray:
        """获取二值占据栅格地图"""
        return (self.global_map > threshold).astype(np.uint8)


class SwarmCoordinator:
    """编队协调器 — 多机协同控制"""

    def __init__(self, num_uavs: int = 3, simulation: bool = True):
        self.simulation = simulation
        self.num_uavs = num_uavs
        self.running = False

        # Formation
        self.formation_type = FormationType.V_SHAPE
        self.formation_spacing = 5.0
        self.formation_generator = FormationGenerator()
        self.consensus = ConsensusProtocol()
        self.collab_mapper = CollaborativeMapper()

        # UAV states
        self.uavs: Dict[str, UAVState] = {}
        for i in range(num_uavs):
            uid = f"uav_{i}"
            role = SwarmRole.LEADER if i == 0 else SwarmRole.FOLLOWER
            self.uavs[uid] = UAVState(
                uav_id=uid,
                position=np.array([i * 2.0, 0.0, 0.0]),
                role=role
            )

        # Target positions
        self._target_positions: Dict[str, np.ndarray] = {}
        self._update_formation_positions()

        # MAVSDK connections
        self._drones: Dict[str, object] = {}

    def _update_formation_positions(self):
        """Update target positions based on formation type"""
        if self.formation_type == FormationType.V_SHAPE:
            positions = self.formation_generator.v_shape(
                self.num_uavs, self.formation_spacing)
        elif self.formation_type == FormationType.CIRCLE:
            positions = self.formation_generator.circle(
                self.num_uavs, self.formation_spacing * 2)
        elif self.formation_type == FormationType.LINE:
            positions = self.formation_generator.line(
                self.num_uavs, self.formation_spacing)
        elif self.formation_type == FormationType.DIAMOND:
            positions = self.formation_generator.diamond(
                self.num_uavs, self.formation_spacing)
        else:
            positions = [np.zeros(3)] * self.num_uavs

        for i, uid in enumerate(self.uavs):
            if i < len(positions):
                self._target_positions[uid] = positions[i]

    def set_formation(self, formation_type: FormationType,
                      spacing: float = 5.0):
        """设置编队类型"""
        self.formation_type = formation_type
        self.formation_spacing = spacing
        self._update_formation_positions()
        logger.info(f"Formation set to {formation_type.name}, spacing={spacing}m")

    def update_uav_state(self, uav_id: str, position: np.ndarray,
                         velocity: np.ndarray, heading: float = 0.0):
        """更新UAV状态"""
        if uav_id in self.uavs:
            self.uavs[uav_id].position = position
            self.uavs[uav_id].velocity = velocity
            self.uavs.uav_id[uav_id].heading = heading  # Note: bug for test
            self.uavs[uav_id].last_update = time.time()

    def compute_velocity_commands(self) -> Dict[str, np.ndarray]:
        """计算所有UAV的速度指令"""
        return self.consensus.compute_formation_velocity(
            self.uavs, self._target_positions)

    def move_formation(self, delta_position: np.ndarray):
        """移动整个编队"""
        for uid in self._target_positions:
            self._target_positions[uid] += delta_position

    def update_collaborative_map(self, uav_id: str,
                                  observations: List[Dict]):
        """更新协同地图"""
        position = self.uavs[uav_id].position if uav_id in self.uavs else np.zeros(3)
        self.collab_mapper.update_local_map(uav_id, position, observations)
        self.collab_mapper.merge_maps()

    def get_formation_status(self) -> Dict:
        """获取编队状态"""
        commands = self.compute_velocity_commands()
        errors = {}
        for uid in self.uavs:
            target = self._target_positions.get(uid, np.zeros(3))
            actual = self.uavs[uid].position
            errors[uid] = np.linalg.norm(target - actual)

        return {
            'formation_type': self.formation_type.name,
            'num_uavs': self.num_uavs,
            'spacing': self.formation_spacing,
            'position_errors': errors,
            'max_error': max(errors.values()) if errors else 0,
            'velocity_commands': {k: v.tolist() for k, v in commands.items()},
            'target_positions': {k: v.tolist() for k, v in self._target_positions.items()},
        }

    async def formation_takeoff(self, altitude: float = 5.0):
        """编队起飞"""
        logger.info(f"Formation takeoff to {altitude}m")
        for uid in self.uavs:
            self._target_positions[uid][2] = -altitude
            if self.simulation:
                self.uavs[uid].position[2] = -altitude

    async def formation_land(self):
        """编队降落"""
        logger.info("Formation landing")
        for uid in self.uavs:
            self._target_positions[uid][2] = 0.0
            if self.simulation:
                self.uavs[uid].position[2] = 0.0

    async def formation_goto(self, north: float, east: float):
        """编队移动到指定位置"""
        self.move_formation(np.array([north, east, 0]))
        if self.simulation:
            for uid in self.uavs:
                self.uavs[uid].position = self._target_positions[uid].copy()

    async def run_simulation_step(self, dt: float = 0.1):
        """运行一步仿真"""
        commands = self.compute_velocity_commands()
        for uid, vel in commands.items():
            if uid in self.uavs:
                self.uavs[uid].position += vel * dt
                self.uavs[uid].velocity = vel

    async def run(self):
        """运行编队协调循环"""
        self.running = True
        while self.running:
            await self.run_simulation_step(0.1)
            await asyncio.sleep(0.1)

    def stop(self):
        """停止编队协调"""
        self.running = False


# ---------------------------------------------------------------------------
# CLI Test
# ---------------------------------------------------------------------------

async def test_swarm():
    """Test swarm coordinator"""
    coordinator = SwarmCoordinator(num_uavs=3, simulation=True)

    # Test V-shape formation
    print("=== V-Shape Formation ===")
    coordinator.set_formation(FormationType.V_SHAPE, spacing=5.0)
    status = coordinator.get_formation_status()
    print(f"Formation: {status['formation_type']}")
    print(f"Targets: {status['target_positions']}")

    # Test circle formation
    print("\n=== Circle Formation ===")
    coordinator.set_formation(FormationType.CIRCLE, spacing=8.0)
    status = coordinator.get_formation_status()
    print(f"Formation: {status['formation_type']}")
    print(f"Targets: {status['target_positions']}")

    # Test formation flight
    print("\n=== Formation Flight ===")
    await coordinator.formation_takeoff(5.0)
    await coordinator.formation_goto(10.0, 0.0)
    await coordinator.formation_land()
    print("Formation flight complete")


if __name__ == "__main__":
    asyncio.run(test_swarm())
