# LeoDrone Phoenix — 室内仿真环境搭建指南

> **零硬件起步** — 只需要一台Linux电脑，即可运行所有仿真测试

---

## 📋 系统要求

| 要求 | 最低配置 | 推荐配置 |
|------|----------|----------|
| OS | Ubuntu 20.04 | Ubuntu 22.04 |
| CPU | 4核 x86_64 | 8核+ |
| RAM | 8GB | 16GB+ |
| GPU | 集成显卡 | NVIDIA RTX 3060+ |
| 硬盘 | 20GB | 50GB+ SSD |
| Docker | 20.10+ | 24.0+ |

---

## 🚀 快速开始 (5分钟)

### 1. 安装 Docker

```bash
# 安装 Docker
curl -fsSL https://get.docker.com | sudo sh
sudo usermod -aG docker $USER
newgrp docker

# 安装 Docker Compose
sudo apt install docker-compose-plugin
```

### 2. 一键启动仿真

```bash
cd /home/fenn/projects/leo-drone-phoenix
./run.sh --sim
```

或手动执行：

```bash
cd /home/fenn/projects/leo-drone-phoenix
make sim-start
```

### 3. 运行测试

```bash
./run.sh --test
```

---

## 📦 Docker 仿真环境

### PX4 SITL + Gazebo 容器

```bash
# 拉取镜像
docker pull px4io/px4-dev-ros2-gazebo:latest

# 或使用项目自带的 Dockerfile
cd SIMULATION/
docker build -t phoenix-sim .
```

### 启动仿真

```bash
# 启动 PX4 SITL + Gazebo
docker run -it --rm \
  -p 14540:14540 \    # MAVLink UDP
  -p 14550:14550 \    # MAVLink UDP (GCS)
  -p 5760:5760 \      # MAVLink TCP
  -p 8080:8080 \      # Gazebo Web
  --name phoenix-sitl \
  phoenix-sim
```

### 连接参数

| 连接方式 | 地址 | 用途 |
|----------|------|------|
| MAVSDK (UDP) | `udp://:14540` | 伴飞控制 |
| QGC (UDP) | `udp://:14550` | 地面站 |
| MAVLink (TCP) | `tcp://:5760` | 备用连接 |
| Gazebo Web | `http://:8080` | 3D可视化 |

---

## 🔧 逐项测试指南

### 测试1: 基础飞行控制

```bash
cd /home/fenn/projects/leo-drone-phoenix/SIMULATION
python3 sim_test_flight.py
```

**验证内容**:
- ✅ 起飞到指定高度
- ✅ 悬停稳定性
- ✅ 航点飞行
- ✅ 返航降落
- ✅ Offboard 模式切换

### 测试2: 360° 全景拼接

```bash
cd /home/fenn/projects/leo-drone-phoenix/SIMULATION
python3 sim_360_stitch.py
```

**验证内容**:
- ✅ 4路相机仿真数据生成
- ✅ 鱼眼去畸变
- ✅ 全景拼接
- ✅ 等距柱状投影输出

### 测试3: VINS-Fusion SLAM

```bash
cd /home/fenn/projects/leo-drone-phoenix/SIMULATION
python3 sim_slam.py
```

**验证内容**:
- ✅ 仿真IMU数据生成
- ✅ 仿真视觉特征
- ✅ VIO状态估计
- ✅ 轨迹漂移 < 1%

### 测试4: YOLOv8 目标追踪

```bash
cd /home/fenn/projects/leo-drone-phoenix/SIMULATION
python3 sim_tracking.py
```

**验证内容**:
- ✅ Gazebo中人物模型检测
- ✅ 多目标跟踪
- ✅ 追踪跟随控制
- ✅ 遮挡恢复

### 测试5: 编队飞行

```bash
cd /home/fenn/projects/leo-drone-phoenix/SIMULATION
python3 sim_swarm.py
```

**验证内容**:
- ✅ 3机编队起飞
- ✅ V字形编队保持
- ✅ 编队变换
- ✅ 编队降落

---

## 🧪 MAVSDK Python Offboard 控制示例

### 基本连接

```python
#!/usr/bin/env python3
"""基本MAVSDK连接示例"""

import asyncio
from mavsdk import System

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    print("等待连接...")
    async for state in drone.core.connection_state():
        if state.is_connected:
            print(f"已连接!")
            break

    # 获取遥测数据
    async for position in drone.telemetry.position():
        print(f"高度: {position.relative_altitude_m:.2f}m")
        break

    await drone.close()

if __name__ == "__main__":
    asyncio.run(main())
```

### Offboard 起飞

```python
#!/usr/bin/env python3
"""Offboard模式起飞示例"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    # 等待连接
    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    # 解锁
    print("解锁...")
    await drone.action.arm()

    # 设置初始offboard位置
    print("设置offboard位置...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, 0.0, 0.0)
    )

    # 启动offboard模式
    print("启动offboard...")
    try:
        await drone.offboard.start()
    except OffboardError as e:
        print(f"Offboard启动失败: {e}")
        await drone.action.disarm()
        return

    # 上升到5米
    print("上升到5米...")
    await drone.offboard.set_position_ned(
        PositionNedYaw(0.0, 0.0, -5.0, 0.0)
    )
    await asyncio.sleep(5)

    # 停止offboard
    print("停止offboard...")
    await drone.offboard.stop()

    # 降落
    await drone.action.land()

if __name__ == "__main__":
    asyncio.run(main())
```

### 航点飞行

```python
#!/usr/bin/env python3
"""航点飞行示例"""

import asyncio
from mavsdk import System
from mavsdk.offboard import OffboardError, PositionNedYaw

WAYPOINTS = [
    PositionNedYaw(0.0, 0.0, -5.0, 0.0),     # 北5m, 高5m
    PositionNedYaw(10.0, 0.0, -5.0, 0.0),     # 北10m
    PositionNedYaw(10.0, 10.0, -5.0, 90.0),   # 东10m, 朝东
    PositionNedYaw(0.0, 10.0, -5.0, 180.0),   # 南10m, 朝南
    PositionNedYaw(0.0, 0.0, -5.0, 270.0),    # 西10m, 朝西
]

async def main():
    drone = System()
    await drone.connect(system_address="udp://:14540")

    async for state in drone.core.connection_state():
        if state.is_connected:
            break

    await drone.action.arm()
    await drone.offboard.set_position_ned(WAYPOINTS[0])
    await drone.offboard.start()

    # 逐个飞往航点
    for i, wp in enumerate(WAYPOINTS):
        print(f"飞往航点 {i+1}/{len(WAYPOINTS)}")
        await drone.offboard.set_position_ned(wp)
        await asyncio.sleep(5)

    # 返航
    await drone.offboard.stop()
    await drone.action.return_to_launch()

if __name__ == "__main__":
    asyncio.run(main())
```

---

## 📡 仿真传感器数据

### 仿真 IMU 数据

```python
#!/usr/bin/env python3
"""仿真IMU数据生成"""

import numpy as np
import time

class SimulatedIMU:
    """仿真IMU，模拟ICM-42688-P"""

    def __init__(self, sample_rate=200.0, noise_density=0.01):
        self.sample_rate = sample_rate
        self.dt = 1.0 / sample_rate
        self.noise_density = noise_density
        self.gravity = np.array([0, 0, 9.81])

    def get_measurement(self, true_accel, true_gyro):
        """添加噪声到真实值"""
        accel_noise = np.random.normal(0, self.noise_density, 3)
        gyro_noise = np.random.normal(0, self.noise_density * 0.1, 3)
        accel_bias = np.random.normal(0, 0.001, 3)

        return {
            'accel': true_accel + self.gravity + accel_noise + accel_bias,
            'gyro': true_gyro + gyro_noise,
            'timestamp': time.time()
        }
```

### 仿真 GPS 数据

```python
#!/usr/bin/env python3
"""仿真GPS数据生成"""

import numpy as np

class SimulatedGPS:
    """仿真GPS，模拟M10N"""

    def __init__(self, update_rate=10.0, noise_std=2.5):
        self.update_rate = update_rate
        self.noise_std = noise_std  # 米

    def get_measurement(self, true_lat, true_lon, true_alt):
        """添加GPS噪声"""
        lat_noise = np.random.normal(0, self.noise_std * 1e-7)
        lon_noise = np.random.normal(0, self.noise_std * 1e-7)
        alt_noise = np.random.normal(0, self.noise_std)

        return {
            'lat': true_lat + lat_noise,
            'lon': true_lon + lon_noise,
            'alt': true_alt + alt_noise,
            'fix_type': 3,  # 3D fix
            'num_satellites': np.random.randint(8, 14),
            'hdop': np.random.uniform(0.5, 1.5)
        }
```

### 仿真相机数据

```python
#!/usr/bin/env python3
"""仿真4路相机数据生成"""

import numpy as np

class SimulatedCameraRig:
    """仿真4路IMX219鱼眼相机"""

    def __init__(self, resolution=(640, 480), fov_deg=160):
        self.resolution = resolution
        self.fov = np.radians(fov_deg)
        self.num_cameras = 4

        # 每路相机朝向 (前/右/后/左)
        self.yaw_offsets = [0, 90, 180, 270]

    def generate_frame(self, camera_id, scene_features=None):
        """生成单路仿真帧"""
        frame = np.random.randint(0, 255,
            (*self.resolution, 3), dtype=np.uint8)

        # 添加仿真特征点
        if scene_features:
            for feat in scene_features:
                x, y = feat['pixel_pos']
                frame[max(0,y-2):y+2, max(0,x-2):x+2] = [255, 255, 255]

        return {
            'frame': frame,
            'camera_id': camera_id,
            'yaw': self.yaw_offsets[camera_id],
            'timestamp': time.time()
        }

    def generate_all_frames(self, scene_features=None):
        """生成所有4路帧"""
        return [self.generate_frame(i, scene_features)
                for i in range(self.num_cameras)]
```

---

## 🔬 各功能仿真测试方法

### 追踪拍摄 (Tracking Shot)
1. Gazebo 中放置移动人物模型
2. 启动 YOLOv8 检测
3. Offboard 模式跟随目标
4. 验证跟踪稳定性

### 视频防抖 (Video Stabilization)
1. 生成高频振动轨迹
2. 仿真相机帧序列
3. 运行 EIS 算法
4. 对比稳