# LeoDrone Phoenix — 9层架构设计

```
┌─────────────────────────────────────────────────────────┐
│                    L8 Ground Station                     │
│        QGC · Web Dashboard · 3D Visualization            │
├─────────────────────────────────────────────────────────┤
│                    L7 Safety                              │
│  Geo-fence · Obstacle Avoidance · Weather · Failsafe     │
├─────────────────────────────────────────────────────────┤
│                    L6 Interaction                         │
│    Voice LLM · Gesture · Real-time Video Edit            │
├─────────────────────────────────────────────────────────┤
│                    L5 Coordination                        │
│   UAV Planner · Swarm · Collaborative Mapping            │
├─────────────────────────────────────────────────────────┤
│                    L4 Cognition                           │
│     GOAT-Mamba · DeepSeekMoE · WorldModel                │
├─────────────────────────────────────────────────────────┤
│                    L3 Fusion                              │
│        EKF · CausalGraph · TemporalKG                    │
├─────────────────────────────────────────────────────────┤
│                    L2 Perception                          │
│  360° Stitch · EIS · VINS-Fusion · YOLOv8               │
├─────────────────────────────────────────────────────────┤
│                    L1 Sensing                             │
│    IMU · GPS · Baro · Camera · Thermo-Hygro              │
├─────────────────────────────────────────────────────────┤
│                    L0 Hardware                            │
│  Pixhawk 6C · RPi5/Orin Nano · 4×IMX219 · BME280       │
└─────────────────────────────────────────────────────────┘
```

---

## L0 — 硬件层 (Hardware)

### 核心计算平台
| 组件 | 型号 | 规格 | 接口 |
|------|------|------|------|
| 飞控 | Pixhawk 6C | STM32H743, 480MHz | UART, I2C, SPI, CAN |
| 伴飞电脑 | RPi5 8GB / Jetson Orin Nano | ARM Cortex-A76 / Ampere GPU | USB3, CSI, GPIO, I2C |
| 存储扩展 | NVMe SSD (可选) | 256GB | PCIe (RPi5) |

### 传感器组
| 传感器 | 型号 | 接口 | 采样率 |
|--------|------|------|--------|
| IMU (飞控) | ICM-42688-P | SPI | 8kHz |
| IMU (伴飞) | MPU-6050 | I2C | 1kHz |
| GPS | M10N | UART | 10Hz |
| 气压计 | BME280 | I2C | 50Hz |
| 温湿度 | BME280 | I2C | 1Hz |
| 相机组 | IMX219 × 4 | CSI-2 | 30fps @ 1080p |

### 通信链路
| 链路 | 协议 | 频率 | 带宽 |
|------|------|------|------|
| 遥控 | IBUS/SBUS | 2.4GHz | - |
| 数传 | MAVLink/SiK | 433MHz | 57600bps |
| 图传 | WiFi RTSP | 5.8GHz | 20Mbps |
| 伴飞↔飞控 | MAVLink/UART | 有线 | 921600bps |

### 电源系统
| 组件 | 规格 | 输出 |
|------|------|------|
| 主电池 | 4S 4000mAh LiPo | 14.8V |
| BEC | 5V/3A | 飞控/伴飞 |
| 电源模块 | Matek PDB | 5V/12V |

---

## L1 — 感知层 (Sensing)

### IMU 数据流
```
ICM-42688-P → SPI → Pixhawk → EKF → MAVLink → Companion
                                                      ↓
MPU-6050    → I2C → RPi5  → VINS-Fusion ←────────────┘
```

- **加速度计**: ±16g, 采样率 8kHz → 降采样 200Hz
- **陀螺仪**: ±2000°/s, 采样率 8kHz → 降采样 200Hz
- **磁力计**: QMC5883L, ±8 Gauss

### GPS 数据流
```
M10N → UART → Pixhawk → NMEA/UBX → EKF → 位置/速度
                                       → 10Hz 更新
```

- **定位精度**: RTK 1cm / 标准 2.5m
- **冷启动**: < 26s
- **支持星座**: GPS + GLONASS + Galileo + BeiDou

### 气压/温湿度数据流
```
BME280 → I2C → RPi5 → 气压高度 / 温度 / 湿度
                      → 气象预警模块
```

- **气压精度**: ±1 hPa
- **温度范围**: -40°C ~ +85°C, ±1°C
- **湿度精度**: ±3% RH

### 相机数据流
```
IMX219-Front  ┐
IMX219-Right  ├→ CSI-2 → RPi5 → 360° Stitch → EIS → SLAM/Detect
IMX219-Back   │                                    ↓
IMX219-Left   ┘                          视频编码 → RTSP推流
```

- **分辨率**: 1080p30 / 720p60
- **FOV**: 160° 鱼眼 (每路)
- **同步**: 硬件触发同步

---

## L2 — 感知层 (Perception)

### 360° 全景拼接
```
4×Fisheye → 去畸变 → 特征匹配 → 拼接 → 等距柱状投影 → 4K输出
```

- **算法**: OpenCV Stitcher + APAP
- **输出**: 3840×1920 equirectangular @ 30fps
- **延迟**: < 50ms (GPU加速)

### 电子图像防抖 (EIS)
```
原始帧 → 特征点跟踪 → 运动估计 → 卡尔曼平滑 → 仿射变换 → 稳定帧
```

- **算法**: Lucas-Kanade + Kalman Filter
- **补偿范围**: ±5° 旋转, ±10像素平移
- **裁剪**: 5% 边缘裁剪

### VINS-Fusion SLAM
```
Camera + IMU → 特征提取 → 光流跟踪 → 状态估计 → 滑窗优化
                                         ↓
                                    局部地图 → 回环检测 → 全局优化
```

- **算法**: VINS-Fusion (VIO)
- **精度**: 1% 漂移 (短距离)
- **频率**: 200Hz (IMU), 30Hz (视觉)
- **依赖**: `omni_perception_fusion.slam`

### YOLOv8 目标检测
```
帧 → YOLOv8-Nano → BBox + Class + Conf → NMS → 跟踪器
                                                ↓
                                           DeepSORT → ID跟踪
```

- **模型**: YOLOv8n (4.1MB) / YOLOv8s (11.2MB)
- **类别**: person, vehicle, animal 等
- **速度**: 30fps (Orin Nano), 15fps (RPi5)
- **依赖**: `omni_perception_fusion.detection`

---

## L3 — 融合层 (Fusion)

### 扩展卡尔曼滤波 (EKF)
```
IMU(预测) + GPS(观测) + Baro(观测) + Vision(观测) → EKF → 状态估计
                                                               ↓
                                                    位置/速度/姿态/偏差
```

- **状态维度**: 24 (位置3 + 速度3 + 姿态4 + 陀螺偏差3 + 加速度偏差3 + 磁偏差3 + 气压偏差1 + 风速3 + 地磁倾角1)
- **频率**: 200Hz 输出
- **依赖**: `omni_perception_fusion.ekf`

### 因果图 (CausalGraph)
```
观测数据 → 结构学习 → 因果图 → 干预推理 → 决策建议
```

- **节点**: 环境变量、飞行状态、目标行为
- **边**: 因果关系 (有向)
- **推理**: do-calculus 干预效果估计
- **依赖**: `omni_perception_fusion.causal`

### 时序知识图谱 (TemporalKG)
```
时序事件 → 实体/关系抽取 → 时序图嵌入 → 时序推理 → 预测
```

- **时间粒度**: 秒级 / 分钟级 / 小时级
- **实体**: 位置、目标、天气、障碍物
- **关系**: 接近、远离、遮挡、影响
- **依赖**: `omni_perception_fusion.temporal_kg`

---

## L4 — 认知层 (Cognition)

### GOAT-Mamba 状态空间模型
```
时序状态 → Mamba SSM → 长程依赖建模 → 状态预测
```

- **架构**: Mamba (选择性状态空间模型)
- **序列长度**: 支持无限长上下文
- **参数量**: ~30M (轻量部署)
- **用途**: 飞行状态预测、轨迹规划

### DeepSeekMoE 混合专家
```
输入 → Router → Expert Selection → 专家推理 → 输出
```

- **专家数量**: 8 (导航/避障/跟踪/语音/视觉/规划/安全/通用)
- **激活专家**: Top-2 per token
- **总参数**: ~1.5B, 激活 ~300M
- **依赖**: `omni_perception_fusion.moe`

### 世界模型 (WorldModel)
```
当前状态 + 动作 → 动力学模型 → 下一状态预测 → 风险评估
```

- **状态空间**: 位置、速度、姿态、环境
- **动作空间**: 6-DOF 运动指令
- **预测范围**: 1-10秒
- **训练**: 在线学习 + 仿真预训练
- **依赖**: `omni_perception_fusion.world_model`

---

## L5 — 协调层 (Coordination)

### UAV 路径规划器
```
目标点 + 地图 → A*/RRT* → 轨迹生成 → 平滑 → MAVLink指令
```

- **算法**: A* (全局) + RRT* (局部) + MINCO (轨迹优化)
- **约束**: 最大速度/加速度、禁飞区、障碍物
- **重规划**: < 100ms (环境变化触发)

### 蜂群编队
```
编队配置 → 位置分配 → 一致性协议 → 分布式控制 → MAVLink
```

- **编队类型**: V字形、环形、线形、自主
- **一致性**: Leader-Follower / 虚拟结构
- **通信**: 广播 + 点对点
- **规模**: 支持 3-10 架 UAV

### 协同建图
```
UAV-1 地图 ┐
UAV-2 地图 ├→ 地图融合 → 全局一致地图 → 分布式存储
UAV-3 地图 ┘
```

- **表示**: OctoMap (体素) + 2.5D 栅格
- **融合**: 基于位姿图优化
- **更新率**: 1Hz (全局), 10Hz (局部)
- **依赖**: `omni_perception_fusion.mapping`

---

## L6 — 交互层 (Interaction)

### 语音 LLM 对话
```
麦克风 → VAD → 唤醒词检测 → STT(Whisper) → LLM(DeepSeek) → TTS → 扬声器
```

- **唤醒词**: "Phoenix" / "凤凰"
- **STT**: Whisper-Tiny (中文+英文)
- **LLM**: DeepSeek-V2-Lite (本地) / API (远程)
- **TTS**: Edge-TTS / VITS
- **延迟**: < 2s (端到端)

### 手势控制
```
相机 → MediaPipe → 21关键点 → 手势分类 → MAVLink指令
```

- **手势集**:
  - 👆 指向上 → 上升
  - 👇 指向下 → 下降
  - ✋ 张开手掌 → 悬停
  - ✊ 握拳 → 拍照
  - 👈 左滑 → 左移
  - 👉 右滑 → 右移
  - 🤏 捏合 → 变焦
  - 👋 挥手 → 跟随模式
- **延迟**: < 100ms
- **距离**: 0.5m - 5m

### 实时视频编辑
```
原始帧 → 人物分割 → 背景替换 → 滤镜叠加 → 编码输出
```

- **人物分割**: MediaPipe Selfie Segmentation
- **背景库**: 预设 + 用户上传
- **滤镜**: 电影级调色预设
- **输出**: RTSP 实时流 / 本地录制

---

## L7 — 安全层 (Safety)

### 地理围栏
```
GPS位置 → 地理围栏数据库 → 区域判定 → 告警/限制
```

- **围栏类型**: 禁飞区(NFZ)、限高区、授权区
- **数据源**: 本地数据库 + 在线更新
- **响应**: 软限制(告警) / 硬限制(自动返航)

### 障碍物避障
```
点云/深度图 → 障碍物检测 → 距离估计 → 避障路径 → 控制指令
```

- **检测范围**: 0.5m - 20m
- **避障策略**: 速度障碍法 (VO) + 人工势场法 (APF)
- **响应时间**: < 50ms

### 气象预警
```
BME280 + GPS → 气象数据 → 风速估计 → 阵风检测 → 预警等级
```

- **监测**: 温度、湿度、气压变化率、风速
- **预警级别**: 绿/黄/橙/红
- **响应**: 限制飞行 / 自动返航 / 紧急降落

### Failsafe 机制
```
异常检测 → 分类 → 响应策略
```

| 异常类型 | 响应策略 |
|----------|----------|
| 信号丢失 | 返航 (5s无信号) |
| 低电量 | 返航 (20%) / 降落 (10%) |
| GPS丢失 | 视觉定位 / 悬停 |
| 避障触发 | 刹车 / 绕行 |
| 地理围栏 | 返回围栏内 |
| 伴飞宕机 | 飞控独立控制 |
| 电机故障 | 紧急降落 |
| 过温保护 | 降低功率 / 降落 |

---

## L8 — 地面站层 (Ground Station)

### QGroundControl
```
QGC ←→ MAVLink ←→ UAV
  ↓
任务规划 · 参数配置 · 实时监控 · 日志分析
```

- **连接**: SiK 数传 / WiFi / USB
- **功能**: 任务规划、参数调优、固件升级、日志下载

### Web Dashboard
```
浏览器 ←→ WebSocket ←→ UAV Server
  ↓
实时数据 · 历史回放 · AI分析 · 远程控制
```

- **技术栈**: Vue3 + ECharts + Three.js
- **功能**: 3D轨迹、实时视频、气象数据、编队状态

### 3D 可视化
```
地图数据 + UAV状态 → Three.js → 3D场景渲染
  ↓
地形 · 建筑 · UAV模型 · 轨迹 · 传感器数据叠加
```

- **引擎**: Three.js / CesiumJS
- **数据源**: DEM + OSM + UAV实时状态
- **更新率**: 30fps

---

## 数据流总览

```
传感器 ──→ 感知 ──→ 融合 ──→ 认知 ──→ 协调 ──→ 控制
  │         │        │        │        │        │
  │         ↓        ↓        ↓        ↓        ↓
  │      360°全景   EKF状态  世界模型  路径规划  MAVLink
  │      目标检测   因果图    预测推理  编队控制  PWM
  │      SLAM      时序KG    MoE推理  协同建图  遥控
  │         │        │        │        │        │
  ↓         ↓        ↓        ↓        ↓        ↑
安全层 ◄────┴────────┴────────┴────────┘        │
  │ 避障 · 围栏 · 气象 · Failsafe                │
  ↓                                              │
地面站 ◄────────── 状态/视频/日志 ────────────────┘
```
