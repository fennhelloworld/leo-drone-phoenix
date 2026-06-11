# LeoDrone Phoenix — 360°全景AI穿越无人机

<div align="center">

**全功能AI无人机平台 · 从室内仿真到户外飞行**

[![Python 3.10](https://img.shields.io/badge/Python-3.10-blue.svg)](https://python.org)
[![PX4 SITL](https://img.shields.io/badge/PX4-SITL-green.svg)](https://px4.io)
[![MAVSDK](https://img.shields.io/badge/MAVSDK-Python-orange.svg)](https://mavsdk.io)
[![Tests](https://img.shields.io/badge/Tests-30+-brightgreen.svg)](#testing)

</div>

---

## 🌟 核心特性

### 🚁 飞行能力
| 特性 | 描述 |
|------|------|
| 360°全景飞行 | 四方向自由飞行，无死角机动 |
| 智能路径规划 | A* / RRT* 自主避障路径规划 |
| 自主导航定位 | GPS+视觉融合定位，厘米级精度 |
| 跟随飞行 | 目标锁定，智能跟随，保持构图 |
| 编队飞行 | 多机协同编队，蜂群协作 |
| 协同建图 | 多机联合SLAM，实时地图融合 |

### 📹 拍摄与视频
| 特性 | 描述 |
|------|------|
| 360°全景拍摄 | 4×IMX219 全景拼接，等距柱状投影 |
| 视频防抖 | 电子图像防抖(EIS)，卡尔曼滤波 |
| 追踪拍摄 | YOLOv8 目标追踪，自动构图 |
| 实时视频编辑 | 飞行中实时剪辑，背景切换 |
| 人物分割 | 实时人物抠图，虚拟背景合成 |
| 手势拍摄 | 手势识别控制拍摄开始/停止 |

### 🧠 AI与感知
| 特性 | 描述 |
|------|------|
| VINS-Fusion SLAM | 视觉惯性里程计，实时建图定位 |
| 3D建模 | 实时稠密重建，点云生成 |
| 视觉里程计 | 单目/双目视觉里程计 |
| 人物识别 | YOLOv8 + ReID 多目标跟踪 |
| 手势识别 | MediaPipe 21点手部关键点 |
| 速度测量 | 光流法 + IMU 融合测速 |

### 🗣️ 交互与控制
| 特性 | 描述 |
|------|------|
| LLM语音对话 | 唤醒词 + STT + DeepSeek LLM + TTS |
| 语音控制 | 自然语言指令→飞行控制 |
| 世界模型 | GOAT-Mamba + DeepSeekMoE 预测推理 |
| 因果推理 | CausalGraph 因果关系建模 |
| 时序知识图谱 | TemporalKG 时序推理 |

### 🌍 环境感知
| 特性 | 描述 |
|------|------|
| 水文感知 | BME280 温湿度气压传感 |
| 气象预警 | 实时气象数据分析与预警 |
| 物理建模 | 流体力学 + 热力学建模 |
| 地理围栏 | 自动禁飞区检测与规避 |
| 障碍物避障 | 3D障碍物检测与规避 |
| 安全保障 | 多重Failsafe机制 |

---

## 📋 两阶段开发路线

### Phase 1: 室内仿真 (0-3个月)
- ✅ 零硬件需求，纯软件仿真
- ✅ PX4 SITL + Gazebo 仿真环境
- ✅ 所有AI算法在仿真中验证
- ✅ 30+自动化测试全部通过
- 最低配置：一台开发电脑即可

### Phase 2: 户外飞行 (3-6个月)
- 🔧 Pixhawk 6C + 伴飞电脑组装
- 🔧 4×IMX219 相机组安装标定
- 🔧 户外校准与安全验证
- 🔧 合规注册与飞行许可

---

## 🔧 硬件清单 (BOM)

### 最低仿真配置 (Phase 1)
| 组件 | 型号 | 数量 | 用途 | 价格 |
|------|------|------|------|------|
| — | 仅需电脑 | 1 | SITL仿真无需硬件 | ¥0 |

### 完整户外配置 (Phase 2)
| 组件 | 型号 | 数量 | 用途 | 参考价 |
|------|------|------|------|--------|
| 飞控 | Pixhawk 6C | 1 | 飞行控制 | ¥580 |
| 伴飞电脑 | Raspberry Pi 5 (8GB) | 1 | AI推理/通信 | ¥480 |
| 伴飞电脑(高配) | Jetson Orin Nano 8GB | 1 | 高性能AI推理 | ¥1,599 |
| 相机 | IMX219 模组 | 4 | 360°全景拍摄 | ¥280 |
| 气压温湿度 | BME280 | 1 | 气象感知 | ¥15 |
| IMU | ICM-42688-P | 1 | 高精度惯性测量 | ¥35 |
| GPS | M10N GPS模组 | 1 | 定位导航 | ¥85 |
| 电调 | 30A ESC × 4 | 1套 | 电机驱动 | ¥160 |
| 电机 | 2212 920KV × 4 | 1套 | 动力 | ¥200 |
| 螺旋桨 | 1045 正反桨 × 4 | 1套 | 推进 | ¥30 |
| 机架 | F450 四轴机架 | 1 | 结构 | ¥120 |
| 电池 | 4S 4000mAh LiPo | 2 | 电源 | ¥280 |
| 遥控 | Flysky FS-i6S | 1 | 手动控制 | ¥280 |
| 接收机 | FS-iA6B | 1 | 遥控信号接收 | ¥65 |
| 数传 | SiK Telemetry V2 | 1对 | 地面站通信 | ¥80 |
| 图传 | ESP32-CAM WiFi | 1 | 视频传输 | ¥35 |
| 电源模块 | Matek PDB | 1 | 配电 | ¥45 |
| 连接线材 | 各类杜邦线/排线 | 1套 | 连接 | ¥50 |
| **总计** | | | | **¥4,419** |

### 推荐升级选项
| 升级 | 替换 | 差价 |
|------|------|------|
| Jetson Orin Nano | RPi5 | +¥1,119 |
| OAK-D Pro 相机 | IMX219 × 4 | +¥800 |
| 高增益天线 | 默认天线 | +¥60 |

---

## 🚀 快速开始

```bash
# 克隆项目
cd /home/fenn/projects/leo-drone-phoenix

# 运行仿真测试
./run.sh --test

# 启动SITL仿真
./run.sh --sim

# 运行功能演示
./run.sh --demo

# 户外飞行准备
./run.sh --outdoor-prep

# 安全审计
./run.sh --safety
```

---

## 📁 项目结构

```
leo-drone-phoenix/
├── README.md              # 本文件
├── ARCHITECTURE.md        # 9层架构文档
├── INDOOR_SIM_SETUP.md    # 室内仿真指南
├── OUTDOOR_CONFIG.md      # 户外飞行配置
├── SAFETY.md              # 安全文档
├── Makefile               # 构建与测试
├── run.sh                 # 一键入口脚本
├── FIRMWARE/
│   ├── main.py            # 主伴飞计算机脚本
│   ├── sensor_node.py     # 传感器节点
│   ├── voice_assistant.py # 语音助手
│   ├── gesture_controller.py  # 手势控制
│   ├── swarm_coordinator.py   # 编队协调
│   └── video_editor.py    # 实时视频编辑
├── SIMULATION/
│   ├── sitl_start.sh      # SITL启动脚本
│   ├── sim_test_flight.py # 仿真飞行测试
│   ├── sim_360_stitch.py  # 全景拼接仿真
│   ├── sim_slam.py        # SLAM仿真
│   ├── sim_tracking.py    # 目标追踪仿真
│   └── sim_swarm.py       # 编队仿真
└── tests/
    └── test_phoenix.py    # 30+自动化测试
```

---

## 📚 相关项目

- **[omni-perception-fusion](../omni-perception-fusion/)** — 全感知融合库（8项专利，42个测试）
- **[drone-system](../drone-system/)** — 基础无人机系统（3个硬件方案，119个测试）
- **[leo-drone-ultimate](../leo-drone-ultimate/)** — Case A 基础版无人机

---

## 📄 许可证

MIT License — 自由使用、修改和分发

---

<div align="center">

**LeoDrone Phoenix — 让AI赋予无人机全视角智慧**

</div>
