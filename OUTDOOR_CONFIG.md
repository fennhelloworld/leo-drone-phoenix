# LeoDrone Phoenix — 户外飞行配置指南

> **安全第一** — 每次户外飞行前必须完成所有检查项

---

## 🔧 硬件组装指南

### 第1步: 机架组装

```
         前
    M1 ↗    ↖ M2
       \  /
        \/
        /\
       /  \
    M4 ↙    ↘ M3
         后

电机布局 (X型, 从上方看)
M1: 前右 (CCW)    M2: 前左 (CW)
M3: 后左 (CCW)    M4: 后右 (CW)
```

1. 组装 F450 机架上下板
2. 安装4个电机座，注意方向
3. 电机编号: M1(前右CCW), M2(前左CW), M3(后左CCW), M4(后右CW)

### 第2步: 电气连接

#### 电机↔ESC↔电源
```
M1 → ESC1 → PDB 前右
M2 → ESC2 → PDB 前左
M3 → ESC3 → PDB 后左
M4 → ESC4 → PDB 后右
```

#### 飞控接线
```
Pixhawk 6C 引脚分配:
├── MAIN OUT 1-4  → ESC 1-4 (电机)
├── GPS/I2C       → M10N GPS + 磁力计
├── TELEM 1       → SiK 数传 (地面站)
├── TELEM 2       → RPi5 UART (伴飞)
├── RC IN         → FS-iA6B 接收机
├── ADC           → 电源模块 (电压/电流)
├── USB           → 配置/刷固件
└── BUS           → 扩展 (BME280)
```

#### 伴飞电脑连接
```
RPi5 / Jetson Orin Nano:
├── UART (/dev/ttyAMA0)  → Pixhawk TELEM2
├── CSI-0  → IMX219 前置
├── CSI-1  → IMX219 后置
├── USB-1  → IMX219 左侧 (USB适配)
├── USB-2  → IMX219 右侧 (USB适配)
├── I2C    → BME280 + MPU-6050
├── GPIO   → 状态LED
└── WiFi   → 图传 / Web Dashboard
```

### 第3步: 相机组安装

```
        前
   CAM-0 → 0°
   CAM-1 → 90° (右)
   CAM-2 → 180° (后)
   CAM-3 → 270° (左)

安装要点:
- 4路相机水平安装，间隔90°
- 镜头中心与机架中心同高
- 使用3D打印支架固定
- 标记相机编号和方向
```

### 第4步: 整机组装

```
自下而上:
1. 电池绑带 (底部)
2. 下板 + PDB + ESC
3. 电机 + 螺旋桨 (暂不装桨)
4. 飞控 (减震垫)
5. 伴飞电脑
6. GPS支架 (顶部)
7. 上板
8. 相机组 (侧面)
9. 数传天线
```

---

## 🎯 校准步骤

### 1. 飞控固件刷写

```bash
# 使用 QGroundControl 刷写
1. USB 连接 Pixhawk
2. QGC → Firmware → PX4 Stable
3. 选择 Pixhawk 6C
4. 等待刷写完成
```

### 2. 传感器校准

#### 罗盘校准
```
QGC → Sensors → Compass
1. 按提示旋转无人机 (6个方向)
2. 校准完成，检查偏差 < 5%
3. 如偏差过大，远离金属重试
```

#### 加速度计校准
```
QGC → Sensors → Accelerometer
1. 按提示放置6个面朝下
2. 每个方向保持3秒
3. 校准完成
```

#### 陀螺仪校准
```
QGC → Sensors → Gyroscope
1. 放置在水平面上
2. 保持静止
3. 点击校准
4. 等待完成
```

#### 水平面校准
```
QGC → Sensors → Level Horizon
1. 放置在水平面上
2. 点击校准
3. 验证地平线水平
```

### 3. 遥控校准

```
QGC → Radio → Calibrate
1. 遥控器开机
2. 按提示移动所有摇杆到极限位置
3. 设置模式开关 (手动/Stabilized/Offboard)
4. 验证通道映射正确
```

### 4. ESC 校准

```
QGC → Parameters → ESC
1. 拆除螺旋桨!
2. 遥控油门拉到最高
3. 给电调上电
4. 听到提示音后拉低油门
5. 确认4个ESC校准一致
```

### 5. 相机标定

```bash
# 使用棋盘格标定每路相机
cd /home/fenn/projects/leo-drone-phoenix/FIRMWARE
python3 -c "
import cv2
import numpy as np
import glob

# 棋盘格参数
chess_size = (9, 6)
square_size = 0.025  # 25mm

# 拍摄标定图像
images = glob.glob('calib_cam*/ *.jpg')

# 标定流程 (OpenCV)
objpoints = []
imgpoints = []
objp = np.zeros((chess_size[0]*chess_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chess_size[0], 0:chess_size[1]].T.reshape(-1, 2)
objp *= square_size

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    ret, corners = cv2.findChessboardCorners(gray, chess_size, None)
    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

# 计算内参和畸变系数
ret, K, dist, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)
print(f'内参矩阵 K:\\n{K}')
print(f'畸变系数: {dist.ravel()}')
"
```

### 6. 伴飞电脑配置

```bash
# RPi5 / Jetson 初始化
sudo apt update && sudo apt upgrade -y
sudo apt install -y python3-pip git i2c-tools

# 安装依赖
pip3 install mavsdk opencv-python numpy mediapipe \
  whisper torch torchvision pyaudio edge-tts

# 配置串口
sudo raspi-config  # 启用 I2C, SPI, Serial

# 配置 MAVLink 路由
# /etc/mavlink-router/main.conf
[UartEndpoint to_fc]
Device = /dev/ttyAMA0
Baud = 921600

[UdpEndpoint to_gcs]
Mode = Normal
Address = 0.0.0.0
Port = 14550
```

---

## ✅ 飞行安全检查清单

### 飞行前检查 (Pre-Flight)

| # | 检查项 | 通过 |
|---|--------|------|
| 1 | 天气条件良好 (风速 < 5级, 无雨) | □ |
| 2 | 飞行区域合法 (非禁飞区) | □ |
| 3 | 电池充满 (≥ 4.1V/cell) | □ |
| 4 | 螺旋桨紧固，无裂纹 | □ |
| 5 | 电机旋转方向正确 | □ |
| 6 | 遥控信号正常 | □ |
| 7 | GPS 锁定 (≥ 8 颗卫星) | □ |
| 8 | 罗盘校准有效 | □ |
| 9 | 数传连接正常 | □ |
| 10 | 伴飞电脑运行正常 | □ |
| 11 | 相机画面正常 | □ |
| 12 | Failsafe 参数设置正确 | □ |
| 13 | 地理围栏配置正确 | □ |
| 14 | 附近无人员/障碍物 | □ |
| 15 | 紧急降落点已确认 | □ |

### 飞行中监控 (In-Flight)

| # | 监控项 | 正常范围 |
|---|--------|----------|
| 1 | 高度 | < 120m AGL |
| 2 | 距离 | < 500m |
| 3 | 电池电压 | > 14.0V (4S) |
| 4 | 信号强度 | > 70% |
| 5 | GPS 卫星数 | > 8 |
| 6 | 电机温度 | < 80°C |
| 7 | 飞行模式 | 符合预期 |
| 8 | 位置漂移 | < 2m |
| 9 | 障碍物距离 | > 3m |
| 10 | 风速 | < 8m/s |

### 飞行后检查 (Post-Flight)

| # | 检查项 |
|---|--------|
| 1 | 电池电压记录 |
| 2 | 电机温度检查 |
| 3 | 螺旋桨损伤检查 |
| 4 | 结构完整性检查 |
| 5 | 飞行日志下载 |
| 6 | 问题记录 |

---

## ⚖️ 法律合规

### 中国无人机法规

| 要求 | 详情 |
|------|------|
| 实名登记 | 民航局UOM系统注册，贴标 |
| 驾驶资质 | 微型(≤250g)免证，其他需执照 |
| 飞行高度 | ≤ 120m AGL |
| 飞行区域 | 远离机场8km+, 避开禁飞区 |
| 视距内飞行 | VLOS ≤ 500m |
| 夜间飞行 | 禁止 (除非获批准) |
| 人群上空 | 禁止 |
| 保险 | 建议购买第三方责任险 |

### 禁飞区查询

- **民航局**: https://www.caac.gov.cn
- **大疆禁飞区**: https://www.dji.com/flysafe/geo-map
- **本地公安**: 咨询当地公安部门

### 飞行报备

```
1. 登录民航局UOM系统
2. 填写飞行计划
3. 等待审批 (通常24h)
4. 飞行前确认审批通过
5. 飞行后上传日志
```

---

## 🌤️ 环境条件

| 条件 | 可飞 | 限制 | 禁飞 |
|------|------|------|------|
| 风速 | < 5m/s | 5-8m/s | > 8m/s |
| 能见度 | > 3km | 1-3km | < 1km |
| 降雨 | 无 | 毛毛雨 | 中雨+ |
| 温度 | 0°C - 40°C | -10°C - 0°C | < -10°C |
| 湿度 | < 80% | 80-95% | > 95% |
| 气压 | 980-1030 hPa | - | - |

---

## 🔋 电池安全

### 充电规范
- 使用平衡充电器
- 充电电流 ≤ 1C (4A for 4000mAh)
- 充电电压 4.2V/cell (16.8V for 4S)
- 充电环境温度 10°C - 30°C
- 充电时必须有人看管

### 存储规范
- 存储电压 3.8V/cell (15.2V for 4S)
- 存储温度 10°C - 25°C
- 防火袋/防爆箱存储
- 远离易燃物

### 退役标准
- 鼓包 → 立即退役
- 容量 < 80% 标称 → 退役
- 内阻增大 > 20% → 退役
- 循环次数 > 200 → 考虑退役

---

## 🛠️ 工具清单

| 工具 | 用途 |
|------|------|
| 六角扳手套装 | 电机/机架螺丝 |
| 焊台 | 电气连接 |
| 万用表 | 电路检测 |
| 热缩管 | 线缆绝缘 |
| 扎带 | 线缆固定 |
| 3M胶 | 传感器固定 |
| 螺丝胶 | 防松 |
| 标定板 (棋盘格) | 相机标定 |
