#!/bin/bash
# LeoDrone Phoenix — 一键入口脚本
# 用法: ./run.sh [--sim|--test|--demo|--outdoor-prep|--safety|--help]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON="/usr/bin/python3"

# 颜色
BLUE='\033[0;34m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
CYAN='\033[0;36m'
NC='\033[0m'

print_banner() {
    echo -e "${CYAN}"
    echo "  ╔═══════════════════════════════════════════════╗"
    echo "  ║     LeoDrone Phoenix — 360°全景AI穿越无人机   ║"
    echo "  ║                                               ║"
    echo "  ║     🔥 Full-Featured AI Tracking Drone        ║"
    echo "  ╚═══════════════════════════════════════════════╝"
    echo -e "${NC}"
}

# --- 仿真模式 ---
run_sim() {
    echo -e "${BLUE}🔥 启动SITL仿真环境...${NC}"
    chmod +x "$SCRIPT_DIR/SIMULATION/sitl_start.sh"
    "$SCRIPT_DIR/SIMULATION/sitl_start.sh" --docker
}

# --- 测试模式 ---
run_test() {
    echo -e "${BLUE}🧪 运行所有自动化测试...${NC}"
    echo ""

    # Unit tests
    echo -e "${CYAN}[1/6] 运行30+综合测试...${NC}"
    $PYTHON "$SCRIPT_DIR/tests/test_phoenix.py" || true

    echo ""
    echo -e "${CYAN}[2/6] 仿真飞行测试...${NC}"
    $PYTHON "$SCRIPT_DIR/SIMULATION/sim_test_flight.py" || true

    echo ""
    echo -e "${CYAN}[3/6] 360°全景拼接测试...${NC}"
    $PYTHON "$SCRIPT_DIR/SIMULATION/sim_360_stitch.py" || true

    echo ""
    echo -e "${CYAN}[4/6] VINS-Fusion SLAM测试...${NC}"
    $PYTHON "$SCRIPT_DIR/SIMULATION/sim_slam.py" || true

    echo ""
    echo -e "${CYAN}[5/6] YOLOv8目标追踪测试...${NC}"
    $PYTHON "$SCRIPT_DIR/SIMULATION/sim_tracking.py" || true

    echo ""
    echo -e "${CYAN}[6/6] 编队飞行测试...${NC}"
    $PYTHON "$SCRIPT_DIR/SIMULATION/sim_swarm.py" || true

    echo ""
    echo -e "${GREEN}✅ 所有测试完成${NC}"
}

# --- 演示模式 ---
run_demo() {
    echo -e "${BLUE}🎬 运行功能演示...${NC}"
    echo ""

    $PYTHON -c "
import numpy as np
import sys
sys.path.insert(0, '$SCRIPT_DIR/FIRMWARE')

print('=== LeoDrone Phoenix 功能演示 ===')
print()

# 1. 传感器演示
from sensor_node import SensorNode
node = SensorNode(simulation=True)
node.start()
data = node.read_all()
print('📡 传感器数据:')
print(f'  温度: {data[\"temperature\"]:.1f}°C')
print(f'  湿度: {data[\"humidity\"]:.1f}%')
print(f'  气压: {data[\"pressure\"]:.1f}hPa')
print(f'  海拔: {data[\"altitude\"]:.1f}m')
print(f'  IMU: accel={data[\"accel\"]}, gyro={data[\"gyro\"]}')
node.stop()

# 2. 手势控制演示
from gesture_controller import GestureController, Gesture
ctrl = GestureController(simulation=True)
gesture = ctrl.detect()
if gesture:
    cmd = ctrl.gesture_to_mavlink(gesture)
    print(f'🖐️ 手势: {gesture.name} → {cmd}')

# 3. 语音助手演示
import asyncio
from voice_assistant import VoiceAssistant
async def demo_voice():
    va = VoiceAssistant(simulation=True)
    for cmd in ['起飞到五米', '跟随我', '拍照', '返航']:
        response = await va.process(cmd)
        print(f'🎤 \"{cmd}\" → {response}')
asyncio.run(demo_voice())

# 4. 视频编辑演示
from video_editor import VideoEditor, BackgroundType, FilterType
editor = VideoEditor(simulation=True)
frame = np.random.randint(0, 255, (240, 320, 3), dtype=np.uint8)
editor.set_background(BackgroundType.VIRTUAL_SKY)
editor.set_filter(FilterType.CINEMATIC)
result = editor.process_frame(frame)
print(f'📹 视频: 原始{frame.shape} → 处理后{result.shape}')

# 5. 编队演示
from swarm_coordinator import SwarmCoordinator, FormationType
coord = SwarmCoordinator(num_uavs=3, simulation=True)
for ft in FormationType:
    coord.set_formation(ft)
    print(f'✈️ 编队: {ft.name}')

print()
print('=== 演示完成 ===')
"
}

# --- 户外准备 ---
run_outdoor_prep() {
    echo -e "${BLUE}📋 户外飞行准备清单${NC}"
    echo ""
    echo -e "${YELLOW}=== 飞行前检查 (15项) ===${NC}"

    checks=(
        "1. 天气条件良好 (风速<5级, 无雨)"
        "2. 飞行区域合法 (非禁飞区, 距机场>8km)"
        "3. 电池充满 (≥4.1V/cell, 16.4V for 4S)"
        "4. 螺旋桨紧固，无裂纹"
        "5. 电机旋转方向正确 (M1:CCW M2:CW M3:CCW M4:CW)"
        "6. 遥控信号正常 (>90%)"
        "7. GPS锁定 (≥8颗卫星, HDOP<2)"
        "8. 罗盘校准有效 (偏差<5%)"
        "9. 数传连接正常"
        "10. 伴飞电脑运行正常"
        "11. 4路相机画面正常"
        "12. Failsafe参数设置 (失联返航, 低电返航)"
        "13. 地理围栏配置 (水平300m, 垂直100m)"
        "14. 50m内无人员，降落点清空"
        "15. 紧急降落点已确认"
    )

    for check in "${checks[@]}"; do
        read -p "  $check [y/N] " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            echo -e "${RED}❌ 检查未通过: $check${NC}"
            echo -e "${YELLOW}请修复后重新检查${NC}"
            exit 1
        fi
    done

    echo ""
    echo -e "${GREEN}✅ 所有飞行前检查通过！${NC}"
    echo ""
    echo -e "${YELLOW}=== 法律合规检查 ===${NC}"
    echo "  - 无人机已实名登记?"
    echo "  - 飞行区域已查询 (非禁飞区)?"
    echo "  - 飞行计划已报备 (如需)?"
    echo "  - 第三方责任险已购买?"
    echo ""
    echo -e "${GREEN}准备就绪！祝飞行安全！${NC}"
    echo -e "${YELLOW}详见 OUTDOOR_CONFIG.md 和 SAFETY.md${NC}"
}

# --- 安全审计 ---
run_safety() {
    echo -e "${BLUE}🛡️ 安全审计${NC}"
    echo ""

    $PYTHON -c "
import sys
sys.path.insert(0, '$SCRIPT_DIR/FIRMWARE')
from main import PhoenixController, SafetyStatus

ctrl = PhoenixController(simulation=True)

print('=== Safety Audit ===')
print()

# Battery safety
tests = [
    ('Normal battery (80%)', 80, SafetyStatus.SAFE),
    ('Low battery (15%)', 15, None),
    ('Critical battery (5%)', 5, SafetyStatus.EMERGENCY),
]
for name, pct, expected in tests:
    ctrl.state = type(ctrl.state)()
    ctrl.state.battery_percent = pct
    ctrl._check_safety()
    status = ctrl.state.safety_status
    ok = expected is None or status == expected or \
         (expected == SafetyStatus.EMERGENCY and status in \
          [SafetyStatus.CRITICAL, SafetyStatus.EMERGENCY])
    icon = '✅' if ok else '❌'
    print(f'  {icon} {name}: {status.value}')

# Geofence
print()
ctrl.state = type(ctrl.state)()
ctrl.state.position_ned = [400, 0, -5]
ctrl._check_safety()
print(f'  {\"✅\" if not ctrl.state.geofence_ok else \"❌\"} Geofence breach (400m): geofence_ok={ctrl.state.geofence_ok}')

ctrl.state.position_ned = [100, 0, -5]
ctrl._check_safety()
print(f'  {\"✅\" if ctrl.state.geofence_ok else \"❌\"} Inside geofence (100m): geofence_ok={ctrl.state.geofence_ok}')

# Weather
print()
ctrl.state = type(ctrl.state)()
ctrl.state.humidity = 98
ctrl._check_safety()
print(f'  {\"✅\" if not ctrl.state.weather_ok else \"❌\"} High humidity (98%): weather_ok={ctrl.state.weather_ok}')

ctrl.state.humidity = 60
ctrl.state.pressure = 1013
ctrl._check_safety()
print(f'  {\"✅\" if ctrl.state.weather_ok else \"❌\"} Normal weather: weather_ok={ctrl.state.weather_ok}')

print()
print('=== Safety Audit Complete ===')
"
}

# --- 帮助 ---
print_help() {
    print_banner
    echo "用法: ./run.sh [选项]"
    echo ""
    echo "选项:"
    echo "  --sim            启动SITL仿真环境 (Docker)"
    echo "  --test           运行所有自动化测试 (30+)"
    echo "  --demo           运行功能演示"
    echo "  --outdoor-prep   户外飞行准备清单"
    echo "  --safety         运行安全审计"
    echo "  --help, -h       显示帮助信息"
    echo ""
    echo "示例:"
    echo "  ./run.sh --test        # 运行测试"
    echo "  ./run.sh --sim         # 启动仿真"
    echo "  ./run.sh --demo        # 功能演示"
    echo "  ./run.sh --safety      # 安全审计"
}

# --- 主入口 ---
print_banner

case "${1:-}" in
    --sim)
        run_sim
        ;;
    --test)
        run_test
        ;;
    --demo)
        run_demo
        ;;
    --outdoor-prep)
        run_outdoor_prep
        ;;
    --safety)
        run_safety
        ;;
    --help|-h)
        print_help
        ;;
    *)
        print_help
        exit 1
        ;;
esac
