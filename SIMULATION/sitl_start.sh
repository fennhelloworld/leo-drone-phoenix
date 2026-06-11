#!/bin/bash
# LeoDrone Phoenix — PX4 SITL + Gazebo 仿真启动脚本
# 用法: ./sitl_start.sh [--docker|--local]

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_DIR="$(dirname "$SCRIPT_DIR")"

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

echo -e "${BLUE}🔥 LeoDrone Phoenix — SITL Simulation${NC}"
echo "=========================================="

# 检查Docker
check_docker() {
    if ! command -v docker &> /dev/null; then
        echo -e "${RED}Docker not found! Install: https://docs.docker.com/get-docker/${NC}"
        exit 1
    fi
    echo -e "${GREEN}✅ Docker found${NC}"
}

# 检查X11 (用于Gazebo GUI)
check_x11() {
    if [ -z "$DISPLAY" ]; then
        echo -e "${YELLOW}⚠️  No DISPLAY set, running headless (no Gazebo GUI)${NC}"
        HEADLESS=1
    else
        echo -e "${GREEN}✅ X11 display available${NC}"
        HEADLESS=0
    fi
}

# 启动PX4 SITL Docker容器
start_docker_sitl() {
    echo -e "${BLUE}📦 Starting PX4 SITL in Docker...${NC}"

    # 检查是否已有容器运行
    if docker ps -a --format '{{.Names}}' | grep -q "phoenix-sitl"; then
        echo -e "${YELLOW}Removing existing phoenix-sitl container...${NC}"
        docker rm -f phoenix-sitl 2>/dev/null || true
    fi

    # X11授权 (用于GUI)
    if [ "$HEADLESS" = "0" ]; then
        xhost +local:docker 2>/dev/null || true
    fi

    # 运行容器
    local DISPLAY_OPT=""
    if [ "$HEADLESS" = "0" ]; then
        DISPLAY_OPT="-e DISPLAY=$DISPLAY -v /tmp/.X11-unix:/tmp/.X11-unix"
    fi

    docker run -d \
        --name phoenix-sitl \
        -p 14540:14540 \
        -p 14550:14550 \
        -p 5760:5760 \
        -p 8080:8080 \
        $DISPLAY_OPT \
        --privileged \
        px4io/px4-dev-ros2-gazebo:latest \
        /bin/bash -c "
            cd /home/user/PX4-Autopilot && \
            source Tools/simulation/gz/bridge.sh && \
            make px4_sitl gz_x500
        "

    echo -e "${GREEN}✅ Docker container started${NC}"
    echo ""
    echo -e "${BLUE}Connection endpoints:${NC}"
    echo "  MAVSDK (UDP):  udp://:14540"
    echo "  QGC (UDP):     udp://:14550"
    echo "  MAVLink (TCP): tcp://:5760"
    echo "  Gazebo Web:    http://localhost:8080"
    echo ""
    echo -e "${YELLOW}Wait ~30s for SITL to start, then connect with:${NC}"
    echo "  python3 $PROJECT_DIR/SIMULATION/sim_test_flight.py"
}

# 本地启动PX4 SITL
start_local_sitl() {
    echo -e "${BLUE}💻 Starting PX4 SITL locally...${NC}"

    # 检查PX4是否已安装
    if [ ! -d "$HOME/PX4-Autopilot" ]; then
        echo -e "${YELLOW}PX4 not found, cloning...${NC}"
        git clone https://github.com/PX4/PX4-Autopilot.git "$HOME/PX4-Autopilot" --recursive
        cd "$HOME/PX4-Autopilot"
        make submodulesclean
    else
        cd "$HOME/PX4-Autopilot"
    fi

    # 启动SITL
    echo -e "${BLUE}Starting SITL with Gazebo...${NC}"
    make px4_sitl gz_x500 &

    SITL_PID=$!
    echo -e "${GREEN}✅ SITL started (PID: $SITL_PID)${NC}"

    # 等待启动
    echo -e "${YELLOW}Waiting for SITL to initialize...${NC}"
    sleep 15

    # 验证连接
    if curl -s http://localhost:8080 > /dev/null 2>&1; then
        echo -e "${GREEN}✅ Gazebo web interface available${NC}"
    else
        echo -e "${YELLOW}⚠️  Gazebo web not yet available (may need more time)${NC}"
    fi
}

# 停止仿真
stop_sitl() {
    echo -e "${BLUE}🛑 Stopping SITL simulation...${NC}"

    # 停止Docker容器
    if docker ps -a --format '{{.Names}}' | grep -q "phoenix-sitl"; then
        docker rm -f phoenix-sitl 2>/dev/null
        echo -e "${GREEN}✅ Docker container stopped${NC}"
    fi

    # 停止本地进程
    pkill -f "px4_sitl" 2>/dev/null || true
    pkill -f "gz sim" 2>/dev/null || true
    echo -e "${GREEN}✅ Local SITL processes stopped${NC}"
}

# 显示状态
show_status() {
    echo -e "${BLUE}📊 SITL Status:${NC}"

    # Docker状态
    if docker ps --format '{{.Names}}' | grep -q "phoenix-sitl"; then
        echo -e "  Docker: ${GREEN}running${NC}"
    else
        echo -e "  Docker: ${RED}not running${NC}"
    fi

    # 端口检查
    for port in 14540 14550 5760 8080; do
        if ss -tlnp 2>/dev/null | grep -q ":$port " || \
           netstat -tlnp 2>/dev/null | grep -q ":$port "; then
            echo -e "  Port $port: ${GREEN}open${NC}"
        else
            echo -e "  Port $port: ${YELLOW}closed${NC}"
        fi
    done
}

# 主入口
MODE="${1:-docker}"

case "$MODE" in
    --docker|-d)
        check_docker
        check_x11
        start_docker_sitl
        ;;
    --local|-l)
        start_local_sitl
        ;;
    --stop|-s)
        stop_sitl
        ;;
    --status|-t)
        show_status
        ;;
    --help|-h)
        echo "Usage: $0 [--docker|--local|--stop|--status|--help]"
        echo ""
        echo "Options:"
        echo "  --docker, -d   Start SITL in Docker (default)"
        echo "  --local,  -l   Start SITL locally (requires PX4 installed)"
        echo "  --stop,   -s   Stop SITL simulation"
        echo "  --status, -t   Show SITL status"
        echo "  --help,   -h   Show this help"
        ;;
    *)
        check_docker
        check_x11
        start_docker_sitl
        ;;
esac
