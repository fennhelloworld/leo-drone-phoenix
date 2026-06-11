# LeoDrone Phoenix — Makefile
# 用法: make <target>

.PHONY: sim-start sim-test sim-all flash outdoor-check test all clean help

# 项目路径
PROJECT_DIR := $(shell pwd)
FIRMWARE_DIR := $(PROJECT_DIR)/FIRMWARE
SIMULATION_DIR := $(PROJECT_DIR)/SIMULATION
TEST_DIR := $(PROJECT_DIR)/tests

# Python
PYTHON := /usr/bin/python3
PIP := $(PYTHON) -m pip

# Docker
DOCKER := docker

# 颜色
BLUE := \033[0;34m
GREEN := \033[0;32m
YELLOW := \033[1;33m
RED := \033[0;31m
NC := \033[0m

# 默认目标
.DEFAULT_GOAL := help

##@ 仿真

sim-start: ## 启动PX4 SITL仿真环境 (Docker)
	@echo "$(BLUE)🔥 启动SITL仿真...$(NC)"
	@chmod +x $(SIMULATION_DIR)/sitl_start.sh
	@$(SIMULATION_DIR)/sitl_start.sh --docker

sim-test: ## 运行仿真测试 (飞行/拼接/SLAM/追踪)
	@echo "$(BLUE)🧪 运行仿真测试...$(NC)"
	@$(PYTHON) $(SIMULATION_DIR)/sim_test_flight.py
	@$(PYTHON) $(SIMULATION_DIR)/sim_360_stitch.py
	@$(PYTHON) $(SIMULATION_DIR)/sim_slam.py
	@$(PYTHON) $(SIMULATION_DIR)/sim_tracking.py

sim-all: ## 启动仿真 + 运行所有测试
	@echo "$(BLUE)🔥 完整仿真测试...$(NC)"
	@$(MAKE) sim-start
	@sleep 30
	@$(MAKE) sim-test

##@ 固件

flash: ## 刷写固件到Pixhawk (需要连接USB)
	@echo "$(YELLOW)⚠️  刷写固件...$(NC)"
	@if ! command -v qgc &> /dev/null; then \
		echo "$(RED)QGroundControl未安装$(NC)"; \
		exit 1; \
	fi
	@echo "$(GREEN)请使用QGroundControl刷写PX4固件$(NC)"

##@ 户外

outdoor-check: ## 户外飞行前检查
	@echo "$(BLUE)📋 户外飞行前检查清单$(NC)"
	@echo ""
	@echo "=== 飞行前检查 ==="
	@echo "1. 天气条件良好 (风速<5级, 无雨)"
	@echo "2. 飞行区域合法 (非禁飞区)"
	@echo "3. 电池充满 (≥4.1V/cell)"
	@echo "4. 螺旋桨紧固，无裂纹"
	@echo "5. 电机旋转方向正确"
	@echo "6. 遥控信号正常"
	@echo "7. GPS锁定 (≥8卫星)"
	@echo "8. 罗盘校准有效"
	@echo "9. 数传连接正常"
	@echo "10. 伴飞电脑运行正常"
	@echo "11. 相机画面正常"
	@echo "12. Failsafe参数设置正确"
	@echo "13. 地理围栏配置正确"
	@echo "14. 附近无人员/障碍物"
	@echo "15. 紧急降落点已确认"
	@echo ""
	@echo "$(GREEN)详见 SAFETY.md$(NC)"

##@ 测试

test: ## 运行所有自动化测试 (30+测试)
	@echo "$(BLUE)🧪 运行自动化测试...$(NC)"
	@$(PYTHON) -m pytest $(TEST_DIR)/test_phoenix.py -v || \
		$(PYTHON) $(TEST_DIR)/test_phoenix.py

##@ 构建

all: test ## 运行所有测试 (等同 make test)
	@echo "$(GREEN)✅ 所有测试通过$(NC)"

##@ 清理

clean: ## 清理生成文件
	@echo "$(BLUE)🧹 清理...$(NC)"
	@find . -type d -name __pycache__ -exec rm -rf {} + 2>/dev/null || true
	@find . -type f -name "*.pyc" -delete 2>/dev/null || true
	@find . -type f -name "*.pyo" -delete 2>/dev/null || true
	@find . -type d -name ".pytest_cache" -exec rm -rf {} + 2>/dev/null || true
	@find . -type d -name "*.egg-info" -exec rm -rf {} + 2>/dev/null || true
	@echo "$(GREEN)✅ 清理完成$(NC)"

##@ 帮助

help: ## 显示帮助信息
	@echo ""
	@echo "$(BLUE)LeoDrone Phoenix — 360°全景AI穿越无人机$(NC)"
	@echo ""
	@echo "$(YELLOW)用法:$(NC) make [target]"
	@echo ""
	@echo "$(YELLOW)仿真目标:$(NC)"
	@grep -E '^[a-zA-Z_-]+:.*?## .*$$' $(MAKEFILE_LIST) | \
		awk 'BEGIN {FS = ":.*?## "}; {printf "  $(GREEN)%-18s$(NC) %s\n", $$1, $$2}' | \
		sort
	@echo ""
	@echo "$(YELLOW)示例:$(NC)"
	@echo "  make sim-start    # 启动仿真环境"
	@echo "  make sim-test     # 运行仿真测试"
	@echo "  make test         # 运行自动化测试"
	@echo "  make outdoor-check  # 户外飞行检查"
	@echo ""
