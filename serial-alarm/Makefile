# 顶层 Makefile — 全部在 ARM 服务器本机执行 (手动部署工程后使用)
#
# 用法:
#   make            # 编译内核模块 + 用户态程序
#   make test       # 编译 + 加载模块 + 突发测试 + 报告
#   make clean      # 清理两个子目录的构建产物
#
# 测试规模:  make test TEST_COUNT=100 TEST_INTERVAL=100000

TEST_COUNT     ?= 1000
TEST_INTERVAL  ?= 1000000

.PHONY: all build test clean

all: build

build:
	$(MAKE) -C kmod
	$(MAKE) -C user

test: build
	COUNT=$(TEST_COUNT) INTERVAL=$(TEST_INTERVAL) bash tools/run_test.sh

clean:
	$(MAKE) -C kmod clean
	$(MAKE) -C user clean
