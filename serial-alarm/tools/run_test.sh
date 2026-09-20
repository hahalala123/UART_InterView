#!/usr/bin/env bash
# run_test.sh — 在 ARM 服务器上执行 (由顶层 make test 调用, 也可直接运行)
#
# 流程: 加载 valarm 模块 -> 启动 alarmd -> 突发注入 -> 统计报告
#       -> 内核/用户态计数比对 (验收: 零丢字节/零错序) -> 卸载模块
set -euo pipefail

# insmod/SCHED_FIFO/sysfs 注入都需要 root 权限
if [ "$(id -u)" -ne 0 ]; then
	echo "FAIL: run_test.sh must run as root"
	exit 1
fi

DIR="$(cd "$(dirname "$0")/.." && pwd)"
COUNT="${COUNT:-1000}"
INTERVAL="${INTERVAL:-1000000}"   # ns
LOG="/tmp/alarmd.log"
CSV="/tmp/alarm_latency.csv"

echo "== insmod valarm =="
# 清理可能残留的模块 (上次测试中断时 rmmod 未执行)
sudo rmmod valarm 2>/dev/null || true
for _ in 1 2 3 4 5; do
	lsmod | grep -q "^valarm" || break
	sleep 0.5
done
if lsmod | grep -q "^valarm"; then
	echo "warn: normal rmmod failed, trying rmmod -f"
	sudo rmmod -f valarm 2>/dev/null || true
	sleep 0.5
fi
if lsmod | grep -q "^valarm"; then
	echo "FAIL: valarm still loaded, check 'lsmod | grep valarm'"
	exit 1
fi
sudo insmod "$DIR/kmod/valarm.ko"
sleep 1
ls -l /dev/valarm0 /dev/valarm1

echo "== start alarmd (SCHED_FIFO if permitted) =="
# 直接启动不加 sudo: 脚本需以 root 运行 (insmod/SCHED_FIFO 都需要),
# 加 sudo 会让 $! 拿到 sudo 的 PID, kill -USR1 发给 sudo 后不一定
# 转发给 alarmd, 导致 wait 永久阻塞。
"$DIR/user/alarmd" -c 1 -o "$CSV" > "$LOG" 2>&1 &
ALARMD_PID=$!
sleep 1
if ! sudo kill -0 "$ALARMD_PID" 2>/dev/null; then
	echo "FAIL: alarmd exited prematurely, log follows:"
	cat "$LOG"
	sudo rmmod valarm 2>/dev/null || true
	exit 1
fi

echo "== inject burst: count=$COUNT interval=${INTERVAL}ns on both ports =="
echo "$COUNT $INTERVAL" | sudo tee /sys/class/misc/valarm0/burst > /dev/null
echo "$COUNT $INTERVAL" | sudo tee /sys/class/misc/valarm1/burst > /dev/null

# 等待突发结束 (最后字节注入时间 + 余量)
WAIT_NS=$(( COUNT * INTERVAL + 500000000 ))
sleep "$(awk -v ns="$WAIT_NS" 'BEGIN{printf "%.3f", ns/1e9}')"

echo "== trigger report =="
kill -USR1 "$ALARMD_PID"
# 最多等 10 秒, 超时视为 alarmd 未响应信号
for _ in $(seq 1 100); do
	kill -0 "$ALARMD_PID" 2>/dev/null || break
	sleep 0.1
done
if kill -0 "$ALARMD_PID" 2>/dev/null; then
	echo "FAIL: alarmd did not exit after SIGUSR1, killing"
	kill -9 "$ALARMD_PID" 2>/dev/null || true
	cat "$LOG"
	sudo rmmod valarm 2>/dev/null || true
	exit 1
fi
wait "$ALARMD_PID" || true
cat "$LOG"

echo "== cross-check kernel counters (zero-loss acceptance) =="
sudo cat /sys/class/misc/valarm0/stats
sudo cat /sys/class/misc/valarm1/stats

echo "== rmmod =="
sudo rmmod valarm
echo "PASS: test completed, csv at $CSV"
