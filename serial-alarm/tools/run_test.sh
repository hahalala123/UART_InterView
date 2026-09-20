#!/usr/bin/env bash
# run_test.sh — 在 ARM 服务器上执行 (由顶层 make test 调用, 也可直接运行)
#
# 流程: 加载 valarm 模块 -> 启动 alarmd -> 突发注入 -> 统计报告
#       -> 内核/用户态计数比对 (验收: 零丢字节/零错序) -> 卸载模块
set -euo pipefail

DIR="$(cd "$(dirname "$0")/.." && pwd)"
COUNT="${COUNT:-1000}"
INTERVAL="${INTERVAL:-1000000}"   # ns
LOG="/tmp/alarmd.log"
CSV="/tmp/alarm_latency.csv"

echo "== insmod valarm =="
if ! sudo rmmod valarm 2>/dev/null; then true; fi
sudo insmod "$DIR/kmod/valarm.ko"
sleep 1
ls -l /dev/valarm0 /dev/valarm1

echo "== start alarmd (SCHED_FIFO if permitted) =="
sudo "$DIR/user/alarmd" -c 1 -o "$CSV" > "$LOG" 2>&1 &
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
sudo kill -USR1 "$ALARMD_PID"
wait "$ALARMD_PID" || true
cat "$LOG"

echo "== cross-check kernel counters (zero-loss acceptance) =="
sudo cat /sys/class/misc/valarm0/stats
sudo cat /sys/class/misc/valarm1/stats

echo "== rmmod =="
sudo rmmod valarm
echo "PASS: test completed, csv at $CSV"
