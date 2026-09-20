#!/usr/bin/env python3
"""stats.py — alarmd 延迟 CSV 统计与直方图 (方案 7.1 通道二后处理)

用法: python3 stats.py alarm_latency.csv [--hist 20]
"""
import argparse
import csv
import sys


def load(path):
    lats = []
    with open(path, newline="") as f:
        for row in csv.DictReader(f):
            lats.append(int(row["latency_ns"]))
    return lats


def percentile(sorted_lats, p):
    return sorted_lats[min(len(sorted_lats) - 1, len(sorted_lats) * p // 100)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("csv")
    ap.add_argument("--hist", type=int, default=0, help="直方图桶数")
    args = ap.parse_args()

    lats = load(args.csv)
    if not lats:
        sys.exit("no samples")
    s = sorted(lats)
    n = len(s)
    avg = sum(s) / n
    print(f"samples : {n}")
    print(f"avg     : {avg:12.1f} ns")
    print(f"p50     : {percentile(s, 50):12d} ns")
    print(f"p99     : {percentile(s, 99):12d} ns")
    print(f"max     : {s[-1]:12d} ns")
    print(f"min     : {s[0]:12d} ns")

    if args.hist:
        lo, hi = s[0], s[-1]
        width = max(1, (hi - lo + args.hist) // args.hist)
        buckets = [0] * (args.hist + 1)
        for v in s:
            buckets[min(args.hist, (v - lo) // width)] += 1
        top = max(buckets)
        for i, c in enumerate(buckets):
            bar = "#" * (60 * c // max(1, top))
            print(f"{lo + i * width:>10} ns | {bar} {c}")


if __name__ == "__main__":
    main()
