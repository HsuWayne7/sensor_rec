#!/usr/bin/env python3
"""等所有 topic 真的有資料進來,量頻率;--expo 模式持續印 ZED 影像亮度,用來調曝光/增益/白平衡。

用法:
  sensor_check.py --timeout 90 /topic1 /topic2 ...   # 全部到齊 exit 0,否則 exit 1
  sensor_check.py --expo                             # 持續印亮度 (Ctrl+C 離開)
"""
import argparse
import io
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rosidl_runtime_py.utilities import get_message

IMAGE_TOPIC = '/zed/zed_node/rgb/color/rect/image/compressed'
# 低於這個頻率就警告 (Hz)
MIN_RATE = {
    '/livox/lidar': 8, '/livox/imu': 100,
    '/t265/odom': 100, '/t265/imu/gyro': 100, '/t265/imu/accel': 30,
    IMAGE_TOPIC: 10, '/zed/zed_node/rgb/color/rect/camera_info': 10,
    '/zed/zed_node/imu/data': 50, '/zed/zed_node/odom': 10, '/zed/zed_node/pose': 10,
}
ONE_SHOT = {'/tf_static'}


def image_stats(msg):
    """CompressedImage -> (平均亮度, 過曝%, 過暗%, R/G/B 平均)"""
    from PIL import Image, ImageStat
    im = Image.open(io.BytesIO(bytes(msg.data))).convert('RGB')
    rgb = ImageStat.Stat(im).mean
    hist = im.convert('L').histogram()
    n = sum(hist)
    mean = sum(i * c for i, c in enumerate(hist)) / n
    return mean, 100 * sum(hist[250:]) / n, 100 * sum(hist[:15]) / n, rgb


def expo_verdict(mean, clip):
    if clip > 2 and mean < 110:
        return '高對比 (畫面有燈/窗) -> 燈本身過曝可接受,以 mean 90~120 為準'
    if clip > 2 or mean > 160:
        return '過亮 -> 先降 gain,gain 到 0 還亮再降 exposure'
    if mean < 80:
        return '過暗 -> 先升 gain (到 ~50),還暗再升 exposure (上限 ~40)'
    return 'OK'


class Checker(Node):
    def __init__(self, topics):
        super().__init__('sensor_check')
        self.topics = topics
        self.subs = {}
        self.stamps = {t: [] for t in topics}
        self.last_img = None

    def try_subscribe(self):
        types = dict(self.get_topic_names_and_types())
        for t in self.topics:
            if t in self.subs or t not in types:
                continue
            if t in ONE_SHOT:
                qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.RELIABLE,
                                 durability=DurabilityPolicy.TRANSIENT_LOCAL)
            else:
                qos = QoSProfile(depth=50, history=HistoryPolicy.KEEP_LAST,
                                 reliability=ReliabilityPolicy.BEST_EFFORT)
            cls = get_message(types[t][0])
            raw = t != IMAGE_TOPIC
            self.subs[t] = self.create_subscription(
                cls, t, lambda m, t=t: self.on_msg(t, m), qos, raw=raw)

    def on_msg(self, topic, msg):
        self.stamps[topic].append(time.monotonic())
        if topic == IMAGE_TOPIC:
            self.last_img = msg

    def rate(self, topic, window):
        now = time.monotonic()
        return sum(1 for s in self.stamps[topic] if now - s <= window) / window


def run_check(node, timeout, window):
    t0 = time.monotonic()
    while time.monotonic() - t0 < timeout:
        node.try_subscribe()
        rclpy.spin_once(node, timeout_sec=0.1)
        if all(node.stamps[t] for t in node.topics):
            break
    missing = [t for t in node.topics if not node.stamps[t]]
    if missing:
        print(f'\n[FAIL] {timeout:.0f}s 內沒收到:')
        for t in missing:
            print('   ', t)
        return 1

    # 全部到齊後再量 window 秒的頻率
    t1 = time.monotonic()
    while time.monotonic() - t1 < window:
        rclpy.spin_once(node, timeout_sec=0.1)
    ok = True
    print(f'\n{"topic":50s} {"Hz":>8s}')
    for t in node.topics:
        if t in ONE_SHOT:
            print(f'{t:50s} {"latched":>8s}  ok')
            continue
        hz = node.rate(t, window)
        bad = hz < MIN_RATE.get(t, 1)
        ok &= not bad
        print(f'{t:50s} {hz:8.1f}  {"LOW!" if bad else "ok"}')
    if node.last_img is not None:
        mean, clip, dark, rgb = image_stats(node.last_img)
        print(f'\nZED 亮度 mean={mean:.0f} 過曝={clip:.1f}% 過暗={dark:.1f}% '
              f'R/G/B={rgb[0]:.0f}/{rgb[1]:.0f}/{rgb[2]:.0f}  -> {expo_verdict(mean, clip)}')
    return 0 if ok else 2


def run_expo(node):
    print('目標: mean 100~140, 過曝 < 1%, R≈G≈B (白牆/白紙)。另開終端機調:')
    print('  ~/sensor_rec/zed_set.sh 25 10      (exposure gain)')
    print('  ~/sensor_rec/zed_set.sh 25 10 42   (exposure gain 白平衡)')
    print('  滿意後: ~/sensor_rec/zed_set.sh --save 25 10 42\n')
    last = 0
    while True:
        node.try_subscribe()
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.last_img is not None and time.monotonic() - last > 1.0:
            last = time.monotonic()
            mean, clip, dark, rgb = image_stats(node.last_img)
            print(f'mean={mean:5.1f}  過曝={clip:5.2f}%  過暗={dark:5.2f}%  '
                  f'R/G/B={rgb[0]:.0f}/{rgb[1]:.0f}/{rgb[2]:.0f}  {expo_verdict(mean, clip)}',
                  flush=True)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('topics', nargs='*')
    ap.add_argument('--timeout', type=float, default=90)
    ap.add_argument('--window', type=float, default=3)
    ap.add_argument('--expo', action='store_true')
    a = ap.parse_args()

    rclpy.init()
    node = Checker([IMAGE_TOPIC] if a.expo else a.topics)
    try:
        rc = run_expo(node) if a.expo else run_check(node, a.timeout, a.window)
    except KeyboardInterrupt:
        rc = 0 if a.expo else 130   # 檢查途中 Ctrl+C = 取消,不能當成通過
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(rc)


if __name__ == '__main__':
    main()
