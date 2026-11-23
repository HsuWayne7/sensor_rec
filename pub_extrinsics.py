#!/usr/bin/env python3
"""讀 extrinsics.yaml (各 frame 在 base 下的位姿),發到 /tf_static。
attach frame (已由其他節點帶著跑的 frame) 反向發成 attach -> base,其他發 base -> frame。
  pub_extrinsics.py extrinsics.yaml          發布並保持
  pub_extrinsics.py extrinsics.yaml --check  只印出,有沒填的回傳 2"""
import math, sys, yaml
import numpy as np


def rpy_to_R(r, p, y):
    cr, sr, cp, sp, cy, sy = math.cos(r), math.sin(r), math.cos(p), math.sin(p), math.cos(y), math.sin(y)
    return np.array([[cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
                     [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
                     [-sp, cp * sr, cp * cr]])


def R_to_quat(R):  # (x, y, z, w)
    w = math.sqrt(max(0.0, 1 + R[0, 0] + R[1, 1] + R[2, 2])) / 2
    x = math.copysign(math.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2])) / 2, R[2, 1] - R[1, 2])
    y = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2])) / 2, R[0, 2] - R[2, 0])
    z = math.copysign(math.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2])) / 2, R[1, 0] - R[0, 1])
    return x, y, z, w


def R_to_rpy_deg(R):
    return [round(math.degrees(a), 3) for a in
            (math.atan2(R[2, 1], R[2, 2]), math.asin(max(-1, min(1, -R[2, 0]))), math.atan2(R[1, 0], R[0, 0]))]


def load(path):
    cfg = yaml.safe_load(open(path))
    base, attach = cfg['base'], cfg.get('attach')
    out, missing = [], []
    for name, v in (cfg.get('frames') or {}).items():
        xyz, rpy = v.get('xyz'), v.get('rpy_deg')
        if xyz is None or rpy is None or None in list(xyz) + list(rpy):
            missing.append(name); continue
        T = np.eye(4)
        T[:3, :3] = rpy_to_R(*[math.radians(float(a)) for a in rpy])
        T[:3, 3] = [float(a) for a in xyz]
        if name == attach:
            out.append((name, base, np.linalg.inv(T)))
        else:
            out.append((base, name, T))
    if attach and attach in missing:
        raise SystemExit(f'[ERR] attach frame {attach} 沒填,TF 樹會斷開')
    return out, missing


def main():
    path = sys.argv[1]
    tfs, missing = load(path)
    if '--check' in sys.argv:
        for p, c, T in tfs:
            print(f'      {p} -> {c}: xyz={np.round(T[:3, 3], 4).tolist()} rpy_deg={R_to_rpy_deg(T[:3, :3])}')
        for c in missing:
            print(f'      {c}: 還沒填 (不發布)')
        sys.exit(2 if missing else 0)

    import rclpy
    from rclpy.node import Node
    from geometry_msgs.msg import TransformStamped
    from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
    rclpy.init()
    node = Node('rig_extrinsics')
    msgs = []
    for p, c, T in tfs:
        m = TransformStamped()
        m.header.stamp = node.get_clock().now().to_msg()
        m.header.frame_id, m.child_frame_id = p, c
        m.transform.translation.x, m.transform.translation.y, m.transform.translation.z = T[:3, 3].tolist()
        q = m.transform.rotation
        q.x, q.y, q.z, q.w = R_to_quat(T[:3, :3])
        msgs.append(m)
    br = StaticTransformBroadcaster(node)
    if msgs:
        br.sendTransform(msgs)
    node.get_logger().info(f'published {len(msgs)} static TF from {path}' +
                           (f' (未填: {", ".join(missing)})' if missing else ''))
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
