#!/usr/bin/env python3
import os
import sys
import threading
import queue

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, CameraInfo, Imu
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster, StaticTransformBroadcaster
from scipy.spatial.transform import Rotation as Rot

# T265 座標 (x右, y上, z後) -> ROS 座標 (x前, y左, z上)
R_CONV = np.array([[0, 0, -1],
                   [-1, 0, 0],
                   [0, 1, 0]], dtype=float)


class T265Node(Node):
    def __init__(self):
        super().__init__('t265_node')
        self.declare_parameter('odom_frame', 't265_odom')
        self.declare_parameter('base_frame', 't265_link')
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('enable_fisheye', True)
        self.declare_parameter('enable_imu', True)
        self.declare_parameter('pyrealsense_path',
                               '~/librealsense/build/wrappers/python')

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.enable_fisheye = self.get_parameter('enable_fisheye').value
        self.enable_imu = self.get_parameter('enable_imu').value

        try:
            import pyrealsense2 as rs
        except ImportError:
            path = os.path.expanduser(self.get_parameter('pyrealsense_path').value)
            sys.path.append(path)
            import pyrealsense2 as rs
        self.rs = rs

        # ---- 先把所有狀態建好,再啟動 pipeline(回呼一啟動就會用到) ----
        self.stop_flag = False
        self.last_fn = {}
        self.counts = {'odom': 0, 'img1': 0, 'img2': 0, 'gyro': 0,
                       'accel': 0, 'drop1': 0, 'drop2': 0}
        self.img_queue = {1: queue.Queue(maxsize=3), 2: queue.Queue(maxsize=3)}
        self.cam_info = {}
        self.fe_frame = {}
        self.img_pub = {}
        self.info_pub = {}

        self.odom_pub = self.create_publisher(Odometry, 't265/odom', 10)
        self.tf_br = TransformBroadcaster(self)
        self.static_br = StaticTransformBroadcaster(self)

        if self.enable_fisheye:
            for i in (1, 2):
                self.img_pub[i] = self.create_publisher(
                    Image, f't265/fisheye{i}/image_raw', qos_profile_sensor_data)
                self.info_pub[i] = self.create_publisher(
                    CameraInfo, f't265/fisheye{i}/camera_info', qos_profile_sensor_data)
                self.fe_frame[i] = f't265_fisheye{i}_optical_frame'
        if self.enable_imu:
            self.gyro_pub = self.create_publisher(Imu, 't265/imu/gyro', qos_profile_sensor_data)
            self.accel_pub = self.create_publisher(Imu, 't265/imu/accel', qos_profile_sensor_data)

        self.workers = []
        if self.enable_fisheye:
            for i in (1, 2):
                th = threading.Thread(target=self.image_worker, args=(i,), daemon=True)
                th.start()
                self.workers.append(th)

        self.create_timer(1.0, self.report_stats)

        # ---- 啟動 pipeline ----
        self.pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.pose)
        if self.enable_fisheye:
            cfg.enable_stream(rs.stream.fisheye, 1)
            cfg.enable_stream(rs.stream.fisheye, 2)
        if self.enable_imu:
            cfg.enable_stream(rs.stream.gyro)
            cfg.enable_stream(rs.stream.accel)

        self.profile = self.pipe.start(cfg, self.on_frame)
        if self.enable_fisheye:
            self.setup_fisheye_info()
        self.get_logger().info(
            f'T265 started (fisheye={self.enable_fisheye}, imu={self.enable_imu})')

    # ---------- 魚眼內參與外參 ----------
    def setup_fisheye_info(self):
        rs = self.rs
        pose_prof = self.profile.get_stream(rs.stream.pose)
        static_msgs = []
        infos = {}
        for i in (1, 2):
            sp = self.profile.get_stream(rs.stream.fisheye, i).as_video_stream_profile()
            intr = sp.get_intrinsics()

            info = CameraInfo()
            info.header.frame_id = self.fe_frame[i]
            info.width = intr.width
            info.height = intr.height
            info.distortion_model = 'equidistant'
            info.d = [float(c) for c in intr.coeffs[:4]]
            info.k = [intr.fx, 0.0, intr.ppx,
                      0.0, intr.fy, intr.ppy,
                      0.0, 0.0, 1.0]
            info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
            info.p = [intr.fx, 0.0, intr.ppx, 0.0,
                      0.0, intr.fy, intr.ppy, 0.0,
                      0.0, 0.0, 1.0, 0.0]
            infos[i] = info

            ext = sp.get_extrinsics_to(pose_prof)
            R = np.array(ext.rotation).reshape(3, 3).T
            t = np.array(ext.translation)
            R_link = R_CONV @ R
            t_link = R_CONV @ t
            q = Rot.from_matrix(R_link).as_quat()

            m = TransformStamped()
            m.header.stamp = self.get_clock().now().to_msg()
            m.header.frame_id = self.base_frame
            m.child_frame_id = self.fe_frame[i]
            m.transform.translation.x = float(t_link[0])
            m.transform.translation.y = float(t_link[1])
            m.transform.translation.z = float(t_link[2])
            m.transform.rotation.x = float(q[0])
            m.transform.rotation.y = float(q[1])
            m.transform.rotation.z = float(q[2])
            m.transform.rotation.w = float(q[3])
            static_msgs.append(m)
        self.static_br.sendTransform(static_msgs)
        self.cam_info = infos  # 最後才賦值,回呼端以此判斷是否就緒

    # ---------- 回呼(librealsense 執行緒) ----------
    def on_frame(self, frame):
        try:
            if frame.is_frameset():
                for f in frame.as_frameset():
                    self.handle_frame(f)
            else:
                self.handle_frame(frame)
        except Exception as e:
            self.get_logger().error(f'frame error: {e}')

    def handle_frame(self, f):
        if f.is_pose_frame():
            self.on_pose(f.as_pose_frame())
        elif f.is_motion_frame():
            if self.enable_imu:
                self.on_motion(f.as_motion_frame())
        elif f.is_video_frame():
            if self.enable_fisheye:
                self.on_image(f.as_video_frame())

    def on_pose(self, pf):
        d = pf.get_pose_data()
        p = R_CONV @ np.array([d.translation.x, d.translation.y, d.translation.z])
        v = R_CONV @ np.array([d.velocity.x, d.velocity.y, d.velocity.z])
        w = R_CONV @ np.array([d.angular_velocity.x, d.angular_velocity.y,
                               d.angular_velocity.z])
        Rm = Rot.from_quat([d.rotation.x, d.rotation.y,
                            d.rotation.z, d.rotation.w]).as_matrix()
        q = Rot.from_matrix(R_CONV @ Rm @ R_CONV.T).as_quat()

        now = self.get_clock().now().to_msg()
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        pp = odom.pose.pose
        pp.position.x, pp.position.y, pp.position.z = map(float, p)
        pp.orientation.x, pp.orientation.y, pp.orientation.z, pp.orientation.w = map(float, q)
        tw = odom.twist.twist
        tw.linear.x, tw.linear.y, tw.linear.z = map(float, v)
        tw.angular.x, tw.angular.y, tw.angular.z = map(float, w)
        cov = {0: 1e3, 1: 1e-1, 2: 1e-2, 3: 1e-3}[d.tracker_confidence]
        for i in range(6):
            odom.pose.covariance[i * 7] = cov
            odom.twist.covariance[i * 7] = cov
        self.odom_pub.publish(odom)
        self.counts['odom'] += 1

        if self.publish_tf:
            t = TransformStamped()
            t.header = odom.header
            t.child_frame_id = self.base_frame
            t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = map(float, p)
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_br.sendTransform(t)

    def on_image(self, vf):
        idx = vf.get_profile().stream_index()
        if idx not in self.cam_info:
            return
        fn = vf.get_frame_number()
        if self.last_fn.get(idx) == fn:
            return  # 同一張影像重複送達,略過
        self.last_fn[idx] = fn
        stamp = self.get_clock().now().to_msg()
        data = np.array(vf.get_data(), copy=True)  # 複製,避免 frame 被回收
        try:
            self.img_queue[idx].put_nowait((stamp, vf.get_width(), vf.get_height(), data))
        except queue.Full:
            self.counts[f'drop{idx}'] += 1

    def image_worker(self, idx):
        while not self.stop_flag:
            try:
                stamp, w, h, data = self.img_queue[idx].get(timeout=0.5)
            except queue.Empty:
                continue
            try:
                img = Image()
                img.header.stamp = stamp
                img.header.frame_id = self.fe_frame[idx]
                img.height = h
                img.width = w
                img.encoding = 'mono8'
                img.is_bigendian = 0
                img.step = w
                img.data = data.tobytes()
                self.img_pub[idx].publish(img)

                info = self.cam_info[idx]
                info.header.stamp = stamp
                self.info_pub[idx].publish(info)
                self.counts[f'img{idx}'] += 1
            except Exception as e:
                self.get_logger().error(f'image publish error: {e}')

    def on_motion(self, mf):
        rs = self.rs
        st = mf.get_profile().stream_type()
        data = mf.get_motion_data()
        vec = R_CONV @ np.array([data.x, data.y, data.z])

        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.base_frame
        m.orientation_covariance[0] = -1.0
        if st == rs.stream.gyro:
            m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = map(float, vec)
            m.linear_acceleration_covariance[0] = -1.0
            self.gyro_pub.publish(m)
            self.counts['gyro'] += 1
        elif st == rs.stream.accel:
            m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = map(float, vec)
            m.angular_velocity_covariance[0] = -1.0
            self.accel_pub.publish(m)
            self.counts['accel'] += 1

    def report_stats(self):
        c = self.counts
        self.get_logger().info(
            f"/s odom={c['odom']} img1={c['img1']} img2={c['img2']} "
            f"gyro={c['gyro']} accel={c['accel']} drop1={c['drop1']} drop2={c['drop2']}")
        for k in c:
            c[k] = 0

    def destroy_node(self):
        self.stop_flag = True
        try:
            self.pipe.stop()
        except Exception:
            pass
        super().destroy_node()


def main():
    rclpy.init()
    node = T265Node()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
