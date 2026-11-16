#!/usr/bin/env python3
"""
ROS 2 + Webots Bridge Node (Menggabungkan Layer 0, Layer 1, dan Layer 2)
Arsitektur Sensor: 100% Pure Intel RealSense D455 (RGB-D + Internal IMU) + 3 Wheel Encoders.
Tanpa LiDAR.

Dilengkapi Proteksi Anti-Slip & Blind-Zone D455:
1. Depth Bumper D455: Mencegah motor roda terus melaju maju menabrak tembok
   ketika jarak objek di depan D455 <= 0.28 m.
2. Visual Stall Rejection: Jika kamera D455 mendeteksi pemandangan diam total
   (`is_scene_stationary=True`) saat roda berputar, Layer 1 otomatis menolak
   translasi selip roda agar pose di RViz2 tidak berjalan menembus tembok.
"""

import os
import sys
import math
import ctypes

if 'WEBOTS_HOME' not in os.environ:
    os.environ['WEBOTS_HOME'] = '/usr/local/webots'
sys.path.append(os.path.join(os.environ['WEBOTS_HOME'], 'lib', 'controller', 'python'))

from controller import Robot
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped, Quaternion, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu, Image, LaserScan
import tf2_ros

sys.path.append(os.path.dirname(os.path.abspath(__file__)))
from layer1_odometry import OmniImuOdometry
from layer2_d455_scan import D455DepthToScanConverter


def euler_to_quaternion(yaw: float, pitch: float = 0.0, roll: float = 0.0) -> Quaternion:
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    q = Quaternion()
    q.w = cr * cp * cy + sr * sp * sy
    q.x = sr * cp * cy - cr * sp * sy
    q.y = cr * sp * cy + sr * cp * sy
    q.z = cr * cp * sy - sr * sp * cy
    return q


class BroneD455DriverNode(Node):
    def __init__(self, robot: Robot):
        super().__init__('brone_d455_driver')
        self.robot = robot
        self.timestep = int(self.robot.getBasicTimeStep())
        self.dt = self.timestep / 1000.0
        self.MAX_WHEEL_SPEED = 10.0  # rad/s

        # 1. Inisialisasi Hardware Layer 0 (Webots)
        self.motors = []
        self.encoders = []
        for i in range(1, 4):
            motor = self.robot.getDevice(f'wheel{i}')
            motor.setPosition(float('inf'))
            motor.setVelocity(0.0)
            self.motors.append(motor)

            enc = self.robot.getDevice(f'wheel{i}_sensor')
            enc.enable(self.timestep)
            self.encoders.append(enc)

        self.imu = self.robot.getDevice('imu')
        self.imu.enable(self.timestep)
        self.gyro = self.robot.getDevice('gyro')
        self.gyro.enable(self.timestep)
        self.accel = self.robot.getDevice('accelerometer')
        self.accel.enable(self.timestep)

        self.d455_color = self.robot.getDevice('d455_color')
        self.d455_color.enable(32)
        self.d455_depth = self.robot.getDevice('d455_depth')
        self.d455_depth.enable(32)

        self.gps = self.robot.getDevice('gps')
        self.gps.enable(self.timestep)

        # 2. Inisialisasi Modul Layer 1 & Layer 2
        self.odometry_model = OmniImuOdometry(wheel_radius=0.05, body_radius=0.12)
        self.scan_converter = D455DepthToScanConverter(
            width=self.d455_depth.getWidth(),
            height=self.d455_depth.getHeight(),
            hfov_rad=self.d455_depth.getFov(),
            range_min=0.15,
            range_max=self.d455_depth.getMaxRange(),
            num_beams=160
        )

        # State proteksi Depth Bumper & Visual Stall
        self.front_min_dist = float('inf')
        self.is_scene_stationary = False
        self.cmd_vx = 0.0
        self.cmd_vy = 0.0
        self.cmd_wz = 0.0

        # Anchor awal untuk Ground Truth Pose (agar berhimpit di (0,0) pada RViz2)
        self.init_gt_xy = None
        self.init_gt_yaw = None

        # 3. ROS 2 Publishers, Subscribers & TF Broadcasters
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)

        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)
        self.scan_pub = self.create_publisher(LaserScan, '/scan', 10)
        self.imu_pub = self.create_publisher(Imu, '/imu/data', 10)
        self.rgb_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        self.gt_pub = self.create_publisher(PoseStamped, '/ground_truth/pose', 10)

        self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
        self._publish_static_tf()

        self.step_count = 0
        self.get_logger().info('=============================================================')
        self.get_logger().info('  BRONE D455 SLAM DRIVER READY (Multi-Room Apartment World)  ')
        self.get_logger().info('  - Anti-Slip Depth Bumper & Visual Stall Guard: AKTIF       ')
        self.get_logger().info('=============================================================')

    def _publish_static_tf(self):
        t_cam = TransformStamped()
        t_cam.header.stamp = self.get_clock().now().to_msg()
        t_cam.header.frame_id = 'base_link'
        t_cam.child_frame_id = 'camera_link'
        t_cam.transform.translation.x = 0.08
        t_cam.transform.translation.y = 0.0
        t_cam.transform.translation.z = 0.10
        t_cam.transform.rotation.w = 1.0
        self.static_tf_broadcaster.sendTransform([t_cam])

    def _apply_wheel_commands(self):
        vx, vy, wz = self.cmd_vx, self.cmd_vy, self.cmd_wz
        # Proteksi Depth Bumper D455: Jika tembok di depan <= 0.28m, cegah gerak maju!
        if self.front_min_dist < 0.28 and vx > 0.0:
            vx = 0.0
        w1, w2, w3 = self.odometry_model.inverse_kinematics(vx, vy, wz)
        for motor, speed in zip(self.motors, [w1, w2, w3]):
            clamped = max(-self.MAX_WHEEL_SPEED, min(self.MAX_WHEEL_SPEED, speed))
            motor.setVelocity(clamped)

    def cmd_vel_callback(self, msg: Twist):
        self.cmd_vx = msg.linear.x
        self.cmd_vy = msg.linear.y
        self.cmd_wz = msg.angular.z
        self._apply_wheel_commands()

    def step(self):
        self.step_count += 1
        stamp = self.get_clock().now().to_msg()

        # ---------------------------------------------------------------------
        # EKSEKUSI LAYER 2 TERLEBIH DAHULU: Baca Depth D455 untuk Deteksi Stall & /scan
        # ---------------------------------------------------------------------
        if self.step_count % 2 == 0:
            range_ptr = self.d455_depth.range_image
            if range_ptr:
                w = self.scan_converter.width
                h = self.scan_converter.height
                depth_bytes = ctypes.string_at(range_ptr, w * h * 4)

                ranges, self.front_min_dist, self.is_scene_stationary = (
                    self.scan_converter.analyze_depth_and_extract_scan(depth_bytes)
                )
                # Terapkan ulang rem otomatis jika jarak depan terlalu dekat
                self._apply_wheel_commands()

                scan_msg = LaserScan()
                scan_msg.header.stamp = stamp
                scan_msg.header.frame_id = 'camera_link'
                scan_msg.angle_min = self.scan_converter.angle_min
                scan_msg.angle_max = self.scan_converter.angle_max
                scan_msg.angle_increment = self.scan_converter.angle_increment
                scan_msg.time_increment = 0.0
                scan_msg.scan_time = self.dt * 2.0
                scan_msg.range_min = self.scan_converter.range_min
                scan_msg.range_max = self.scan_converter.range_max
                scan_msg.ranges = ranges
                self.scan_pub.publish(scan_msg)

                depth_img_msg = Image()
                depth_img_msg.header.stamp = stamp
                depth_img_msg.header.frame_id = 'camera_link'
                depth_img_msg.height = h
                depth_img_msg.width = w
                depth_img_msg.encoding = '32FC1'
                depth_img_msg.step = w * 4
                depth_img_msg.data = depth_bytes
                self.depth_pub.publish(depth_img_msg)

            color_bytes = self.d455_color.getImage()
            if color_bytes:
                rgb_msg = Image()
                rgb_msg.header.stamp = stamp
                rgb_msg.header.frame_id = 'camera_link'
                rgb_msg.height = self.d455_color.getHeight()
                rgb_msg.width = self.d455_color.getWidth()
                rgb_msg.encoding = 'bgra8'
                rgb_msg.step = rgb_msg.width * 4
                rgb_msg.data = color_bytes
                self.rgb_pub.publish(rgb_msg)

        # ---------------------------------------------------------------------
        # EKSEKUSI LAYER 1: Proprioceptive Motion Update + Anti-Slip Guard
        # ---------------------------------------------------------------------
        phi1 = self.encoders[0].getValue()
        phi2 = self.encoders[1].getValue()
        phi3 = self.encoders[2].getValue()
        rpy = self.imu.getRollPitchYaw()
        gyro_vals = self.gyro.getValues()
        accel_vals = self.accel.getValues()

        if any(math.isnan(v) for v in [phi1, phi2, phi3]):
            return

        x, y, theta, vx, vy, wz = self.odometry_model.update(
            phi1, phi2, phi3, rpy[2], self.dt, is_stalled=self.is_scene_stationary
        )
        q_odom = euler_to_quaternion(theta)

        t_odom = TransformStamped()
        t_odom.header.stamp = stamp
        t_odom.header.frame_id = 'odom'
        t_odom.child_frame_id = 'base_link'
        t_odom.transform.translation.x = x
        t_odom.transform.translation.y = y
        t_odom.transform.translation.z = 0.0
        t_odom.transform.rotation = q_odom
        self.tf_broadcaster.sendTransform(t_odom)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = x
        odom_msg.pose.pose.position.y = y
        odom_msg.pose.pose.orientation = q_odom
        odom_msg.pose.covariance = self.odometry_model.get_pose_covariance_6x6()
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.angular.z = wz
        self.odom_pub.publish(odom_msg)

        imu_msg = Imu()
        imu_msg.header.stamp = stamp
        imu_msg.header.frame_id = 'camera_link'
        imu_msg.orientation = euler_to_quaternion(rpy[2], rpy[1], rpy[0])
        imu_msg.angular_velocity.x = gyro_vals[0]
        imu_msg.angular_velocity.y = gyro_vals[1]
        imu_msg.angular_velocity.z = gyro_vals[2]
        imu_msg.linear_acceleration.x = accel_vals[0]
        imu_msg.linear_acceleration.y = accel_vals[1]
        imu_msg.linear_acceleration.z = accel_vals[2]
        self.imu_pub.publish(imu_msg)

        # Ground Truth Pose relatif terhadap titik awal (untuk pembanding di RViz2)
        gt_vals = self.gps.getValues()
        if gt_vals and not math.isnan(gt_vals[0]):
            if self.init_gt_xy is None:
                self.init_gt_xy = (gt_vals[0], gt_vals[1])
                self.init_gt_yaw = rpy[2]
            dx_w = gt_vals[0] - self.init_gt_xy[0]
            dy_w = gt_vals[1] - self.init_gt_xy[1]
            c0, s0 = math.cos(self.init_gt_yaw), math.sin(self.init_gt_yaw)
            gt_x = dx_w * c0 + dy_w * s0
            gt_y = -dx_w * s0 + dy_w * c0

            gt_msg = PoseStamped()
            gt_msg.header.stamp = stamp
            gt_msg.header.frame_id = 'map'
            gt_msg.pose.position.x = gt_x
            gt_msg.pose.position.y = gt_y
            gt_msg.pose.position.z = 0.0
            gt_msg.pose.orientation = q_odom
            self.gt_pub.publish(gt_msg)


def main(args=None):
    robot = Robot()
    timestep = int(robot.getBasicTimeStep())
    rclpy.init(args=args)
    node = BroneD455DriverNode(robot)

    try:
        while rclpy.ok() and robot.step(timestep) != -1:
            rclpy.spin_once(node, timeout_sec=0.0)
            node.step()
    except KeyboardInterrupt:
        pass
    finally:
        for m in node.motors:
            m.setVelocity(0.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
