#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
基于 PySerial 的纯 Python 激光雷达驱动节点 (内置 180° 有效扇区与可视化 Marker)
针对 STC8 / CDC 驱动板高度优化，100% 复现 test_lidar.py 物理启停时序
"""

import math
import struct
import time
import serial

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point


class LidarDriverPy(Node):
    def __init__(self):
        super().__init__('lidar_driver_py')

        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('frame_id', 'laser')
        self.declare_parameter('baudrate', 150000)
        self.declare_parameter('angle_crop_enabled', True)

        self.port_name = self.get_parameter('port_name').value
        self.frame_id = self.get_parameter('frame_id').value
        self.baudrate = self.get_parameter('baudrate').value
        self.angle_crop_enabled = self.get_parameter('angle_crop_enabled').value

        self.scan_pub = self.create_publisher(LaserScan, 'scan', 10)
        self.marker_pub = self.create_publisher(Marker, 'lidar_fov_marker', 10)

        self.ser = None
        self.open_serial()

        self.full_scan = []  # [(angle_rad, dist_m)]
        self.last_angle_rad = 0.0
        self.scan_count = 0
        self.scan_start_time = self.get_clock().now()

        # 启动接收定时器 (高频轮询串口 200Hz)
        self.timer = self.create_timer(0.005, self.read_serial_loop)

        # 启动每秒心跳打印
        self.create_timer(2.0, self.heartbeat_log)

    def open_serial(self):
        self.get_logger().info(f"正在以 {self.baudrate} 打开雷达串口: {self.port_name}...")
        try:
            self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
        except Exception as e:
            self.get_logger().fatal(f"打开雷达串口失败: {e}")
            raise e

        # 1. 硬件拉高 DTR/RTS 启停控制线
        self.ser.setDTR(True)
        self.ser.setRTS(True)
        time.sleep(0.2)

        # 2. 发送启动指令 A5 60
        self.get_logger().info("下发物理电机启动指令 (0xA5 0x60)...")
        for _ in range(5):
            self.ser.write(bytes([0xA5, 0x60]))
            time.sleep(0.05)

        self.get_logger().info("雷达初始化指令发送完毕，开始监听扫描点云数据流...")

    def heartbeat_log(self):
        if self.scan_count > 0:
            self.get_logger().info(f">>> 雷达运行正常! 正在持续发布 180° 扫描 [/scan] (已处理: {self.scan_count} 圈)")
        else:
            self.get_logger().warn(">>> 等待雷达数据帧 (如电机未转动，请检查串口是否插错)...")

    def read_serial_loop(self):
        if not self.ser or not self.ser.is_open:
            return

        try:
            n = self.ser.in_waiting
            if n < 8:
                return

            raw = self.ser.read(n)
            self.process_raw_bytes(raw)
        except Exception as e:
            self.get_logger().error(f"串口读取异常: {e}")

    def process_raw_bytes(self, data: bytes):
        if not hasattr(self, '_buf'):
            self._buf = bytearray()

        self._buf.extend(data)

        while len(self._buf) >= 10:
            # 匹配帧头 0xAA 0x55
            if self._buf[0] != 0xAA or self._buf[1] != 0x55:
                idx = self._buf.find(b'\xAA\x55')
                if idx == -1:
                    self._buf.clear()
                    return
                self._buf = self._buf[idx:]
                if len(self._buf) < 10:
                    return

            lsn = self._buf[3]
            target_len = 10 + lsn * 3
            if len(self._buf) < target_len:
                return  # 等待完整数据包

            packet = bytes(self._buf[:target_len])
            self._buf = self._buf[target_len:]
            self.parse_packet(packet, lsn)

    def parse_packet(self, pkt: bytes, lsn: int):
        if lsn == 0:
            return

        fs_raw = (pkt[5] << 8) | pkt[4]
        ls_raw = (pkt[7] << 8) | pkt[6]

        angle_start_deg = (fs_raw >> 1) / 64.0
        angle_end_deg = (ls_raw >> 1) / 64.0

        diff_deg = angle_end_deg - angle_start_deg
        if diff_deg < 0:
            diff_deg += 360.0

        for i in range(lsn):
            offset = 8 + i * 3
            if offset + 1 >= len(pkt):
                break

            dist_raw = (pkt[offset + 1] << 8) | pkt[offset]
            dist_m = dist_raw / 4.0 / 1000.0
            dist_mm = dist_raw / 4.0

            angle_deg = angle_start_deg
            if lsn > 1:
                angle_deg = (diff_deg / (lsn - 1)) * i + angle_start_deg

            # 角度修正公式
            angle_correct_deg = 0.0
            if dist_mm > 0:
                numerator = 21.8 * (155.3 - dist_mm)
                denominator = 155.3 * dist_mm
                if denominator != 0:
                    rad = math.atan(numerator / denominator)
                    angle_correct_deg = math.degrees(rad)

            final_deg = (angle_deg + angle_correct_deg) % 360.0
            angle_rad = math.radians(final_deg)

            if dist_m > 0.05:
                # 过零检测 (一圈结束)
                if angle_rad < self.last_angle_rad - math.pi:
                    if self.scan_count > 0:
                        self.publish_scan()
                    self.scan_count += 1
                    self.full_scan.clear()
                    self.scan_start_time = self.get_clock().now()

                self.last_angle_rad = angle_rad

                # 180° 有效扇区过滤: [270°, 360°] U [0°, 90°]
                if self.angle_crop_enabled:
                    if not (final_deg >= 270.0 or final_deg <= 90.0):
                        continue

                self.full_scan.append((angle_rad, dist_m))

    def publish_scan(self):
        if not self.full_scan:
            return

        scan = LaserScan()
        scan.header.stamp = self.scan_start_time.to_msg()
        scan.header.frame_id = self.frame_id
        scan.range_min = 0.05
        scan.range_max = 8.0

        # 限定 180° 扫描 [-PI/2, +PI/2]
        scan.angle_min = -math.pi / 2.0
        scan.angle_max = math.pi / 2.0
        scan_size = 360
        scan.angle_increment = math.pi / scan_size
        scan.ranges = [float('inf')] * scan_size

        for raw_angle, dist_m in self.full_scan:
            # 转换为 ROS 前向坐标系
            if raw_angle >= 1.5 * math.pi:
                ros_angle = 2.0 * math.pi - raw_angle  # [0, +PI/2] (左)
            elif raw_angle <= 0.5 * math.pi:
                ros_angle = -raw_angle                # [-PI/2, 0] (右)
            else:
                continue

            idx = int((ros_angle - scan.angle_min) / scan.angle_increment)
            if 0 <= idx < scan_size:
                if scan.ranges[idx] == float('inf') or dist_m < scan.ranges[idx]:
                    scan.ranges[idx] = float(dist_m)

        scan.scan_time = 1.0 / 7.0
        scan.time_increment = scan.scan_time / scan_size

        self.scan_pub.publish(scan)
        self.publish_fov_marker(scan.header.stamp)

    def publish_fov_marker(self, stamp):
        # 绿色半透明 180° 扇形
        fan = Marker()
        fan.header.stamp = stamp
        fan.header.frame_id = self.frame_id
        fan.ns = "restricted_fov_zone"
        fan.id = 0
        fan.type = Marker.TRIANGLE_LIST
        fan.action = Marker.ADD
        fan.scale.x = 1.0
        fan.scale.y = 1.0
        fan.scale.z = 1.0
        fan.color.r = 0.0
        fan.color.g = 0.9
        fan.color.b = 0.5
        fan.color.a = 0.20

        radius = 3.0
        seg = 36
        step = math.pi / seg
        o = Point(x=0.0, y=0.0, z=0.0)

        for i in range(seg):
            a1 = -math.pi / 2.0 + i * step
            a2 = a1 + step
            p1 = Point(x=radius * math.cos(a1), y=radius * math.sin(a1), z=0.0)
            p2 = Point(x=radius * math.cos(a2), y=radius * math.sin(a2), z=0.0)
            fan.points.extend([o, p1, p2])

        self.marker_pub.publish(fan)


def main(args=None):
    rclpy.init(args=args)
    node = LidarDriverPy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.ser and node.ser.is_open:
            # 停止指令
            node.ser.write(bytes([0xA5, 0x00, 0xA5, 0x65]))
            node.ser.close()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
