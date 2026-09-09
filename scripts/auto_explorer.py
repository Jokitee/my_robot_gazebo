#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
import math
import random

class AutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        # 订阅激光雷达
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/red_robot/scan',
            self.scan_callback,
            10
        )

        # 发布底盘控制速度
        self.cmd_pub = self.create_publisher(
            Twist,
            '/red_robot/cmd_vel',
            10
        )

        # 控制定时器：每秒执行 10 次状态决策
        self.timer = self.create_timer(0.1, self.control_loop)

        # 避障距离阈值 (米)
        self.safe_distance = 0.65       # 前方安全距离
        self.side_safe_dist = 0.45      # 侧向安全距离

        # 扇区距离
        self.front_dist = 10.0
        self.left_dist = 10.0
        self.right_dist = 10.0

        # 巡航状态机: 'FORWARD', 'TURN_LEFT', 'TURN_RIGHT', 'REVERSE'
        self.state = 'FORWARD'
        self.state_ticks = 0
        self.get_logger().info('RoboMaster 自动探索建图巡航节点已启动！')

    def scan_callback(self, msg: LaserScan):
        ranges = msg.ranges
        num_readings = len(ranges)
        if num_readings == 0:
            return

        def get_min_range(start_deg, end_deg):
            """计算雷达特定角度范围内的最近障碍物距离"""
            start_idx = int((start_deg / 360.0) * num_readings) % num_readings
            end_idx = int((end_deg / 360.0) * num_readings) % num_readings
            
            valid_vals = []
            idx = start_idx
            while True:
                r = ranges[idx]
                if msg.range_min < r < msg.range_max and not math.isinf(r) and not math.isnan(r):
                    valid_vals.append(r)
                if idx == end_idx:
                    break
                idx = (idx + 1) % num_readings

            return min(valid_vals) if len(valid_vals) > 0 else 10.0

        # 划分三个主要感知扇区：
        # 前方: 正负 30 度 (330° 到 30°)
        # 左方: 30° 到 90°
        # 右方: 270° 到 330°
        self.front_dist = min(get_min_range(330, 359), get_min_range(0, 30))
        self.left_dist = get_min_range(30, 90)
        self.right_dist = get_min_range(270, 330)

    def control_loop(self):
        twist = Twist()

        if self.state_ticks > 0:
            self.state_ticks -= 1

        # 状态机行为决策
        if self.state == 'FORWARD':
            # 前方遇障，转向避障
            if self.front_dist < self.safe_distance:
                if self.left_dist > self.right_dist:
                    self.state = 'TURN_LEFT'
                    self.state_ticks = random.randint(15, 25) # 旋转 1.5~2.5 秒
                else:
                    self.state = 'TURN_RIGHT'
                    self.state_ticks = random.randint(15, 25)
            # 贴近侧墙时微调航向
            elif self.left_dist < self.side_safe_dist:
                twist.linear.x = 0.3
                twist.angular.z = -0.4
                self.cmd_pub.publish(twist)
                return
            elif self.right_dist < self.side_safe_dist:
                twist.linear.x = 0.3
                twist.angular.z = 0.4
                self.cmd_pub.publish(twist)
                return
            else:
                # 开阔地带正常巡航加速
                twist.linear.x = 0.5
                twist.angular.z = 0.0
                self.cmd_pub.publish(twist)
                return

        elif self.state == 'TURN_LEFT':
            twist.linear.x = 0.05
            twist.angular.z = 0.8
            if self.front_dist > self.safe_distance * 1.3 and self.state_ticks == 0:
                self.state = 'FORWARD'

        elif self.state == 'TURN_RIGHT':
            twist.linear.x = 0.05
            twist.angular.z = -0.8
            if self.front_dist > self.safe_distance * 1.3 and self.state_ticks == 0:
                self.state = 'FORWARD'

        self.cmd_pub.publish(twist)

def main(args=None):
    rclpy.init(args=args)
    node = AutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 停车
        stop_twist = Twist()
        node.cmd_pub.publish(stop_twist)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
