#!/usr/bin/env python3
"""
RoboMaster 自动探索建图巡航节点 (基于激光雷达闭合曲线分析与大阈值开环未知区域引导)
- 核心逻辑:
  1. 维持平稳、缓慢前进 (linear.x ~ 0.22 m/s)，保证激光建图质量与平稳度。
  2. 闭合曲线与未知区域界定 (大阈值策略):
     针对单线/低线束激光雷达(360点)，远距离激光点物理间距扩散大的特性，采用大跨距阈值判断连续性。
     若相邻激光反射点物理间距 < 大阈值，视为连续闭合墙体/死胡同；
     若出现断裂间隙(间距 > 大阈值)或射线达到深远开阔处，判定为非闭合未知区域(Frontier Gap)。
  3. 自动导向: 计算未知开阔区域的最佳引导角，平滑控制车头“开进”未知区域，遇死胡同自动掉头。
"""

import math
import random
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist


class FrontierAutoExplorer(Node):
    def __init__(self):
        super().__init__('auto_explorer')

        # 1. 通信接口配置
        self.scan_sub = self.create_subscription(
            LaserScan,
            '/red_robot/scan',
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            '/red_robot/cmd_vel',
            10
        )

        # 2. 定时控制循环 (10 Hz，0.1秒一次决策)
        self.timer = self.create_timer(0.1, self.control_loop)

        # 3. 速度与安全参数 (平稳低速前行)
        self.base_forward_speed = 0.22      # 缓慢前进基准线速度 (m/s)
        self.emergency_stop_dist = 0.42     # 紧急避碰前向安全底线 (m)
        self.side_safe_margin = 0.35        # 车体侧面安全间距 (m)

        # 4. 低线束雷达大阈值策略参数
        # 360点雷达在3~4米处相邻射线发散严重，设为0.8米作为判断闭合性边界的大阈值
        self.closed_curve_gap_thresh = 0.85 # 断开判定大阈值 (米): 两点距离大于此值视作"非闭合开口"
        self.open_frontier_range = 2.6      # 深远未知空旷判定距离 (米): 大于此距离直接视作开阔未知地带

        # 5. 内部状态变量
        self.latest_scan = None
        self.escape_mode = False            # 是否陷入全封闭死角脱困模式
        self.escape_ticks = 0

        self.get_logger().info('RoboMaster 闭合曲线大阈值自动探索巡航节点已就绪！')

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def detect_unknown_frontiers(self, ranges, angle_min, angle_increment):
        """
        利用大阈值策略分析雷达点云的连续性，提取未闭合的未知开阔区域
        返回: 最佳未知区域的目标偏航角 (弧度，正为左，负为右)
        """
        num_points = len(ranges)
        if num_points == 0:
            return 0.0, False

        # 转换极坐标为平面直角坐标 (用于计算相邻点物理欧氏距离)
        cartesian_points = []
        for i in range(num_points):
            r = ranges[i]
            # 过滤无效或超远点，限制在有效感应范围
            valid_r = r if (0.1 < r < 12.0 and not math.isinf(r) and not math.isnan(r)) else 10.0
            angle = angle_min + i * angle_increment
            x = valid_r * math.cos(angle)
            y = valid_r * math.sin(angle)
            cartesian_points.append((valid_r, angle, x, y))

        # 寻找非闭合开口 (Frontier Gaps)
        open_frontiers = []
        current_open_cluster = []

        for i in range(num_points):
            r, angle, x, y = cartesian_points[i]
            next_idx = (i + 1) % num_points
            next_r, _, next_x, next_y = cartesian_points[next_idx]

            # 计算两点间的真实欧氏距离
            euclidean_dist = math.hypot(next_x - x, next_y - y)

            # 大阈值判定:
            # 1. 距离大跳跃 (> closed_curve_gap_thresh) -> 闭合曲线断开，属于开环边界
            # 2. 或者该点直接射向远方 (> open_frontier_range) -> 深度未探索开阔地带
            is_open_gap = (euclidean_dist > self.closed_curve_gap_thresh) or (r > self.open_frontier_range)

            if is_open_gap:
                current_open_cluster.append((r, angle))
            else:
                if len(current_open_cluster) > 0:
                    open_frontiers.append(current_open_cluster)
                    current_open_cluster = []

        if len(current_open_cluster) > 0:
            open_frontiers.append(current_open_cluster)

        # 如果全景所有相邻点距离都很小且很近，说明小车陷入了全封闭闭合凹坑 (Dead End)
        if len(open_frontiers) == 0:
            return 0.0, True

        # 对所有未闭合未知开口进行打分，选出最适合走进去的一个:
        # 打分原则: 深度越深 + 宽度越大 + 偏角越朝向车前 (-90° ~ +90°) 得分越高
        best_angle = 0.0
        max_score = -1.0

        for frontier in open_frontiers:
            avg_r = sum(p[0] for p in frontier) / len(frontier)
            mid_p = frontier[len(frontier) // 2]
            center_angle = mid_p[1]
            
            # 将角度归一化到 [-pi, pi]
            norm_angle = math.atan2(math.sin(center_angle), math.cos(center_angle))

            # 偏好车前扇区 (-100° 到 +100°)，避免盲目倒车
            angle_preference = max(0.1, math.cos(norm_angle / 2.0))
            score = (avg_r ** 1.5) * len(frontier) * angle_preference

            if score > max_score:
                max_score = score
                best_angle = norm_angle

        return best_angle, False

    def control_loop(self):
        if self.latest_scan is None:
            return

        ranges = self.latest_scan.ranges
        num_points = len(ranges)
        if num_points == 0:
            return

        twist = Twist()

        # 1. 紧急安全底线检测 (计算车正前方正负25度的最近障碍物)
        def get_sector_min(start_deg, end_deg):
            start_i = int((start_deg / 360.0) * num_points) % num_points
            end_i = int((end_deg / 360.0) * num_points) % num_points
            vals = []
            idx = start_i
            while True:
                r = ranges[idx]
                if 0.08 < r < 12.0 and not math.isinf(r) and not math.isnan(r):
                    vals.append(r)
                if idx == end_i:
                    break
                idx = (idx + 1) % num_points
            return min(vals) if len(vals) > 0 else 10.0

        front_dist = min(get_sector_min(335, 359), get_sector_min(0, 25))
        front_left = get_sector_min(25, 75)
        front_right = get_sector_min(285, 335)

        # 脱困模式倒计时
        if self.escape_ticks > 0:
            self.escape_ticks -= 1
            twist.linear.x = 0.05
            twist.angular.z = 0.8  # 原地旋转寻找出口
            self.cmd_pub.publish(twist)
            return

        # 2. 局部安全防护 (紧急避碰优先级最高)
        if front_dist < self.emergency_stop_dist:
            # 正前方撞障，停止前进并根据左右空间旋转脱困
            twist.linear.x = 0.0
            twist.angular.z = 0.7 if front_left > front_right else -0.7
            self.cmd_pub.publish(twist)
            return

        # 3. 闭合曲线与未知区域探索引导
        target_angle, is_closed_trap = self.detect_unknown_frontiers(
            ranges, self.latest_scan.angle_min, self.latest_scan.angle_increment
        )

        if is_closed_trap or front_dist < self.emergency_stop_dist * 1.3:
            # 检测到全封闭死胡同，启动自转探索
            self.escape_ticks = random.randint(15, 25)
            twist.linear.x = 0.02
            twist.angular.z = 0.85
            self.cmd_pub.publish(twist)
            return

        # 4. 朝着非闭合的未知区域平稳开进
        # 始终保持缓慢前进 (v_x 在 0.15 ~ 0.25 之间根据航向微调)
        twist.linear.x = max(0.12, self.base_forward_speed * math.cos(target_angle))

        # P 比例控制器产生偏航角速度，引导车头指向未知开阔口
        kp = 0.95
        angular_val = kp * target_angle
        # 限制角速度，防止过冲甩尾
        twist.angular.z = max(-0.8, min(0.8, angular_val))

        # 贴近侧墙时给予适量斥力修正
        if front_left < self.side_safe_margin:
            twist.angular.z -= 0.25
        elif front_right < self.side_safe_margin:
            twist.angular.z += 0.25

        self.cmd_pub.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierAutoExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        stop_cmd = Twist()
        node.cmd_pub.publish(stop_cmd)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
