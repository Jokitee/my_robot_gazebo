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

        # 1. 通信接口参数
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('cmd_topic', '/cmd_vel')
        scan_topic = self.get_parameter('scan_topic').value
        cmd_topic = self.get_parameter('cmd_topic').value

        self.scan_sub = self.create_subscription(
            LaserScan,
            scan_topic,
            self.scan_callback,
            10
        )
        self.cmd_pub = self.create_publisher(
            Twist,
            cmd_topic,
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
        利用大阈值策略分析雷达有效扇区（前向左右180°）的点云连续性，提取未闭合的未知开阔区域
        返回: 最佳未知区域的目标偏航角 (弧度，正为左，负为右)
        """
        num_points = len(ranges)
        if num_points == 0:
            return 0.0, False

        # 1. 仅提取前向左右180度 [-90°, +90°] 内的有效扇区点云
        fov_half_rad = math.radians(90.0)
        valid_front_points = []

        for i in range(num_points):
            r = ranges[i]
            raw_angle = angle_min + i * angle_increment
            # 归一化到 [-pi, pi]
            norm_angle = math.atan2(math.sin(raw_angle), math.cos(raw_angle))

            # 仅处理前向半周 180° 有效视野
            if abs(norm_angle) <= fov_half_rad:
                # 过滤无效或超远点
                if math.isinf(r) or math.isnan(r) or r > 10.0:
                    effective_r = 10.0  # 视作深远开阔未知区域
                elif r < 0.05:
                    effective_r = 0.05
                else:
                    effective_r = r

                x = effective_r * math.cos(norm_angle)
                y = effective_r * math.sin(norm_angle)
                valid_front_points.append((norm_angle, effective_r, x, y))

        if len(valid_front_points) < 5:
            return 0.0, True

        # 按角度从右 (-90°) 到左 (+90°) 排序
        valid_front_points.sort(key=lambda p: p[0])

        # 2. 在 180° 扇区弧段上利用大阈值策略寻找非闭合开口 (Frontier Gaps)
        open_frontiers = []
        current_open_cluster = []

        for idx in range(len(valid_front_points) - 1):
            angle, r, x, y = valid_front_points[idx]
            next_angle, next_r, next_x, next_y = valid_front_points[idx + 1]

            # 计算相邻两点间的空间欧氏距离
            euclidean_dist = math.hypot(next_x - x, next_y - y)

            # 大阈值判定:
            # 1. 两点欧氏距离大跳变 (> closed_curve_gap_thresh) -> 闭合曲线在此断开，形成开阔边界
            # 2. 或者该点测距很远 (> open_frontier_range) -> 深度未探索开阔地带
            is_open_gap = (euclidean_dist > self.closed_curve_gap_thresh) or (r > self.open_frontier_range)

            if is_open_gap:
                current_open_cluster.append((r, angle))
            else:
                if len(current_open_cluster) > 0:
                    open_frontiers.append(current_open_cluster)
                    current_open_cluster = []

        if len(current_open_cluster) > 0:
            open_frontiers.append(current_open_cluster)

        # 如果前向 180° 扇区内所有点距离都很小且紧密连成闭合墙面，说明正前方处于封闭凹坑/死胡同
        if len(open_frontiers) == 0:
            return 0.0, True

        # 3. 对所有未闭合未知开口进行打分评估，选出最优开进方向
        # 评分准则: 深度越深 + 宽度越大 + 偏角越朝向正前方 (-45° ~ +45°) 得分越高
        best_angle = 0.0
        max_score = -1.0

        for frontier in open_frontiers:
            avg_r = sum(p[0] for p in frontier) / len(frontier)
            mid_p = frontier[len(frontier) // 2]
            center_angle = mid_p[1]

            # 偏好正前方扇区，避免过急转弯
            angle_preference = max(0.2, math.cos(center_angle / 1.5))
            score = (avg_r ** 1.5) * len(frontier) * angle_preference

            if score > max_score:
                max_score = score
                best_angle = center_angle

        return best_angle, False

    def control_loop(self):
        if self.latest_scan is None:
            return

        ranges = self.latest_scan.ranges
        num_points = len(ranges)
        if num_points == 0:
            return

        angle_min = self.latest_scan.angle_min
        angle_increment = self.latest_scan.angle_increment

        twist = Twist()

        # 1. 提取前向扇区最近障碍物距离 (角度采用真实物理角度，完全适配 180° 雷达)
        front_vals = []
        left_vals = []
        right_vals = []

        for i in range(num_points):
            r = ranges[i]
            if r < 0.08 or r > 10.0 or math.isinf(r) or math.isnan(r):
                continue

            raw_ang = angle_min + i * angle_increment
            norm_ang = math.atan2(math.sin(raw_ang), math.cos(raw_ang))

            # 正前方安全区: [-22°, +22°]
            if abs(norm_ang) <= math.radians(22):
                front_vals.append(r)
            # 前左侧扇区: [+22°, +80°]
            elif math.radians(22) < norm_ang <= math.radians(80):
                left_vals.append(r)
            # 前右侧扇区: [-80°, -22°]
            elif -math.radians(80) <= norm_ang < -math.radians(22):
                right_vals.append(r)

        front_dist = min(front_vals) if len(front_vals) > 0 else 10.0
        front_left = min(left_vals) if len(left_vals) > 0 else 10.0
        front_right = min(right_vals) if len(right_vals) > 0 else 10.0

        # 脱困模式倒计时
        if self.escape_ticks > 0:
            self.escape_ticks -= 1
            twist.linear.x = 0.04
            twist.angular.z = 0.8  # 原地慢转寻找新开阔面
            self.cmd_pub.publish(twist)
            return

        # 2. 局部安全防护 (紧急避碰优先级最高)
        if front_dist < self.emergency_stop_dist:
            # 正前方逼近障碍物，停止前进并根据左右空间旋转脱困
            twist.linear.x = 0.0
            twist.angular.z = 0.7 if front_left > front_right else -0.7
            self.cmd_pub.publish(twist)
            return

        # 3. 闭合曲线与未知区域探索引导 (针对前向180度有效扇区)
        target_angle, is_closed_trap = self.detect_unknown_frontiers(
            ranges, angle_min, angle_increment
        )

        if is_closed_trap or front_dist < self.emergency_stop_dist * 1.3:
            # 前方 180° 闭合封闭，启动原地旋转探索其他朝向
            self.escape_ticks = random.randint(18, 30)
            twist.linear.x = 0.02
            twist.angular.z = 0.85
            self.cmd_pub.publish(twist)
            return

        # 4. 朝着非闭合的未知区域平稳开进
        # 始终保持缓慢前进 (v_x 在 0.12 ~ 0.22 之间根据航向微调)
        twist.linear.x = max(0.12, self.base_forward_speed * math.cos(target_angle))

        # P 比例控制器产生偏航角速度，引导车头对准未知开口
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
