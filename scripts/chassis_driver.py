#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RoboMaster STM32 F4 底盘 ROS 2 串口驱动节点
- 遵循 ROS2_CHASSIS_PACKAGE.md 协议定义
- 50Hz 循环下发 12 字节控制帧 (维持 200ms 心跳看门狗)
- 实时解析 20 字节反馈帧 (实际线速度、角速度、IMU 航向角)
- 发布 /odom 里程计话题及 odom -> base_link TF 动态坐标变换
- 检测遥控器 S2 拨杆档位，异常时实时提示
"""

import math
import struct
import threading
import time
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
import tf2_ros

try:
    import serial
except ImportError:
    serial = None


class ChassisDriver(Node):
    def __init__(self):
        super().__init__('chassis_driver')

        # 1. 声明并获取参数
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('publish_rate', 50.0)      # 发送与控制频率 (Hz)
        self.declare_parameter('cmd_timeout', 0.5)        # cmd_vel 超时时间 (s)
        self.declare_parameter('max_vx', 1.5)             # 最大前后线速度 (m/s) 对应杆量 660
        self.declare_parameter('max_vy', 1.5)             # 最大左右平移速度 (m/s) 对应杆量 660
        self.declare_parameter('max_wz', 3.0)             # 最大自转角速度 (rad/s) 对应杆量 660
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', True)

        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.publish_rate = self.get_parameter('publish_rate').value
        self.cmd_timeout = self.get_parameter('cmd_timeout').value
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_wz = float(self.get_parameter('max_wz').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        # 2. 状态变量
        self.lock = threading.Lock()
        self.target_vx = 0.0
        self.target_vy = 0.0
        self.target_wz = 0.0
        self.last_cmd_time = 0.0

        # 里程计积分状态
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_odom_time = None
        self.last_warn_time = 0.0

        # 3. 初始化发布者与订阅者
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        self.cmd_sub = self.create_subscription(Twist, 'cmd_vel', self.cmd_vel_callback, 10)

        if self.publish_tf:
            self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)

        # 4. 打开串口
        self.ser = None
        self.is_running = True
        self.connect_serial()

        # 5. 启动串口接收后台线程
        self.rx_thread = threading.Thread(target=self.rx_loop, daemon=True)
        self.rx_thread.start()

        # 6. 启动 50Hz 定时器用于持续向下位机发包 (心跳保活)
        timer_period = 1.0 / self.publish_rate
        self.tx_timer = self.create_timer(timer_period, self.tx_callback)

        self.get_logger().info(
            f"RoboMaster 底盘驱动已启动! 端口: {self.port_name}, 波特率: {self.baudrate}, 循环频率: {self.publish_rate}Hz"
        )
        self.get_logger().info(
            "提示: 控制生效前提是遥控器右侧拨杆 S2 切至最上方 (S2 == 1 自动控制模式)!"
        )

    def connect_serial(self):
        if serial is None:
            self.get_logger().error("未安装 pyserial! 请在终端运行: sudo apt-get install python3-serial 或 pip install pyserial")
            return

        while rclpy.ok() and self.is_running:
            try:
                self.ser = serial.Serial(self.port_name, self.baudrate, timeout=0.1)
                self.get_logger().info(f"成功打开串口: {self.port_name}")
                break
            except Exception as e:
                self.get_logger().warn(
                    f"无法打开串口 {self.port_name}: {e}. 将在 2 秒后重试... (请确认设备已连接且有权限: sudo usermod -aG dialout $USER)"
                )
                time.sleep(2.0)

    def cmd_vel_callback(self, msg: Twist):
        with self.lock:
            self.target_vx = msg.linear.x
            self.target_vy = msg.linear.y
            self.target_wz = msg.angular.z
            self.last_cmd_time = time.time()

    def tx_callback(self):
        """
        以固定频率 (50Hz) 向上位机发送 12 字节控制包
        保持心跳以防止下位机 200ms 看门狗保护超时停机
        """
        now = time.time()
        with self.lock:
            if now - self.last_cmd_time > self.cmd_timeout:
                # 超时置零
                vx_send = 0.0
                vy_send = 0.0
                wz_send = 0.0
            else:
                vx_send = self.target_vx
                vy_send = self.target_vy
                wz_send = self.target_wz

        # 映射到摇杆量: -660 ~ +660
        stick_vx = int(max(-660.0, min(660.0, (vx_send / self.max_vx) * 660.0)))
        stick_vy = int(max(-660.0, min(660.0, (vy_send / self.max_vy) * 660.0)))
        stick_wz = int(max(-660.0, min(660.0, (wz_send / self.max_wz) * 660.0)))

        ctrl_mode = 0  # 0: 普通麦轮底盘模式, 1: 小陀螺
        flags = 0
        reserved = 0

        # 打包前 11 字节 (<BhhhBBH)
        data = struct.pack('<BhhhBBH', 0xA5, stick_vx, stick_vy, stick_wz, ctrl_mode, flags, reserved)
        # 计算前 11 字节累加和
        checksum = sum(data) & 0xFF
        packet = data + bytes([checksum])

        if self.ser and self.ser.is_open:
            try:
                self.ser.write(packet)
            except Exception as e:
                self.get_logger().warn(f"串口发送异常: {e}")
                try:
                    self.ser.close()
                except Exception:
                    pass
                self.connect_serial()

    def rx_loop(self):
        """
        后台线程: 读取并解析 STM32 回传的 20 字节状态帧
        """
        buffer = bytearray()
        while rclpy.ok() and self.is_running:
            if not self.ser or not self.ser.is_open:
                time.sleep(0.1)
                continue

            try:
                data = self.ser.read(32)
                if not data:
                    continue
                buffer.extend(data)

                # 寻找帧头 0x5A 并解析 20 字节数据包
                while len(buffer) >= 20:
                    if buffer[0] != 0x5A:
                        # 丢弃非帧头字节
                        buffer.pop(0)
                        continue

                    # 提取前 20 字节
                    frame = buffer[:20]
                    calc_sum = sum(frame[:19]) & 0xFF
                    recv_sum = frame[19]

                    if calc_sum == recv_sum:
                        # 校验通过，解包
                        # <BffhhhBBBBB: 1+4+4+2+2+2+1+1+1+1+1 = 20 字节
                        parsed = struct.unpack('<BffhhhBBBBB', frame)
                        header = parsed[0]
                        yaw_deg = parsed[1]          # 航向角 (度, 左偏为正, 右偏为负)
                        wz_dps = parsed[2]           # Z轴角速度 (度/秒, 逆时针为正)
                        actual_vx_mms = parsed[3]    # 编码器实际前进线速度 (mm/s)
                        actual_vy_mms = parsed[4]    # 编码器实际平移线速度 (mm/s)
                        actual_wz_mrads = parsed[5]  # 编码器实际自转角速度 (mrad/s)
                        sys_state = parsed[6]        # 0=失能, 1=使能
                        active_mode = parsed[7]      # 0=手动, 1=自动, 2=急停
                        rc_s_right = parsed[8]       # 1=上档(自动), 3=中档, 2=下档(急停)

                        # 移出已处理的一整包
                        buffer = buffer[20:]

                        # 处理数据与发布里程计
                        self.process_feedback(
                            yaw_deg, wz_dps,
                            actual_vx_mms, actual_vy_mms, actual_wz_mrads,
                            sys_state, active_mode, rc_s_right
                        )
                    else:
                        # 校验失败，说明帧头碰巧匹配但数据不对，移出该错误帧头
                        buffer.pop(0)

            except Exception as e:
                self.get_logger().warn(f"串口接收异常: {e}")
                time.sleep(0.05)

    def process_feedback(self, yaw_deg, wz_dps, actual_vx_mms, actual_vy_mms, actual_wz_mrads,
                         sys_state, active_mode, rc_s_right):
        now_time = self.get_clock().now()
        current_time_sec = now_time.nanoseconds * 1e-9

        # 检测右拨杆档位: 若不是上档 1，节流输出告警提示
        if rc_s_right != 1:
            if current_time_sec - self.last_warn_time > 3.0:
                self.last_warn_time = current_time_sec
                self.get_logger().warn(
                    f"【遥控器状态警告】遥控器右侧S2拨杆当前为档位 {rc_s_right} (非上档 1)！底盘不会执行ROS自动控制指令，请将右侧S2拨杆拨到最上方！"
                )

        # 单位换算
        vx = actual_vx_mms / 1000.0           # mm/s -> m/s
        vy = actual_vy_mms / 1000.0           # mm/s -> m/s
        wz = actual_wz_mrads / 1000.0         # mrad/s -> rad/s
        yaw_rad = math.radians(yaw_deg)        # deg -> rad

        # 里程计位姿积分
        if self.last_odom_time is None:
            self.last_odom_time = current_time_sec
            self.yaw = yaw_rad
            return

        dt = current_time_sec - self.last_odom_time
        self.last_odom_time = current_time_sec

        if dt > 0.5:
            # 间隔过大，重置 dt 防止跳变
            dt = 0.02

        # 麦克纳姆轮底盘在世界坐标系 (odom) 下的位移增量
        # 机器人在当前航向角 yaw 下运动
        delta_x = (vx * math.cos(yaw_rad) - vy * math.sin(yaw_rad)) * dt
        delta_y = (vx * math.sin(yaw_rad) + vy * math.cos(yaw_rad)) * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw = yaw_rad  # 直接使用 STM32 高频陀螺仪积分出的高精度航向角

        # 计算四元数 (Z 轴旋转)
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # 构造并发布 Odometry 消息
        odom_msg = Odometry()
        odom_msg.header.stamp = now_time.to_msg()
        odom_msg.header.frame_id = self.odom_frame
        odom_msg.child_frame_id = self.base_frame

        # 位姿
        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = 0.0
        odom_msg.pose.pose.orientation.y = 0.0
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        # 速度
        odom_msg.twist.twist.linear.x = vx
        odom_msg.twist.twist.linear.y = vy
        odom_msg.twist.twist.linear.z = 0.0
        odom_msg.twist.twist.angular.z = wz

        self.odom_pub.publish(odom_msg)

        # 广播 odom -> base_link TF 动态变换
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now_time.to_msg()
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = 0.0
            t.transform.rotation.y = 0.0
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw
            self.tf_broadcaster.sendTransform(t)

    def destroy_node(self):
        self.is_running = False
        if self.ser and self.ser.is_open:
            try:
                # 停机前发送全零停止包
                data = struct.pack('<BhhhBBH', 0xA5, 0, 0, 0, 0, 0, 0)
                pkt = data + bytes([sum(data) & 0xFF])
                self.ser.write(pkt)
                self.ser.close()
            except Exception:
                pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ChassisDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
