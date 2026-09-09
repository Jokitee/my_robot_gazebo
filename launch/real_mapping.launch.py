#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
RoboMaster 真实机器人 2D 激光雷达 SLAM 建图一键启动 Launch
包含:
1. STM32 F4 CDC 底盘串口驱动 (下发 50Hz 控制帧 / 接收 20 字节反馈 / 发布 /odom 与 odom->base_link TF)
2. 真实激光雷达驱动节点 (lidar_pkg)
3. base_link -> laser 静态坐标变换广播
4. slam_toolbox 实时激光 SLAM 建图
5. RViz2 建图可视化监控
6. (可选) auto_explorer.py 闭合边界自主探索巡航
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_gazebo')

    # 1. 配置文件路径
    slam_config_path = os.path.join(pkg_dir, 'config', 'slam_toolbox_params.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'real_mapping.rviz')

    # 2. 声明 Launch 参数
    chassis_port_arg = DeclareLaunchArgument(
        'chassis_port',
        default_value='/dev/ttyACM0',
        description='STM32 F4 底盘 CDC 虚拟串口设备路径 (通常为 /dev/ttyACM0 或 /dev/ttyACM1)'
    )

    lidar_port_arg = DeclareLaunchArgument(
        'lidar_port',
        default_value='/dev/ttyUSB0',
        description='真实激光雷达串口设备路径 (通常为 /dev/ttyUSB0)'
    )

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='是否自动打开 RViz2 监控建图过程'
    )

    use_auto_explorer_arg = DeclareLaunchArgument(
        'use_auto_explorer',
        default_value='false',
        description='是否启动自主探索节点 (true: 机器人自动巡航建图; false: 手动遥控或键盘建图)'
    )

    # 3. 底盘驱动节点 (F4 串口通信)
    chassis_node = Node(
        package='my_robot_gazebo',
        executable='chassis_driver.py',
        name='chassis_driver',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('chassis_port'),
            'baudrate': 115200,
            'publish_rate': 50.0,
            'max_vx': 1.5,
            'max_vy': 1.5,
            'max_wz': 3.0,
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': True
        }]
    )

    # 4. 真实激光雷达驱动节点 (内置于 my_robot_gazebo)
    lidar_node = Node(
        package='my_robot_gazebo',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[{
            'port_name': LaunchConfiguration('lidar_port'),
            'frame_id': 'laser',
            'filter.enabled': True,
            'filter.radius': 0.10,
            'filter.min_neighbors': 2,
            'angle_crop.enabled': True,
            'angle_crop.min_deg': 270.0,
            'angle_crop.max_deg': 90.0
        }]
    )

    # 5. 静态坐标变换发布 (base_link -> laser)
    # 雷达安装在底盘前方中心偏上位置: x=0.10m, y=0.0m, z=0.15m
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['0.10', '0.0', '0.15', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    # 6. SLAM Toolbox 在线建图节点
    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[slam_config_path]
    )

    # 7. RViz2 监控界面
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen'
    )

    # 8. 自主闭合边界探索节点 (可选)
    auto_explorer_node = Node(
        package='my_robot_gazebo',
        executable='auto_explorer.py',
        name='auto_explorer',
        output='screen',
        parameters=[{
            'scan_topic': '/scan',
            'cmd_topic': '/cmd_vel'
        }],
        condition=IfCondition(LaunchConfiguration('use_auto_explorer'))
    )

    return LaunchDescription([
        chassis_port_arg,
        lidar_port_arg,
        use_rviz_arg,
        use_auto_explorer_arg,
        chassis_node,
        lidar_node,
        static_tf_node,
        slam_node,
        rviz_node,
        auto_explorer_node
    ])
