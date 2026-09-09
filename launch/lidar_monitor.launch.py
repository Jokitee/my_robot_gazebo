#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
单雷达 180° 有效区块独立 RViz 监控启动程序
包含:
1. lidar_node: 真实激光雷达驱动，输出 [-90°, +90°] 180° 扫描与绿色半透明区块 Marker
2. static_transform_publisher: 建立 base_link -> laser TF 坐标系
3. rviz2: 启动 RViz 监视器，实时查看 180° 限定扇区与激光点云
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_gazebo')

    config_path = os.path.join(pkg_dir, 'config', 'lidar_params.yaml')
    rviz_config_path = os.path.join(pkg_dir, 'lidar_pkg', 'rviz', 'lidar.rviz')

    port_arg = DeclareLaunchArgument(
        'port_name',
        default_value='/dev/ttyACM0',
        description='Lidar serial port'
    )

    lidar_node = Node(
        package='my_robot_gazebo',
        executable='lidar_node',
        name='lidar_node',
        output='screen',
        parameters=[config_path, {'port_name': LaunchConfiguration('port_name')}]
    )

    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_broadcaster',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', 'base_link', 'laser']
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', rviz_config_path]
    )

    return LaunchDescription([
        port_arg,
        lidar_node,
        static_tf_node,
        rviz_node
    ])
