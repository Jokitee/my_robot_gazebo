import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_gazebo')
    sdf_model_path = os.path.join(pkg_share, 'models', 'my_robot.sdf')

    # 1. 官方标准方式：导入 ros_gz_sim 的 gz_sim.launch.py 启动 Gazebo
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 2. 在 Gazebo 中生成机器人模型
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_model_path, '-name', 'my_robot', '-z', '0.2'],
        output='screen'
    )

    # 3. 启动 ROS 2 <-> Gazebo 话题桥接器 (ros_gz_bridge)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry@gz.msgs.Odometry'
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo_sim,
        spawn_robot,
        bridge
    ])
