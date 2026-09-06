import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_gazebo')
    sdf_model_path = os.path.join(pkg_share, 'models', 'my_robot.sdf')

    # 1. 启动 Gazebo Sim 并加载空世界
    gazebo_sim = ExecuteProcess(
        cmd=['gz', 'sim', '-r', 'empty.sdf'],
        output='screen'
    )

    # 2. 在 Gazebo 中生成机器人
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-file', sdf_model_path, '-name', 'my_robot', '-z', '0.2'],
        output='screen'
    )

    # 3. 启动 ROS 2 <-> Gazebo 话题桥接器 (ros_gz_bridge)
    # 将 Gazebo 中的 /cmd_vel 和 /odom 桥接到 ROS 2
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
