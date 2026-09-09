import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_gazebo')
    world_path = os.path.join(pkg_dir, 'worlds', 'rm_world.sdf')
    model_path = os.path.join(pkg_dir, 'models', 'rm_robot', 'model.sdf')
    target_path = os.path.join(pkg_dir, 'models', 'armor_target', 'model.sdf')

    # 1. 启动 Gazebo Harmonic 仿真环境
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

        # 2. 生成红方步兵机器人 (显式指定 -world rm_world，避免超时死等)
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'rm_world',
            '-name', 'red_robot',
            '-file', model_path,
            '-x', '-4.5', '-y', '0.0', '-z', '0.15'
        ],
        output='screen'
    )

    # 3. 生成装甲板目标
    spawn_target = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-world', 'rm_world',
            '-name', 'armor_target',
            '-file', target_path,
            '-x', '2.0', '-y', '0.0', '-z', '0.0'
        ],
        output='screen'
    )


    # 4. ROS-Gazebo 话题桥接
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/red_robot/cmd_vel@geometry_msgs/msg/Twist@ignition.msgs.Twist',
            '/model/red_robot/joint/gimbal_yaw_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/model/red_robot/joint/gimbal_pitch_joint/cmd_vel@std_msgs/msg/Float64@ignition.msgs.Double',
            '/rm_robot/camera/image@sensor_msgs/msg/Image@ignition.msgs.Image',
            '/red_robot/scan@sensor_msgs/msg/LaserScan@ignition.msgs.LaserScan',
        ],
        remappings=[
            ('/rm_robot/camera/image', '/camera/image_raw'),
            ('/red_robot/scan', '/scan'),
        ],
        output='screen'
    )

    # 5. 【新增】雷达静态坐标系广播（消除 RViz2 黄色感叹号）
    # 将 world 坐标系连接到雷达的 frame: red_robot/chassis/rplidar_a2
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_publisher',
        arguments=[
            '0.0', '0.0', '0.35',       # x, y, z 位置偏移 (米)
            '0.0', '0.0', '0.0',       # roll, pitch, yaw 旋转角 (弧度)
            'world',                   # 父坐标系 parent_frame
            'red_robot/chassis/rplidar_a2'  # 子坐标系 child_frame
        ],
        output='screen'
    )

    return LaunchDescription([
        gazebo,
        spawn_robot,
        spawn_target,
        bridge,
        static_tf_lidar,  # 注册进 LaunchDescription
    ])
