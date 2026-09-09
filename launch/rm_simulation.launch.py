import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_gazebo')
    world_path = os.path.join(pkg_dir, 'worlds', 'rm_world.sdf')
    model_path = os.path.join(pkg_dir, 'models', 'rm_robot', 'model.sdf')
    target_path = os.path.join(pkg_dir, 'models', 'armor_target', 'model.sdf')
    models_dir = os.path.join(pkg_dir, 'models')
    worlds_dir = os.path.join(pkg_dir, 'worlds')

    # 1. 注入 Gazebo 资源路径，确保能够找到模型 mesh 与材质，避免加载超时
    resource_path = f"{pkg_dir}:{models_dir}:{worlds_dir}"
    set_gz_resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=resource_path
    )
    set_ign_resource_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=resource_path
    )

    # 2. 启动 Gazebo 仿真世界 (-r 确保物理引擎直接运行)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        output='screen'
    )

    # 3. 延时 3.5 秒生成红方步兵机器人（等 Gazebo 服务完全就绪，彻底避免 create timed out）
    spawn_robot = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'rm_world',
                    '-name', 'red_robot',
                    '-file', model_path,
                    '-x', '-4.5',
                    '-y', '0.0',
                    '-z', '0.15'
                ],
                output='screen'
            )
        ]
    )

    # 4. 延时 5.5 秒生成装甲板目标（错开 2 秒，避免两个模型并发请求造成服务端竞争死锁）
    spawn_target = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='ros_gz_sim',
                executable='create',
                arguments=[
                    '-world', 'rm_world',
                    '-name', 'armor_target',
                    '-file', target_path,
                    '-x', '2.0',
                    '-y', '0.0',
                    '-z', '0.0'
                ],
                output='screen'
            )
        ]
    )

    # 5. 延时 2.0 秒启动 ROS-Gazebo 话题桥接器
    bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    # 时钟同步
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    # 速度控制与里程计
                    '/red_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
                    '/red_robot/odometry@nav_msgs/msg/Odometry@gz.msgs.Odometry',
                    # 云台关节控制
                    '/model/red_robot/joint/gimbal_yaw_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
                    '/model/red_robot/joint/gimbal_pitch_joint/cmd_vel@std_msgs/msg/Float64@gz.msgs.Double',
                    # 传感器：单目相机与激光雷达
                    '/rm_robot/camera/image@sensor_msgs/msg/Image@gz.msgs.Image',
                    '/red_robot/scan@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
                ],
                remappings=[
                    ('/rm_robot/camera/image', '/camera/image_raw'),
                    ('/red_robot/scan', '/scan'),
                    ('/red_robot/odometry', '/odom'),
                ],
                output='screen'
            )
        ]
    )

    # 6. 雷达静态坐标系广播（消除 RViz2 黄色感叹号）
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
        set_gz_resource_env,
        set_ign_resource_env,
        gazebo,
        bridge,
        spawn_robot,
        spawn_target,
        static_tf_lidar,
    ])
