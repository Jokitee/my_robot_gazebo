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
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'default.rviz')

    # 1. 构建完整的 Gazebo 资源路径（包含 rmoss_gz_resources，机器人的轮子、云台、装甲板网格全在此处）
    resource_paths = [pkg_dir, models_dir, worlds_dir]
    try:
        rmoss_res_dir = os.path.join(
            get_package_share_directory('rmoss_gz_resources'),
            'resource',
            'models'
        )
        if os.path.exists(rmoss_res_dir):
            resource_paths.append(rmoss_res_dir)
    except Exception:
        pass

    # 保留系统现存的环境变量，避免冲掉 source 进来的环境变量
    for env_var in ['GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'SDF_PATH']:
        val = os.environ.get(env_var, '')
        if val:
            resource_paths.append(val)

    full_resource_path = ':'.join(resource_paths)

    set_gz_resource_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=full_resource_path
    )
    set_ign_resource_env = SetEnvironmentVariable(
        name='IGN_GAZEBO_RESOURCE_PATH',
        value=full_resource_path
    )
    set_sdf_path_env = SetEnvironmentVariable(
        name='SDF_PATH',
        value=full_resource_path
    )

    env_dict = {
        'GZ_SIM_RESOURCE_PATH': full_resource_path,
        'IGN_GAZEBO_RESOURCE_PATH': full_resource_path,
        'SDF_PATH': full_resource_path
    }

    # 2. 启动 Gazebo 仿真世界 (-r 确保物理引擎直接运行，显式传递环境变量字典)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        additional_env=env_dict,
        output='screen'
    )

    # 3. 延时 4.0 秒生成红方步兵机器人（此时所有网格路径就位且服务已准备好）
    spawn_robot = TimerAction(
        period=4.0,
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

    # 4. 延时 6.0 秒生成装甲板靶标（错开 2 秒，避免并发死锁）
    spawn_target = TimerAction(
        period=6.0,
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

    # 6. 雷达静态坐标系广播（采用 ROS 2 新式参数格式，消除老语法警告）
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_publisher',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.35',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'world',
            '--child-frame-id', 'red_robot/chassis/rplidar_a2'
        ],
        output='screen'
    )

    # 7. 启动 RViz2 可视化工具
    rviz_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='rviz2',
                executable='rviz2',
                name='rviz2',
                arguments=['-d', rviz_config_path],
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        set_gz_resource_env,
        set_ign_resource_env,
        set_sdf_path_env,
        gazebo,
        bridge,
        static_tf_lidar,
        rviz_node,
        spawn_robot,
        spawn_target,
    ])
