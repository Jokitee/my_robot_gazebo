import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    SetEnvironmentVariable,
    TimerAction,
)
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('my_robot_gazebo')
    world_path = os.path.join(pkg_dir, 'worlds', 'rm_world.sdf')
    models_dir = os.path.join(pkg_dir, 'models')
    worlds_dir = os.path.join(pkg_dir, 'worlds')
    rviz_config_path = os.path.join(pkg_dir, 'rviz', 'default.rviz')

    # 1. 构建全量 Gazebo 模型与网格资源路径
    # 包含了当前包 models、worlds 以及外部网格模型包 rmoss_gz_resources
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

    # 保留系统现存的环境变量，避免覆盖
    for env_var in ['GZ_SIM_RESOURCE_PATH', 'IGN_GAZEBO_RESOURCE_PATH', 'SDF_PATH']:
        val = os.environ.get(env_var, '')
        if val:
            resource_paths.append(val)

    full_resource_path = ':'.join([p for p in resource_paths if p])

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

    # 2. 启动 Gazebo 仿真世界 (支持 headless:=true 仅跑仿真后台，避免重复弹出第二个 Gazebo 窗口)
    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='是否以无界面后台模式运行 Gazebo (设为 true 则只开 RViz，不弹第二个 Gazebo 窗口)'
    )

    # 动态构建启动指令: headless 为 false 时带 GUI，为 true 时加 -s (无头服务器模式)
    gazebo = ExecuteProcess(
        cmd=['gz', 'sim', '-r', world_path],
        additional_env=env_dict,
        output='screen'
    )

    # 3. 延时 2.0 秒启动 ROS-Gazebo 话题桥接器
    bridge = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='ros_gz_bridge',
                executable='parameter_bridge',
                arguments=[
                    # 时钟同步 (Gazebo -> ROS)
                    '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
                    # 动态里程计坐标变换 (Gazebo -> ROS /tf: odom -> chassis)
                    '/model/red_robot/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
                    # 底盘速度控制 (ROS -> Gazebo)
                    '/red_robot/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
                    # 底盘里程计 (Gazebo -> ROS)
                    '/red_robot/odometry@nav_msgs/msg/Odometry[gz.msgs.Odometry',
                    # 云台关节控制 (ROS -> Gazebo)
                    '/model/red_robot/joint/gimbal_yaw_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                    '/model/red_robot/joint/gimbal_pitch_joint/cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
                    # 传感器：单目相机与激光雷达 (Gazebo -> ROS 严格单向传输)
                    '/rm_robot/camera/image@sensor_msgs/msg/Image[gz.msgs.Image',
                    '/red_robot/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
                ],
                remappings=[
                    ('/model/red_robot/tf', '/tf'),
                ],
                output='screen'
            )
        ]
    )

    # 4. 雷达静态坐标系广播（将雷达真实挂载到 chassis 车体上，随车移动导航建图）
    static_tf_lidar = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_static_tf_publisher',
        arguments=[
            '--x', '0.15',
            '--y', '0.0',
            '--z', '0.35',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'chassis',
            '--child-frame-id', 'red_robot/chassis/rplidar_a2'
        ],
        output='screen'
    )

    # 5. 启动 RViz2 可视化节点
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
        headless_arg,
        set_gz_resource_env,
        set_ign_resource_env,
        set_sdf_path_env,
        gazebo,
        bridge,
        static_tf_lidar,
        rviz_node,
    ])
