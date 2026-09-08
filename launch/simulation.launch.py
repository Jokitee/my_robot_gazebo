import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_my_robot = get_package_share_directory('my_robot_gazebo')
    pkg_rm_res = get_package_share_directory('rmoss_gz_resources')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    # 1. 关键：设置 Gazebo 模型搜索路径，让它能找到装甲板、贴纸和车身 mesh
    models_path = os.path.join(pkg_my_robot, 'models') + ':' + \
                  os.path.join(pkg_rm_res, 'resource', 'models')
    set_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models_path)

    # 2. 获取你的机器人模型文件路径
    # (如果你的文件夹叫 rm_robot 就用 rm_robot/model.sdf；如果叫 armor_target 就填对应文件夹名)
    robot_sdf_path = os.path.join(pkg_my_robot, 'models', 'rm_robot', 'model.sdf')

    # 3. 启动 Gazebo Harmonic 仿真环境
    gazebo_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-r empty.sdf'}.items(),
    )

    # 4. 生成我方自瞄机器人（红车，带相机，生成在原点）
    spawn_red_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf_path,
            '-name', 'red_robot',
            '-x', '0.0', '-y', '0.0', '-z', '0.15'
        ],
        output='screen'
    )

    # 5. 生成敌方靶车（蓝车，正对红车 2.5 米处）
    spawn_blue_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', robot_sdf_path,
            '-name', 'blue_robot',
            '-x', '2.5', '-y', '0.0', '-z', '0.15',
            '-Y', '3.14159'  # 旋转 180 度，正对红方车头
        ],
        output='screen'
    )

    # 6. ROS 2 <-> Gazebo 话题桥接器 (ros_gz_bridge)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            # 桥接红方底盘速度控制
            '/red_robot/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist',
            # 桥接机器人位姿为 TF
            '/model/red_robot/pose@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        ],
        remappings=[
            ('/model/red_robot/pose', '/tf')
        ],
        output='screen'
    )

    # 7. 使用 rmoss_gz_cam 把 Gazebo 相机转成 ROS 2 标准图像话题 /camera/image_raw
    gz_cam_node = Node(
        package='rmoss_gz_cam',
        executable='gz_cam',
        parameters=[{
            'gz_camera_image_topic': '/rm_robot/camera/image',
            'camera_name': 'camera',
            'frame_id': 'camera_link',
            'fps': 30
        }],
        output='screen'
    )

    # 8. 启动 RViz2
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen'
    )

    return LaunchDescription([
        set_resource_path,
        gazebo_sim,
        spawn_red_robot,
        spawn_blue_robot,
        bridge,
        gz_cam_node,
        rviz_node
    ])
