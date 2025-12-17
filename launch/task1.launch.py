import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_share = get_package_share_directory('my_robot_bringup')

    # 1. 启动 Gazebo 和 机器人模型 (包括控制器)
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'gazebo.launch.py'))
    )

    # 2. 启动 导航系统 (加载地图和 Nav2)
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py'))
    )

    # 3. 启动 视觉识别节点 (ball_detector)
    ball_detector_node = Node(
        package='my_robot_bringup',
        executable='ball_detector',
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    # 4. 启动 任务主控节点 (mission_master)
    # 延迟 5 秒启动，确保导航系统已经初始化完毕
    mission_master_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='my_robot_bringup',
                executable='mission_master',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        navigation_launch,
        ball_detector_node,
        mission_master_node
    ])