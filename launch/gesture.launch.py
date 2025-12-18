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

    # 2. 启动 导航系统 (加载地图、TF 坐标变换和 Nav2)
    # 这一步很关键，因为它通常包含了机器人定位和传感器数据的处理
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(pkg_share, 'launch', 'navigation.launch.py'))
    )

    # 3. 启动 手势识别控制节点 (gesture_control)
    # 延迟 2 秒启动，确保 Gazebo 已经加载出机器人模型
    gesture_control_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='my_robot_bringup',
                executable='gesture_control',
                output='screen',
                parameters=[{'use_sim_time': True}]
            )
        ]
    )

    return LaunchDescription([
        gazebo_launch,
        navigation_launch,
        gesture_control_node
    ])