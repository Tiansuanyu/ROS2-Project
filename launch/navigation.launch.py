import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_name = 'my_robot_bringup'
    pkg_share = get_package_share_directory(pkg_name)
    
    # 指向你刚刚保存好的地图
    map_file = os.path.join(pkg_share, 'maps', 'my_maze_map.yaml')
    
    # Nav2 官方启动文件路径
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    return LaunchDescription([
        # 启动 Nav2 (加载地图、定位AMCL、路径规划、代价地图)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'bringup_launch.py')
            ),
            launch_arguments={
                'map': map_file,
                'use_sim_time': 'true',
                'params_file': os.path.join(nav2_bringup_dir, 'params', 'nav2_params.yaml'),
            }.items(),
        ),
    ])