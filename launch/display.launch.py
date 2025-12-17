import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # 1. 设置包名和文件路径
    package_name = 'my_robot_bringup'
    urdf_file_name = 'my_robot.urdf.xacro'
    
    # 获取包的安装路径
    pkg_share = get_package_share_directory(package_name)
    # 拼接 URDF 文件的完整路径
    urdf_path = os.path.join(pkg_share, 'urdf', urdf_file_name)
    # 拼接 Rviz 配置文件的路径 (稍后我们会保存这个文件)
    rviz_config_path = os.path.join(pkg_share, 'rviz', 'urdf_config.rviz')

    # 2. 处理 Xacro 文件
    # 使用 xacro 命令将 .xacro 文件转换为纯 URDF 内容
    robot_description_content = Command(['xacro ', urdf_path])
    
    # 创建 robot_description 参数
    robot_description = {'robot_description': robot_description_content}

    # 3. 定义节点
    
    # robot_state_publisher: 发布机器人的 TF 坐标树
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # joint_state_publisher_gui: 弹出一个小窗口，让你能拖动滑块控制关节
    node_joint_state_publisher_gui = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    # rviz2: 可视化工具
    node_rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        # 如果你还没有保存过 .rviz 配置文件，可以先注释掉下面这行 arguments
        # --- 关键修改在这里：加载配置文件 ---
        arguments=['-d', rviz_config_path] 
    )

    # 4. 返回 LaunchDescription
    return LaunchDescription([
        node_robot_state_publisher,
        node_joint_state_publisher_gui,
        node_rviz
    ])