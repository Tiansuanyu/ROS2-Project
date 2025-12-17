import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import xacro

def generate_launch_description():
    pkg_name = 'my_robot_bringup'
    file_subpath = 'urdf/my_robot.urdf.xacro'

    # 1. 处理 Xacro 文件
    xacro_file = os.path.join(get_package_share_directory(pkg_name), file_subpath)
    robot_description_raw = xacro.process_file(xacro_file).toxml()

    # 1.1 地图路径
    world_file_path = os.path.join(get_package_share_directory(pkg_name), 'worlds', 'maze.world')
    
    # 2. Gazebo (加载地图)
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(
            get_package_share_directory('gazebo_ros'), 'launch', 'gazebo.launch.py')]),
        launch_arguments={'world': world_file_path}.items()
    )

    # 3. 生成机器人 (Spawn Entity)
    spawn_entity = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=['-topic', 'robot_description',
                   '-entity', 'my_robot',
                   '-x', '4.0', '-y', '-4.0', '-z', '0.05', '-Y', '3.14'],
        output='screen'
    )

    # 4. Robot State Publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_raw,
                     'use_sim_time': True}]
    )

    # ================= 🔴 新增：控制器加载逻辑 🔴 =================
    
    # 5. 加载关节状态广播器 (Joint State Broadcaster)
    # 作用：从 Gazebo 获取关节状态并发布到 /joint_states 话题，驱动 TF 树
    load_joint_state_broadcaster = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
        output="screen",
    )

    # 6. 加载机械臂位置控制器 (Joint Trajectory Controller)
    # 作用：接收目标位置指令，通过 PID 控制关节转动到指定角度
    # 注意：这里的名称 "arm_controller" 必须匹配 yaml 文件中的定义
    load_arm_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller"], 
        output="screen",
    )

    # 7. 严格的启动顺序控制
    # 只有当 机器人生成完成(spawn_entity退出) 后，才加载 joint_state_broadcaster
    delay_joint_state_broadcaster = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_entity,
            on_exit=[load_joint_state_broadcaster],
        )
    )

    # 只有当 joint_state_broadcaster 加载完成后，才加载 arm_controller
    # 这样可以防止控制器在关节状态还没准备好时报错
    delay_arm_controller = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=load_joint_state_broadcaster,
            on_exit=[load_arm_controller],
        )
    )

    return LaunchDescription([
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        delay_joint_state_broadcaster, # 注意这里返回的是带延时的事件
        delay_arm_controller
    ])