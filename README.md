# ME332 机器人操作系统期末课程设计 - 移动操作机器人

## 1. 项目简介

本项目基于 ROS 2 Humble 开发，实现了一个具备自主导航、目标识别与移动操作能力的移动机器人系统 。

* **底盘系统**：300x300mm 以内的差速驱动底盘 。

* **机械臂系统**：具备 3 个及以上自由度的操作臂，支持三维空间操作 。

* **视觉系统**：基于 OpenCV 的红球识别与追踪。
  
* **导航系统**：集成 Nav2 导航框架，实现多点循环巡逻与 360 度环境扫描 。



---

## 2. 环境要求

* **操作系统**: Ubuntu 22.04 LTS
* **ROS 版本**: ROS 2 Humble
* **仿真器**: Gazebo 11
* **主要依赖**:
    - `nav2_bringup` (导航框架)
    - `gazebo_ros2_control` (硬件接口)
    - `cv_bridge` & `opencv-python` (视觉处理)


---

## 3. 安装与编译

```bash
# 克隆仓库
cd ~/me332_final_ws/src
git clone https://github.com/Tiansuanyu/ROS2-Project.git
# 编译工作空间
cd ~/me332_final_ws
colcon build --symlink-install
source install/setup.bash

```

---

## 4. 运行指南 (一键启动)

本项目已整合 Launch 文件，支持一键启动完整仿真环境：

```bash
# 启动 Gazebo、Nav2、视觉节点及任务主控
ros2 launch my_robot_bringup task1.launch.py

```

### 交互操作

* **刷球脚本**：在另一个终端运行，随机在迷宫中生成红球目标。
```bash
ros2 run my_robot_bringup spawn_ball

```

## 5. 文件结构说明
```text
my_robot_bringup/
├── config/
│   ├── arm_controllers.yaml    # 机械臂 PID 增益与控制器配置
│   └── nav2_params.yaml        # 导航膨胀层与局部路径规划参数
├── launch/
│   ├── gazebo.launch.py        # 启动 Gazebo、加载模型与控制器
│   ├── navigation.launch.py    # 启动 Nav2、AMCL 与地图服务器
│   └── task1.launch.py         # 【总控】一键启动完整任务流
├── maps/
│   └── my_maze_map.yaml        # 迷宫环境地图
├── my_robot_bringup/
│   ├── ball_detector.py        # 红色目标视觉识别节点（OpenCV）
│   ├── mission_master.py       # 巡逻与扫描逻辑状态机
│   └── spawn_ball.py           # 目标随机生成脚本
└── urdf/
    ├── mobile_manipulator.urdf.xacro  # 机器人描述文件
    └── my_robot.gazebo.xacro          # Gazebo 插件配置

```
