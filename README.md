# ME332 机器人操作系统期末课程设计 - 移动操作机器人

## 1. 项目简介

本项目基于 ROS 2 Humble 开发，实现了一个具备自主导航、目标识别与移动操作能力的移动机器人系统 。

* **底盘系统**：300x300mm 以内的差速驱动底盘 。
* **机械臂系统**：具备 3 个及以上自由度的操作臂，支持三维空间操作 。
* **视觉系统**：基于 OpenCV 的红球识别与追踪。
* **导航系统**：集成 Nav2 导航框架，实现多点循环巡逻与 360 度环境扫描 。
* **交互系统 (加分项)**：
    * **手势控制**：基于 MediaPipe 识别手势指令，并带有视觉监控反馈。
    * **语音控制**：基于百度 API 的自然语言控制，支持前进/左转/右转/停止。
    * **自动定位**：交互节点启动后会自动初始化位姿，解决 AMCL 启动卡死问题。
    * **硬避障**：交互控制链路中集成激光雷达数据校验，实现遇障自动拦截。

---

## 2. 环境要求

* **操作系统**: Ubuntu 22.04 LTS
* **ROS 版本**: ROS 2 Humble
* **仿真器**: Gazebo 11
* **主要依赖**:
    - `nav2_bringup` (导航框架)
    - `gazebo_ros2_control` (硬件接口)
    - `cv_bridge` & `opencv-python` (视觉处理)
* **Python 依赖包**:
    * `mediapipe==0.10.11`
    * `baidu-aip`
    * `SpeechRecognition`
    * `numpy<2` (由于 ROS Humble 的 cv_bridge 暂不支持 NumPy 2.x)


---

## 3. 安装与编译

```bash
# 1. 安装项目新增依赖
pip install mediapipe baidu-aip SpeechRecognition "numpy<2"

# 2. 克隆与编译
cd ~/me332_final_ws/src
git clone [https://github.com/Tiansuanyu/ROS2-Project.git](https://github.com/Tiansuanyu/ROS2-Project.git)
cd ~/me332_final_ws
colcon build --symlink-install
source install/setup.bash
```

---

## 4. 运行指南 (一键启动)

本项目已整合 Launch 文件，支持一键启动完整仿真环境：


### 4.1 任务一：自主巡逻与目标生成 (一键启动)

```bash
# 启动 Gazebo、Nav2、视觉节点及任务主控
ros2 launch my_robot_bringup task1.launch.py

# 在另一个终端运行刷球脚本
ros2 run my_robot_bringup spawn_ball
```

### 4.2 任务二：手势识别交互控制 (加分项)

该模式会启动笔记本摄像头，并通过手势控制仿真机器人。节点会自动初始化位姿。

```bash
ros2 launch my_robot_bringup gesture.launch.py
```

### 4.3 任务三：语音识别交互控制 (加分项)

该模式支持通过麦克风输入普通话指令（前进、向左、向右、停止）。

```bash
ros2 launch my_robot_bringup voice.launch.py

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
│   ├── arm_controllers.yaml    # 机械臂控制器配置
│   └── nav2_params.yaml        # 导航代价地图与路径规划参数
├── launch/
│   ├── gazebo.launch.py        # 加载机器人模型与仿真环境
│   ├── navigation.launch.py    # 启动定位与 Nav2 栈
│   ├── task1.launch.py         # 启动自主巡逻任务流
│   ├── gesture_launch.py       # 手势控制模式入口
│   └── voice.launch.py         # 语音控制模式入口
├── maps/
│   ├── my_maze_map.yaml        # 地图描述
│   └── my_maze_map.pgm         # 地图图片
├── my_robot_bringup/
│   ├── ball_detector.py        # 视觉识别节点
│   ├── gesture_control.py      # 手势识别与避障控制节点
│   ├── mission_master.py       # 巡逻任务主控节点
│   ├── spawn_ball.py           # 红球随机生成脚本
│   └── voice_control.py        # 语音识别与避障控制节点
└── urdf/
    ├── mobile_manipulator.urdf.xacro  # 机器人 URDF 描述文件
    └── my_robot.gazebo.xacro          # Gazebo 传感器插件配置
```
