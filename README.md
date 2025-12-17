# ME332 机器人操作系统期末课程设计 - 移动操作机器人

## 1. 项目简介

本项目基于 ROS 2 Humble 开发，实现了一个具备自主导航、目标识别与移动操作能力的移动机器人系统 。

* **底盘系统**：300x300mm 以内的差速驱动底盘 。

* **机械臂系统**：具备 3 个及以上自由度的操作臂，支持三维空间操作 。

* **视觉系统**：基于 OpenCV 的红球识别与追踪。
  
* **导航系统**：集成 Nav2 导航框架，实现多点循环巡逻与 360 度环境扫描 。



---

## 2. 环境要求

* **系统版本**: Ubuntu 22.04 + ROS 2 Humble
* **仿真环境**: Gazebo 11
* **关键依赖**:
* `nav2_bringup`
* `gazebo_ros2_control`
* `cv_bridge`
* `sensor_msgs`



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

