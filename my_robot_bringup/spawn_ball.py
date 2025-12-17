#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
from geometry_msgs.msg import Pose
import cv2
import numpy as np
import random
import os
import math
import time

class BallSpawner(Node):
    def __init__(self):
        # 显式声明节点参数
        super().__init__(
            'ball_spawner',
            parameter_overrides=[
                rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)
            ]
        )
        
        # ================= 配置区域 =================
        # 1. 你的地图文件路径 (请修改为你实际的路径)
        self.map_path = "/home/wangzixi/me332_final_ws/src/my_robot_bringup/maps/my_maze_map.pgm"
        self.yaml_resolution = 0.05  # 地图分辨率 (对应 yaml 中的 resolution)
        self.yaml_origin = [-4.9, -4.88] # 地图原点 (对应 yaml 中的 origin 前两个数)
        
        # 2. 球的模型 XML (SDF格式)
        # 这里定义一个半径 0.2m 的红球
        self.ball_sdf = """
        <sdf version='1.6'>
          <model name='red_ball'>
            <pose>0 0 0.1 0 0 0</pose>
            <link name='ball'>
              <inertial>
                <mass>0.0056</mass> <inertia>
                  <ixx>2.37e-06</ixx><ixy>0</ixy><ixz>0</ixz>
                  <iyy>2.37e-06</iyy><iyz>0</iyz><izz>2.37e-06</izz>
                </inertia>
              </inertial>
              <visual name='visual'>
                <geometry>
                  <sphere><radius>0.0325</radius></sphere> </geometry>
                <material>
                  <script>
                    <uri>file://media/materials/scripts/gazebo.material</uri>
                    <name>Gazebo/RedBright</name>
                  </script>
                </material>
              </visual>
              <collision name='collision'>
                <geometry>
                  <sphere><radius>0.0325</radius></sphere> </geometry>
                <surface>
                  <friction>
                    <ode>
                      <mu>1.0</mu> <mu2>1.0</mu2>
                    </ode>
                  </friction>
                </surface>
              </collision>
            </link>
          </model>
        </sdf>
        """
        # ===========================================

        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

    def load_map(self):
        # 读取灰度图
        self.img = cv2.imread(self.map_path, cv2.IMREAD_GRAYSCALE)
        if self.img is None:
            self.get_logger().error(f"无法读取地图: {self.map_path}")
            return False
        self.height, self.width = self.img.shape
        return True

    def get_random_valid_pose(self):
        # 拒绝采样算法：无限循环直到找到一个合法点
        while True:
            # 1. 随机生成物理坐标 (假设迷宫大概在 -5 到 5 之间，你可以根据实际情况放大范围)
            x = random.uniform(-4.0, 4.0)
            y = random.uniform(-4.0, 4.0)
            
            # 2. 物理坐标 -> 像素坐标转换
            # pixel_x = (world_x - origin_x) / resolution
            px = int((x - self.yaml_origin[0]) / self.yaml_resolution)
            
            # pixel_y = height - (world_y - origin_y) / resolution
            # 注意：PGM图片通常坐标原点在左上角，而地图原点在左下角，所以Y轴要翻转
            py = int(self.height - (y - self.yaml_origin[1]) / self.yaml_resolution)

            # 3. 越界检查
            if px < 0 or px >= self.width or py < 0 or py >= self.height:
                continue

            # 4. 碰撞检查 (检测像素颜色)
            # 254/255 是白色(空地)，0 是黑色(墙)，205 是未知
            # 我们还可以检查这个点周围 5个像素 是否都是白色，防止生成在墙根底下
            pixel_val = self.img[py, px]
            
            # 简单的判断：只要够白就是空地
            if pixel_val > 250:
                self.get_logger().info(f"找到合法位置: ({x:.2f}, {y:.2f}) 像素值: {pixel_val}")
                return x, y

    def spawn(self):
        if not self.load_map():
            return

        # 1. 先尝试删除旧的球 (防止重复生成报错)
        self.get_logger().info("正在清理旧球...")
        del_req = DeleteEntity.Request()
        del_req.name = "red_ball"
        future = self.delete_client.call_async(del_req)
        # 这里不等结果，直接继续，因为如果没球删除也没关系

        time.sleep(1.0)  # <--- 加上这个！给 Gazebo 一秒钟喘息时间

        # 2. 找个新位置
        x, y = self.get_random_valid_pose()

        # 3. 生成新球
        req = SpawnEntity.Request()
        req.name = "red_ball"
        req.xml = self.ball_sdf
        req.initial_pose.position.x = x
        req.initial_pose.position.y = y
        req.initial_pose.position.z = 0.05

        self.get_logger().info("正在生成新球...")
        self.spawn_client.wait_for_service()
        future = self.spawn_client.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("✅ 成功！球已刷新！")
        else:
            self.get_logger().error("❌ 生成失败！")

def main(args=None):
    rclpy.init(args=args)
    spawner = BallSpawner()
    
    try:
        # 执行生成逻辑
        spawner.spawn()
        # 如果你的 spawn() 里面有异步操作(call_async)，可能需要短暂 spin 一下
        # 否则直接退出即可
    except KeyboardInterrupt:
        pass
    finally:
        # 记得销毁节点，释放资源
        spawner.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()