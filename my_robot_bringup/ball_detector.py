#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from geometry_msgs.msg import Point

class BallDetector(Node):
    def __init__(self):
        super().__init__('ball_detector')
        
        # 1. 订阅 Gazebo 的摄像头话题
        self.subscription = self.create_subscription(
            Image,
            '/camera1/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()
        
        # 2. 设定红色的 HSV 阈值
        self.lower_red1 = np.array([0, 100, 100])
        self.upper_red1 = np.array([10, 255, 255])
        self.lower_red2 = np.array([160, 100, 100])
        self.upper_red2 = np.array([180, 255, 255])

        self.get_logger().info("视觉节点已启动（无窗口模式），正在寻找红球...")

        # 3. 创建发布者，话题叫 /detected_ball
        self.publisher_ = self.create_publisher(Point, '/detected_ball', 10)

    def image_callback(self, msg):
        try:
            # 将 ROS 图像转为 OpenCV 格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # 图像处理：BGR -> HSV
            hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
            
            # 提取红色区域 (Mask)
            mask1 = cv2.inRange(hsv, self.lower_red1, self.upper_red1)
            mask2 = cv2.inRange(hsv, self.lower_red2, self.upper_red2)
            mask = mask1 + mask2
            
            # 腐蚀与膨胀 (去除噪点)
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.erode(mask, kernel, iterations=1)
            mask = cv2.dilate(mask, kernel, iterations=2)
            
            # 寻找轮廓
            contours, _ = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
            
            if len(contours) > 0:
                # 找到最大的那个轮廓
                c = max(contours, key=cv2.contourArea)
                ((x, y), radius) = cv2.minEnclosingCircle(c)
                
                # 半径大于10像素才发布
                if radius > 10:
                    point_msg = Point()
                    point_msg.x = float(x)        
                    point_msg.y = float(radius)   
                    point_msg.z = 1.0             
                    
                    self.publisher_.publish(point_msg)
                    # 仅在后台记录日志，不弹窗
                    self.get_logger().info(f"检测到红球: 画面X={x:.1f}, 半径={radius:.1f}")

        except Exception as e:
            self.get_logger().error(f"图像处理出错: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = BallDetector()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()