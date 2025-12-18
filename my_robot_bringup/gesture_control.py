import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped # 🟢 增加初始位姿消息
from std_msgs.msg import String
import cv2
import mediapipe as mp
import math # 🟢 计算角度需要
from mediapipe.python.solutions import hands, drawing_utils

class GestureControl(Node):
    def __init__(self):
        super().__init__('gesture_control')
        # 1. 发布者设置
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        
        # 2. 订阅雷达
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        
        # 3. 摄像头与 MediaPipe
        self.cap = cv2.VideoCapture(0)
        self.mp_hands = hands
        self.hands = self.mp_hands.Hands(max_num_hands=1, min_detection_confidence=0.7)
        self.mp_draw = drawing_utils
        
        self.min_front_dist = 10.0
        
        # 🟢 关键：创建一个定时器，启动 2 秒后自动发布一次初始位姿
        # 这能确保 AMCL 节点已经启动并准备好接收坐标
        self.init_timer = self.create_timer(2.0, self.set_initial_pose)
        
        # 4. 主控制循环
        self.timer = self.create_timer(0.05, self.timer_callback)

    def set_initial_pose(self):
        """完全复刻 mission_master 的初始化逻辑"""
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        
        # 使用你任务中定义的起始坐标
        msg.pose.pose.position.x = 4.0
        msg.pose.pose.position.y = -4.0
        
        yaw = 3.14 # 朝向
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        
        # 协方差矩阵（告诉系统定位很准确）
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        
        self.init_pose_pub.publish(msg)
        self.get_logger().info("已自动发布初始位姿: (4.0, -4.0, 3.14)")
        
        # 只发一次就够了，停掉这个定时器
        self.init_timer.cancel()

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:20] + msg.ranges[-20:]
        valid_ranges = [r for r in front_ranges if r > 0.05]
        if valid_ranges:
            self.min_front_dist = min(valid_ranges)

    def timer_callback(self):
        success, frame = self.cap.read()
        if not success: return

        frame = cv2.flip(frame, 1)
        results = self.hands.process(cv2.cvtColor(frame, cv2.COLOR_BGR2RGB))
        
        twist = Twist()
        cmd_text = "NONE"

        if results.multi_hand_landmarks:
            for hand_lms in results.multi_hand_landmarks:
                self.mp_draw.draw_landmarks(frame, hand_lms, self.mp_hands.HAND_CONNECTIONS)
                cnt = 0
                for tip, pip in zip([8, 12, 16, 20], [6, 10, 14, 18]):
                    if hand_lms.landmark[tip].y < hand_lms.landmark[pip].y: cnt += 1
                
                if cnt == 1:
                    twist.linear.x = 0.2
                    cmd_text = "FORWARD"
                elif cnt == 2:
                    twist.angular.z = 0.5
                    cmd_text = "LEFT"
                elif cnt == 3:
                    twist.angular.z = -0.5
                    cmd_text = "RIGHT"
                else:
                    cmd_text = "STOP"

        # 🛑 硬避障逻辑（无论导航系统怎么想，代码层强制刹车）
        if self.min_front_dist < 0.6 and twist.linear.x > 0:
            twist.linear.x = 0.0
            cmd_text = "STOP (OBSTACLE!)"
            cv2.putText(frame, "WALL AHEAD!", (150, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (0,0,255), 3)

        self.vel_pub.publish(twist)
        
        # 画面叠加状态
        cv2.putText(frame, f"CMD: {cmd_text}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        cv2.putText(frame, f"Dist: {self.min_front_dist:.2f}m", (10, 90), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 1)
        cv2.imshow("Gesture Control", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = GestureControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cap.release()
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()