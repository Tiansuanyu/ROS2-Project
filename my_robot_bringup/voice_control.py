import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist, PoseWithCovarianceStamped
import speech_recognition as sr
from aip import AipSpeech  # 🟢 使用百度官方 SDK
import math
import time

class BaiduVoiceControl(Node):
    def __init__(self):
        super().__init__('voice_control')
        
        # ================= 百度 API 配置 =================
        APP_ID = '7344470'
        API_KEY = 'pMIPbfZSew9D263p8FglHmKW'
        SECRET_KEY = 'ldXzv3lXLLgEuymp0M23pa5K3WgFdU1p'
        self.client = AipSpeech(APP_ID, API_KEY, SECRET_KEY)
        # ===============================================

        self.recognizer = sr.Recognizer()
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)

        self.min_front_dist = 10.0
        self.is_listening = False 

        # 初始定位定时器
        self.init_timer = self.create_timer(2.5, self.set_initial_pose)
        # 语音循环定时器
        self.main_timer = self.create_timer(1.0, self.main_loop)
        self.get_logger().info("【语音节点】官方 SDK 版已启动，准备定位...")

    def set_initial_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.pose.pose.position.x = 4.0
        msg.pose.pose.position.y = -4.0
        yaw = 3.14
        msg.pose.pose.orientation.z = math.sin(yaw / 2)
        msg.pose.pose.orientation.w = math.cos(yaw / 2)
        msg.pose.covariance[0] = 0.25
        msg.pose.covariance[7] = 0.25
        msg.pose.covariance[35] = 0.068
        self.init_pose_pub.publish(msg)
        self.get_logger().info("已发送初始位姿 (4.0, -4.0)，激活 AMCL 定位")
        self.init_timer.cancel()

    def scan_callback(self, msg):
        front_ranges = msg.ranges[0:20] + msg.ranges[-20:]
        valid_ranges = [r for r in front_ranges if r > 0.05]
        if valid_ranges:
            self.min_front_dist = min(valid_ranges)

    def main_loop(self):
        if not self.is_listening:
            self.listen_voice()

    def listen_voice(self):
        self.is_listening = True
        try:
            with sr.Microphone() as source:
                self.get_logger().info(">>> 正在倾听，请下令 (前进/左转/右转/停止)...")
                self.recognizer.adjust_for_ambient_noise(source, duration=0.5)
                audio = self.recognizer.listen(source, timeout=3, phrase_time_limit=4)
                
                # 🟢 将音频转换为百度需要的 PCM 格式
                pcm_data = audio.get_wav_data(convert_rate=16000, convert_width=2)
                
                self.get_logger().info("正在上传百度云...")
                # 1537 代表普通话输入
                result = self.client.asr(pcm_data, 'pcm', 16000, {'dev_pid': 1537})
                
                if result['err_no'] == 0:
                    text = result['result'][0]
                    self.get_logger().info(f"【识别成功】: {text}")
                    self.process_text(text)
                else:
                    self.get_logger().error(f"识别失败: {result['err_msg']}")
                
        except Exception as e:
            self.get_logger().error(f"语音链路故障: {e}")
            time.sleep(1.0)
        finally:
            self.is_listening = False

    def process_text(self, text):
        twist = Twist()
        action = "STOP"
        if any(word in text for word in ["前进", "直走", "往前"]):
            if self.min_front_dist < 0.7:
                self.get_logger().warn("避障警告：离墙太近，无法前进")
            else:
                twist.linear.x = 0.3
                action = "FORWARD"
        elif any(word in text for word in ["左转", "向左"]):
            twist.angular.z = 0.6
            action = "LEFT"
        elif any(word in text for word in ["右转", "向右"]):
            twist.angular.z = -0.6
            action = "RIGHT"
        elif any(word in text for word in ["停", "停止"]):
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            action = "STOP"
        
        self.vel_pub.publish(twist)
        self.get_logger().info(f"执行动作: {action}")

def main(args=None):
    rclpy.init(args=args)
    node = BaiduVoiceControl()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()