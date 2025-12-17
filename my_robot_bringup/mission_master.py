#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point, Twist, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import time
import math
from geometry_msgs.msg import PoseWithCovarianceStamped

class MissionMaster(Node):
    def __init__(self):
        super().__init__('mission_master')

        self.init_pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/initialpose', 10)
        time.sleep(2.0) 
        self.set_initial_pose()

        self.set_parameters([rclpy.parameter.Parameter('use_sim_time', rclpy.Parameter.Type.BOOL, True)])
        
        self.publisher_vel = self.create_publisher(Twist, 'cmd_vel', 10)
        self.subscription = self.create_subscription(Point, '/detected_ball', self.ball_callback, 10)
        
        self.nav = BasicNavigator()
        
        # 初始化位置
        initial_pose = PoseStamped()
        initial_pose.header.frame_id = 'map'
        initial_pose.header.stamp = self.nav.get_clock().now().to_msg()
        initial_pose.pose.position.x = 4.0
        initial_pose.pose.position.y = -4.0
        initial_pose.pose.orientation.z = 1.0 
        initial_pose.pose.orientation.w = 0.0
        
        self.get_logger().info("正在初始化机器人位置并启动 Nav2...")
        self.nav.setInitialPose(initial_pose)
        self.nav.waitUntilNav2Active() 
        self.get_logger().info("导航系统已就绪！")

        self.state = "PATROL" 
        self.last_ball_time = 0
        self.found_logged = False # 防止重复打印坐标的标志位
        
        #随便设了几个巡逻点
        self.patrol_points = [
            [2.0, 0.0, 0.0], 
            [2.0, 2.0, 1.57], 
            [0.0, 0.0, 3.14] 
        ]
        self.current_patrol_index = 0

        self.scan_start_time = None
        self.rotate_speed = 0.8 
        
        self.timer = self.create_timer(0.1, self.control_loop)

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

    def ball_callback(self, msg):
        self.ball_x = msg.x      
        self.ball_radius = msg.y 
        self.last_ball_time = time.time()
        
        if self.state in ["PATROL", "SCAN"]:
            self.get_logger().info("!!! 视觉发现目标 !!!")
            self.nav.cancelTask()
            self.state = "CHASE"
            self.found_logged = False # 重置打印标志

    def control_loop(self):
        if self.state == "CHASE" and (time.time() - self.last_ball_time > 3.0):
            self.get_logger().info("丢失目标，恢复巡逻模式...")
            self.state = "PATROL"

        if self.state == "PATROL":
            if not self.nav.isTaskComplete():
                return 
            self.get_logger().info(f"到达巡逻点 {self.current_patrol_index}，开始自转扫描...")
            self.state = "SCAN"
            self.scan_start_time = time.time()

        elif self.state == "SCAN":
            scan_duration = (2 * math.pi) / self.rotate_speed
            if time.time() - self.scan_start_time < scan_duration:
                cmd = Twist()
                cmd.angular.z = self.rotate_speed
                self.publisher_vel.publish(cmd)
            else:
                self.get_logger().info("扫描完成，前往下一个点。")
                self.current_patrol_index = (self.current_patrol_index + 1) % len(self.patrol_points)
                self.send_nav_goal()
                self.state = "PATROL"

        # 🟢 修改后的 CHASE 逻辑：停止并显示坐标
        elif self.state == "CHASE":
            # 1. 立即停止所有运动
            stop_cmd = Twist()
            stop_cmd.linear.x = 0.0
            stop_cmd.angular.z = 0.0
            self.publisher_vel.publish(stop_cmd)

            # 2. 获取并打印当前坐标
            if not self.found_logged:
                # 获取机器人当前在 map 坐标系下的位置
                robot_pose = self.nav.getRobotPose()
                px = robot_pose.pose.position.x
                py = robot_pose.pose.position.y
                
                self.get_logger().info("*****************************************")
                self.get_logger().info(f"找到球了！发现位置坐标: X={px:.3f}, Y={py:.3f}")
                self.get_logger().info("*****************************************")
                
                self.found_logged = True # 标记为已打印

    def send_nav_goal(self):
        target_data = self.patrol_points[self.current_patrol_index]
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.nav.get_clock().now().to_msg()
        goal_pose.pose.position.x = target_data[0]
        goal_pose.pose.position.y = target_data[1]
        yaw = target_data[2]
        goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
        goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
        self.nav.goToPose(goal_pose)

def main(args=None):
    rclpy.init(args=args)
    node = MissionMaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()