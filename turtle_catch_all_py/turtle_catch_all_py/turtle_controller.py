#!usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import TurtleName
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist
from turtlesim.srv import Kill
from functools import partial

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        #self.pose :Pose = None
        self.target_x = 0.0
        self.target_y = 0.0
        self.theta = 0.0
        self.name = ""
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)

        # FIX: Subscribe to Pose, not Spawn, on /spawn_pose
        self.receive_pose_ = self.create_subscription(Pose, "/spawn_pose", self.store_data, 10)
        self.turtle_name_sub_ = self.create_subscription(TurtleName, "/tturtle_name", self.collect_name, 10)

        self.kill_service_client_ = self.create_client(Kill, "/kill")

        self.kill_timer_ = self.create_timer(2.50, self.kill_crossed_turtle)
        self.controll_timer_ = self.create_timer(1.0, self.control_loop)

    def pose_callback(self, pose: Pose):
        self.pose = pose

    # FIX: store_data expects a Pose message
    def store_data(self, pose: Pose):
        self.target_x = pose.x
        self.target_y = pose.y
        self.theta = pose.theta
        # self.name = pose.name  # Pose does not have name, so remove or set name elsewhere
        self.get_logger().info(f"Target position has updated to: x={self.target_x}, y={self.target_y}, theta={self.theta}")

    def control_loop(self):
        if self.pose is None:
            return
        
        dist_x = self.target_x - self.pose.x
        dist_y = self.target_y - self.pose.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        cmd = Twist()
        if distance > 0.5:
            cmd.linear.x = 2 * distance
            angle_theta = math.atan2(dist_y, dist_x)
            angle_diff = angle_theta - self.pose.theta
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            cmd.angular.z = 2 * angle_diff
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0

        self.get_logger().info(f"Publishing cmd_vel: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}")
        self.cmd_vel_pub_.publish(cmd)

    def collect_name(self, TurtleName):
        self.name = TurtleName.turtle_name
        self.get_logger().info(f"Received: {self.name}")

    def kill_crossed_turtle(self):
        if not self.kill_service_client_.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn("Kill service not available!")
            return
        if not self.name:
            self.get_logger().warn("No turtle name set, cannot call kill service.")
            return
        request = Kill.Request()
        request.name = self.name  # self.name must be set elsewhere
        future = self.kill_service_client_.call_async(request)
        future.add_done_callback(partial(self.kill_service_callback, request=request))

    def kill_service_callback(self, future, request):
        self.get_logger().info(f"Killed {request.name}.")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


