#!usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from functools import partial
from geometry_msgs.msg import Twist

from turtlesim.msg import Pose
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from my_robot_interfaces.srv import CatchTurtle

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.turtles_to_catch_: Turtle = None
        self.pose_: Pose = None
        self.catch_closest_turtle_first_ = True
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10) 
        
        self.controll_timer_ = self.create_timer(0.1, self.control_loop)

        self.alive_turtles_subscriber_ = self.create_subscription(
            TurtleArray, "/alive_turtles", self.callback_alive_turtles, 10)
        
        self.catch_turtle_client_ = self.create_client(CatchTurtle, "/catch_turtle")
        
    def pose_callback(self, pose: Pose):
        self.pose_ = pose

    def callback_alive_turtles(self, msg: TurtleArray):
        if len(msg.turtles) > 0:
            if self.catch_closest_turtle_first_:
                closet_turtle = None
                closest_turtle_distance = None

                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)

                    if closest_turtle_distance is None or distance < closest_turtle_distance:
                        closest_turtle_distance = distance
                        closet_turtle = turtle
                self.turtles_to_catch_ = closet_turtle
            else:
                self.turtles_to_catch_ = msg.turtles[0]

    def control_loop(self):
        if self.pose_ is None or self.turtles_to_catch_ is None:
            return
        
        dist_x = self.turtles_to_catch_.x - self.pose_.x
        dist_y = self.turtles_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        cmd = Twist()

        if distance > 0.1:
            #Position
            cmd.linear.x = 2 * distance
            #Orientation
            angle_theta = math.atan2(dist_y, dist_x)
            angle_diff = angle_theta - self.pose_.theta
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi
            cmd.angular.z = 6 * angle_diff
        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            self.call_catch_turtle_service(self.turtles_to_catch_.name)
            self.turtles_to_catch_ = None

        #self.get_logger().info(f"Publishing cmd_vel: linear.x={cmd.linear.x}, angular.z={cmd.angular.z}")
        self.cmd_vel_pub_.publish(cmd)

    def call_catch_turtle_service(self, turtle_name):
        while not self.catch_turtle_client_.wait_for_service(1.0):
            self.get_logger().info("Waiting for catch turtle service...")
        
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = self.catch_turtle_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_catch_turtle_service, turtle_name = turtle_name))

    def callback_catch_turtle_service(self, future, turtle_name):
        response: CatchTurtle.Response = future.result()
        if not response.success:
            self.get_logger().error(f"Failed to catch turtle {turtle_name}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()


