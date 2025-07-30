#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from functools import partial
from my_robot_interfaces.msg import TurtleName
import random
import math

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.counter = 1
        self.turtle_prefix_ = "turtle"
        
        # Use Pose for the publisher, not Spawn
        self.spawn_pose_pub_ = self.create_publisher(Pose, "/spawn_pose", 1)
        self.spawn_service_client_ = self.create_client(Spawn, "/spawn")

        self.publish_name_turtle_ =  self.create_publisher(TurtleName, "/tturtle_name", 2)

        self.spawn_service_timer_ = self.create_timer(2.0, self.spawn_new_turtle)

    def spawn_new_turtle(self):
        self.counter += 1
        name = self.turtle_prefix_ + str(self.counter)
        x = random.uniform(0.0, 11.0)
        y = random.uniform(0.0, 11.0)
        theta = random.uniform(0.0, 2 * math.pi)
        self.call_spawn_service(x, y, theta, name)

    def call_spawn_service(self, x, y, theta, name):
        if not self.spawn_service_client_.wait_for_service(timeout_sec = 1.0):
            self.get_logger().info("Spawn service not availble, waiting...")

        request = Spawn.Request()
        request.x = x
        request.y = y
        request.theta = theta
        request.name = name 

        future = self.spawn_service_client_.call_async(request)
        future.add_done_callback(partial(self.spawn_service_callback, request = request))
        
        def timer_callback():
            self.publish_spawn(x, y, theta)
            self.publish_turtle_name(name)
            timer.cancel()

        timer = self.create_timer(2.0, timer_callback)

    def publish_spawn(self, x, y, theta):
        pose_msg = Pose()
        pose_msg.x = x
        pose_msg.y = y
        pose_msg.theta = theta
        self.spawn_pose_pub_.publish(pose_msg)

    def publish_turtle_name(self, name):
        name_msg = TurtleName()
        name_msg.turtle_name = name
        self.publish_name_turtle_.publish(name_msg)
        #self.get_logger().info(f"Turtle name is publisher via /turtle_name: {name_msg.turtle_name}")

    def spawn_service_callback(self, future, request):
        response : Spawn.Response = future.result()
        self.get_logger().info(f"New turtle spawned with name : {request.name}")

        #if request.name != "":
        #    self.get_logger().info(f"New turtle spawned with name : {request.name}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
