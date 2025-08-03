#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from functools import partial
import random
import math
from my_robot_interfaces.msg import Turtle
from my_robot_interfaces.msg import TurtleArray
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from my_robot_interfaces.srv import CatchTurtle

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.counter = 1
        self.turtle_name_prefix_ = "turtle"
        self.alive_turtles_ = []

        self.spawn_service_client_ = self.create_client(Spawn, "/spawn")
        self.kill_client_ = self.create_client(Kill, "/kill")

        self.spawn_service_timer_ = self.create_timer(2.0, self.spawn_new_turtle)

        self.alive_turtles_publisher_ = self.create_publisher(TurtleArray, "/alive_turtles", 10)

        self.catch_turtle_service_ = self.create_service(
            CatchTurtle, "/catch_turtle", self.callback_catch_turtle)

    def callback_catch_turtle(self, request: CatchTurtle.Request, response: CatchTurtle.Response):
        self.call_kill_service(request.name)
        response.success = True
        return response

    def publish_alive_turtles(self):
        msg = TurtleArray()
        msg.turtles = self.alive_turtles_
        self.alive_turtles_publisher_.publish(msg)
        
    def spawn_new_turtle(self):
        self.counter += 1
        name = self.turtle_name_prefix_ + str(self.counter)
        x = random.uniform(0.0, 9.0)
        y = random.uniform(0.0, 9.0)
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
        future.add_done_callback(
            partial(self.callback_call_spawn_service, request = request))
        
    def callback_call_spawn_service(self, future, request:Spawn.Request):
        response : Spawn.Response = future.result()
        if response.name != '':
            self.get_logger().info("Spawned " + response.name)
            new_turtle = Turtle()
            new_turtle.name = response.name
            new_turtle.x = request.x
            new_turtle.y = request.y
            new_turtle.theta = request.theta
            self.alive_turtles_.append(new_turtle)
            self.publish_alive_turtles()

    def call_kill_service(self, turtle_name):
        while not self.kill_client_.wait_for_service(1.0):
            self.get_logger().info("kill service not available, waiting...")

        request = Kill.Request()
        request.name = turtle_name

        future = self.kill_client_.call_async(request)
        future.add_done_callback(
            partial(self.callback_call_kill_service, turtle_name = turtle_name))
        
    def callback_call_kill_service(self, future, turtle_name):
        for (i, turtle) in enumerate(self.alive_turtles_):
            if turtle.name == turtle.name:
                del self.alive_turtles_[i]
                self.get_logger().info("Killed " + turtle.name)
                self.publish_alive_turtles()
                return    

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
