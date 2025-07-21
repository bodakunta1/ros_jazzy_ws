#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from turtlesim.srv import Spawn
from functools import partial
import random
import math

class TurtleSpawner(Node):
    def __init__(self):
        super().__init__("turtle_spawner")
        self.counter = 0
        self.turtle_prefix_ = "turtle"
        self.spawn_service_client_ = self.create_client(Spawn, "/spawn")
        self.spawn_servioce_timer_ = self.create_timer(2.0, self.spawn_new_turtle)

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

    def spawn_service_callback(self, future, request):
        response : Spawn.Response = future.result()
        if request.name != "":
            self.get_logger().info(f"New turtle spawned with name : {request.name}")

def main(args=None):
    rclpy.init(args=args)
    node = TurtleSpawner()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    