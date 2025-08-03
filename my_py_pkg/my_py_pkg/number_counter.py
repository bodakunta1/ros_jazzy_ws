#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64
from example_interfaces.srv import SetBool

class number_counter(Node):
    def __init__(self):
        super().__init__("number_couter")
        self.subscriber_ = self.create_subscription(Int64, "number", self.callback_counter, 10)
        self.publisher_ = self.create_publisher(Int64, "number_count", 10)
        self.server_ = self.create_service(SetBool, "reset_counter", self.callback_reset_counter)
        self.total_count = 0
        self.get_logger().info("Number counter has been started.")

    def callback_counter(self, msg: Int64):
        self.total_count += msg.data
        self.get_logger().info(f"data: {self.total_count}")
        total_msg = Int64()
        total_msg.data = self.total_count
        self.publisher_.publish(total_msg)

    def callback_reset_counter(self, request, response):
        if request.data:
            self.get_logger().info("Reseting counter")
            self.total_count = 0
            response.success = True
            response.message = "reset finished"
            return response
        else:
            response.success = False
            response.message = "counter still going"
        return response

def main(args=None):
    rclpy.init(args=args)
    node = number_counter()
    rclpy.spin(node)
    rclpy.shutdown()
if __name__ == "__main__":
    main()