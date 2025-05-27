#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node): 
    def __init__(self):
        super().__init__("hardware_status_publisher_node")
        self.hw_status_pub_ = self.create_publisher(HardwareStatus, "hardware_status", 10)
        self.timer = self.create_timer(1.0, self.publish_hw_status)
        self.get_logger().info("Hardware Status Publisher Node has been started.")


    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 54.5
        msg.motor_ready = True
        msg.debug_message = "All systems operational"
        self.hw_status_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = HardwareStatusPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()