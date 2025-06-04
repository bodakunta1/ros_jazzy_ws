#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import Int64  # Importing Int64 message type
#from my_robot_interfaces.msg import Ledstatus
from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.client_ = self.create_client(SetLed, "set_led")
        self.get_logger().info("Battery node started as service client")
        self.led_number = 0
        self.led_state = 0
        self.timer_ = self.create_timer(4.0, self.timer_callback)

    def timer_callback(self):
        self.call_set_led(self.led_number, self.led_state)
        self.led_state = 1 - self.led_state
        self.led_number = (self.led_number +1 ) % 3
 
    def call_set_led(self, led_number, led_state):
        req = SetLed.Request()
        req.led_number = led_number
        req.led_state = led_state
        self.future = self.client_.call_async(req)
        self.get_logger().info(f"Requested LED {led_number} to state {led_state}")
           
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    