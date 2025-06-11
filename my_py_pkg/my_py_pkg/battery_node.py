#!usr/bin/env python3
import rclpy
from rclpy.node import Node
import time
#from example_interfaces.msg import Int64  # Importing Int64 message type
#from my_robot_interfaces.msg import Ledstatus
from my_robot_interfaces.srv import SetLed

class BatteryNode(Node):
    def __init__(self):
        super().__init__("battery_node")
        self.set_led_client_ = self.create_client(SetLed, "set_led")
        self.get_logger().info("Battery node started as service client")
        self.battery_state_ = "full"
        self.previous_battery_status_ = self.current_time()
        first_time_ = time.strftime("%Y-%m-%d %H:%M:%S")
        self.get_logger().info(f"Current time: {first_time_}")
        self.timer_ = self.create_timer(0.1, self.check_battery_status)
    
    def current_time(self):
        seconds, nanoseconds = self.get_clock().now().seconds_nanoseconds()
        return seconds + nanoseconds / 1000000000.0

    def check_battery_status(self):
        time_now = self.current_time()
        if self.battery_state_ == "full":
            if time_now - self.previous_battery_status_ > 4.0:
                self.battery_state_ = "empty"
                self.get_logger().info("Battery is empty, charging now....")
                self.call_set_led(2, 1)
                self.call_set_led(1, 1)
                self.call_set_led(0, 1)
                self.previous_battery_status_ = time_now
        elif self.battery_state_ == "empty":
            if time_now - self.previous_battery_status_ > 6.0:
                self.battery_state_ = "full"
                self.get_logger().info("Battery is now full.")
                self.call_set_led(2, 0)
                self.call_set_led(1, 0)
                self.call_set_led(0, 0)
                self.previous_battery_status_ = time_now

    def call_set_led(self, led_number, led_state):
        while not self.set_led_client_.wait_for_service(1.0):
            self.get_logger().warn("waiting for Set Led services")
        
        req = SetLed.Request()
        req.led_number = led_number
        req.led_state = led_state

        self.future = self.set_led_client_.call_async(req)
        self.future.add_done_callback(self.callback_call_set_led)


        self.get_logger().info(f"Requested LED {led_number} to state {led_state}")

    def callback_call_set_led(self, future):
        response: SetLed.Response = future.result()
        if response.success:
            self.get_logger().info("Led state changed")
        else:
            self.get_logger().info("Led not changed")
           
def main(args=None):
    rclpy.init(args=args)
    node = BatteryNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    