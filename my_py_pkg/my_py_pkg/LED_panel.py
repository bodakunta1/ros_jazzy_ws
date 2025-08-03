#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from my_robot_interfaces.msg import Ledstatus
from my_robot_interfaces.srv import SetLed


class LEDpanelNode(Node):
    def __init__(self):
        super().__init__("LED_panel_node")

        self.declare_parameter("led_states_", [0, 0, 0])
        self.led_states_ = self.get_parameter("led_states_").value

        self.led_states_pub_ = self.create_publisher(Ledstatus, "led_panel_state", 10)
        self.get_logger().info("Led panel states are publishing")
        self.led_states_timer_ = self.create_timer(5.0, self.publish_led_states)
        self.led_server_ = self.create_service(SetLed, "set_led", self.set_led_callback)

    def set_led_callback(self, request: SetLed.Request, response: SetLed.Response):
        led_number = request.led_number
        led_state = request.led_state
        
        self.get_logger().info(f"Received request to set LED {led_number} {led_state}")

        if led_number >= len(self.led_states_) or led_number < 0:
            response.success = False
            return response
        
        if led_state not in [0, 1]:
            response.success = False
            return response
        
        self.led_states_[led_number] = led_state
        response.success = True
        return response

    def publish_led_states(self):
        msg = Ledstatus()  #Assigning the message type which is a int64 list defined in custom msg in custom interfaces
        msg.led_states = self.led_states_
        self.led_states_pub_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = LEDpanelNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    