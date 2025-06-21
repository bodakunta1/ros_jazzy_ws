#!usr/bin/env python3
import rclpy
from rclpy.node import Node
from example_interfaces.msg import String

class RobotNewsStationNode(Node): #Modify Name
    def __init__(self):
        super().__init__("robot_news_station")
        self.declare_parameter("robot_name_", "R2D2")
        self.declare_parameter("timer_", 1.0)

        self.robot_name_ = self.get_parameter("robot_name_").value
        self.timer_ = self.get_parameter("timer_").value

        self.publisher_ = self.create_publisher(String, "topic_name_string", 10)
        self.timer_ = self.create_timer(0.5, self.publish_news)
        self.get_logger().info("Robot news Station has been started")
        
    def publish_news(self):
        msg = String()
        msg.data = f"Hello,this is {self.robot_name_} from robot news station!"
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RobotNewsStationNode() #Modify Name
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    