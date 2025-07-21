#!usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
from geometry_msgs.msg import Twist

class TurtleController(Node):
    def __init__(self):
        super().__init__("turtle_controller")
        self.pose_ :Pose = None
        self.target_x = 2.0
        self.target_y = 4.0
        self.cmd_vel_pub_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.pose_sub_ = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.controll_timer_ = self.create_timer(0.01, self.control_loop)

    def pose_callback(self, pose: Pose):
        self.pose_ = pose

    def control_loop(self):
        if self.pose_ == None:
            return
                      
        #Calculate the distance to the target
        dist_x = self.target_x - self.pose_.x
        dist_y = self.target_y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        cmd = Twist()

        if distance > 0.5:
            cmd.linear.x = 2 * distance
            
            #Calculate the orientation angle for turtle to face the target
            angle_theta = math.atan2(dist_y, dist_x)
            angle_diff = angle_theta - self.pose_.theta

            #Normalise the angle difference to be within -pi to pi
            if angle_diff > math.pi:
                angle_diff -= 2 * math.pi
            elif angle_diff < -math.pi:
                angle_diff += 2 * math.pi   
                
            #Sending the command to turtle
            cmd.angular.z = 6 * angle_diff

        else:
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0             

        self.cmd_vel_pub_.publish(cmd)
        
def main(args=None):
    rclpy.init(args=args)
    node = TurtleController()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()
    