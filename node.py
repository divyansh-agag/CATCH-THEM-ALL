#! /usr/bin/env python3
import rclpy 
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from my_robot_interfaces.srv import SendCords
import math
import time
class customnode(Node):
    def __init__(self):
        super().__init__("node_name")
        self.dist=1.0#initialization
        self.target=(-1, -1)
        self.sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.pub = self.create_publisher(Twist, "/turtle1/cmd_vel", 10) 
        self.server = self.create_service(SendCords, "go_to", self.go_to_callback)
           
    def go_to_callback(self, request:SendCords.Request, responce: SendCords.Response):
        self.target = (request.x, request.y)
        responce.reached = True
        return responce

    def compute_vel(self):
        if self.target[0] != -1:
            self.dist = math.hypot((self.pose.x - self.target[0]),(self.pose.y - self.target[1]))
            vel_const = 1.0
            vel = vel_const*self.dist
            angle_from_target = math.atan2((self.target[1] - self.pose.y), 
                                        (self.target[0] - self.pose.x))
            
            if(self.pose.theta >= 0): self.pose.theta = self.pose.theta*(180/3.14)
            else: self.pose.theta = 360 + self.pose.theta*(180/3.14)
            if angle_from_target*(180/3.14)>0:angle_from_target = angle_from_target*(180/3.14)
            else: angle_from_target = 360 + angle_from_target*(180/3.14)
            angle_diff = (angle_from_target - self.pose.theta)
            self.get_logger().info(f"theta: {self.pose.theta}, angle_from_tar: {angle_from_target}, diff: {angle_diff}")
            ang_vel = angle_diff*(0.04)
            return (vel, ang_vel)
        else: return (0.0,0.0)
    def pose_callback(self, msg):
        self.pose = msg
        self.publish()
        #self.get_logger().info()
    def publish(self):
        pub_msg = Twist()
        vel,ang_vel = self.compute_vel()
        if(self.dist > 0.25):
            pub_msg.linear.x = vel
            pub_msg.angular.z = ang_vel
        else: self.target = (-1, -1)
        self.pub.publish(pub_msg)


def main(args =None):
    rclpy.init(args=args)
    node=customnode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ =="__main__":
    main()