#! /usr/bin/env python3
import rclpy 
from rclpy.node import Node
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
from random import randrange,choice
import math
from my_robot_interfaces.srv import SendCords
class customnode(Node):
    def __init__(self):
        super().__init__("node_name")
        self.counter = 2
        self.active = False
        self.kill_client = self.create_client(Kill, "/kill")
        self.spawn_client = self.create_client(Spawn, "/spawn")
        self.go_to_client = self.create_client(SendCords, "/go_to")
        self.timer = self.create_timer(1.0, self.timer_callback)
        self.dict_arr=[]
        self.pose_sub = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)


    def compue_close(self):
        min_dist = 100.0
        target_tur = None
        for turtle in self.dict_arr:
            dist = math.hypot((turtle["x"] - self.pose.x), (turtle["y"]- self.pose.y))
            min_dist = min(min_dist, dist)
        for turtle in self.dict_arr:
            dist = math.hypot((turtle["x"] - self.pose.x), (turtle["y"] - self.pose.y))
            if(dist == min_dist):
                target_tur = turtle
                break
        #self.get_logger().info(f"min_dist: {min_dist}")
        return (target_tur)
    def send(self):
        target_tur = self.compue_close()
        #self.get_logger().info(f"entered send with target turtle: {target_tur}, total_turtles: {len(self.dict_arr)}")
        if not self.active and target_tur is not None:
            
            self.call_go_to(target_tur["x"], target_tur["y"])
            self.active = True
            #self.get_logger().info("just sent a command to move!")

        if self.active and target_tur is not None:
            curr_dist = math.hypot((target_tur["x"] - self.pose.x), (target_tur["y"]- self.pose.y))
            if(curr_dist < 0.60): 
                self.active = False
                self.call_kill(target_tur['name'])
                self.dict_arr.remove(target_tur)
                target_tur = None
    def pose_callback(self,msg):
        self.pose = msg
        self.send()
    def timer_callback(self):
        name = f"turtle{self.counter}"
        x,y = float(randrange(2, 8, 1)), float(randrange(2, 8, 1))
        d = {
            "x": x,
            "y": y,
            "name": name
        }
        self.dict_arr.append(d)
        self.call_spawn(x, y, name)
        self.counter+=1


    def call_spawn(self, x, y, name):
        while not self.spawn_client.wait_for_service(1.0):
            pass
            #self.get_logger().warn("Spawn server not started .....")

        request = Spawn.Request()
        request.x=x
        request.y=y
        request.name=name

        future = self.spawn_client.call_async(request)
        future.add_done_callback(self.callback_call_spawn)
    
    def callback_call_spawn(self, future):
        responce = future.result()
        #self.get_logger().info(f"got_resp: {responce.name}")


    def call_go_to(self, x, y):
        while not self.go_to_client.wait_for_service(1.0):
            
            pass
            # self.get_logger().warn("go_to server not started .....")
        
        request = SendCords.Request()
        request.x=x
        request.y=y

        future = self.go_to_client.call_async(request)
        future.add_done_callback(self.callback_call_go_to)
    
    def callback_call_go_to(self, future):
        responce = future.result()
        #self.get_logger().info(f"got_resp: {responce.reached}")

    def call_kill(self, name):
        while not self.kill_client.wait_for_service(1.0):
            pass
            #self.get_logger().warn("Kill server not started .....")

        request = Kill.Request()
        request.name=str(name)

        future = self.kill_client.call_async(request)
        future.add_done_callback(self.callback_call_spawn)
    
    def callback_call_spawn(self, future):
        pass
        #self.get_logger().info(f"kill service completed")


def main(args =None):
    rclpy.init(args=args)
    node=customnode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ =="__main__":
    main()