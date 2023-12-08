#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


class EnableRobotSimServer(Node):
    def __init__(self, name):
        super().__init__(name)
        self.enable_srv = self.create_service(Trigger, "robot_enable", self.enable_cb)
        self.disable_srv = self.create_service(Trigger, "robot_disable", self.disable_cb)
        
        self.get_logger().info("Started simulated robot enable node")
    
    '''callback to enable robot'''
    def enable_cb(self, request: Trigger.Request, response: Trigger.Response):
        return self.cb(request, response, "Enabled robot")
    
    '''callback to disable robot'''
    def disable_cb(self, request, response):
        return self.cb(request, response, "Disabled robot")

    '''callback to simulate robot response'''
    def cb(self, request : Trigger.Request, response : Trigger.Response, message):
        response.success = True
        response.message = message
        return response

if __name__ == "__main__":
    rclpy.init()

    node = EnableRobotSimServer("robot_enable_server_sim")

    rclpy.spin(node)

    rclpy.shutdown()