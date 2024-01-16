#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from telemetry_interfaces.msg import MotorResponse

class MotorInterpreterNode(Node):
    def __init__(self):
        super().__init__("motor_interpreter_node")
        self.motor_rpm_subscriber = self.create_subscription(String, "motor_rpm", self.translate_motor_rpm, 10)
        self.vel_publisher = self.create_publisher(MotorResponse, "rover_vel", 10)

    def translate_motor_rpm(self,msg: String):
        response = MotorResponse()
        
        self.vel_publisher.publish(response)

def main(args=None):
    rclpy.init(args=args)
    node = MotorInterpreterNode()
    rclpy.spin(node)
    rclpy.shutdown()