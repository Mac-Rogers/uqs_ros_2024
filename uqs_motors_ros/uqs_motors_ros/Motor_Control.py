#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from telemetry_interfaces.msg import MotorCommands, WheelVelocities


class MotorControlNode(Node):
    self.wheelRadius = 0.1
    self.roverWidth = 1
    def __init__(self):
        super().__init__("motor_control_node")

        self.subscriber_ = self.create_subscription(MotorCommands, 'motor_commands', self.translate_motor_cmd, 10)
        self.pico_cmd_publisher = self.create_publisher(WheelVelocities, 'pico_commands', 10)

    
    def translate_motor_cmd(self, msg: MotorCommands):
        pico_cmd = WheelVelocities()
        pico_cmd.ang_vel_one = pico_cmd.ang_vel_two = pico_cmd.ang_vel_three = msg.lin_vel/self.wheelRadius - (self.roverWidth*msg.ang_vel)/(2*self.wheelRadius) 
        pico_cmd.ang_vel_four = pico_cmd.ang_vel_five = pico_cmd.ang_vel_six = msg.lin_vel/self.wheelRadius + (self.roverWidth*msg.ang_vel)/(2*self.wheelRadius) 
        self.pico_cmd_publisher.publish(pico_cmd)

def main(args=None):
    rclpy.init(args=args)
    node = MotorControlNode()
    rclpy.spin(node)
    rclpy.shutdown()
    