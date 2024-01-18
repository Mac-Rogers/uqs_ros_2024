#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node
from telemetry_interfaces.msg import MotorDrivers
import socket
import threading

class MotorServer(Node):
    def __init__(self, ip, port):
        super().__init__('motor_server')
        self.create_subscription(
            MotorDrivers,
            'md2telem',
            self.callback,
            10
        )
        self.publisher = self.create_publisher(MotorDrivers, 'telem2md', 10)

        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.bind((ip, port))
        self.server_socket.listen()

        self.get_logger().info(f"Motor server listening on {ip}:{port}")

        self.client_socket = None
        self.client_connected = False

        self.heartbeat_thread = threading.Thread(target=self.send_heartbeat)
        self.heartbeat_thread.start()

        # Start a timer for listening to the TCP socket
        self.tcp_listener_timer = self.create_timer(0.1, self.listen_to_tcp)

    def listen_to_tcp(self):
        if not self.client_connected:
            try:
                self.client_socket, _ = self.server_socket.accept()
                self.client_connected = True
                self.get_logger().info("Client connected")
            except socket.error:
                pass  # No new connection yet, ignore and continue

        try:
            data = self.client_socket.recv(1024)
            if not data:
                self.client_connected = False
                self.get_logger().info("Client disconnected")
            else:
                decoded_data = data.decode('utf-8')
                self.process_received_data(decoded_data)
        except socket.error:
            self.client_connected = False
            self.get_logger().error("Error receiving data from TCP client")

    def process_received_data(self, decoded_data):
        message = MotorDrivers()
        self.get_logger().info(f"Received from TCP client: {decoded_data}")

        if decoded_data.split(':')[0] in ['ATLAS', 'GAIA', 'PAVERS']:
            self.get_logger().info(f"Payload: {decoded_data}")
            # Construct your MotorDrivers message based on the received payload

        elif decoded_data.split(':')[-1] in ['FWD', 'BWD', 'LEFT', 'RIGHT']:
            # Determine linear and angular velocities based on the received command
            if decoded_data.split(':')[-1] == 'FWD':
                message.lin_vel = 1.0
            elif decoded_data.split(':')[-1] == 'BWD':
                message.lin_vel = -1.0
            elif decoded_data.split(':')[-1] == 'LEFT':
                message.ang_vel = 1.0
            elif decoded_data.split(':')[-1] == 'RIGHT':
                message.ang_vel = -1.0

            self.publisher.publish(message)
            self.get_logger().info(f"Published message to telem2md: {message}")

    def callback(self, msg):
        # Convert the received ROS message to a string or another suitable format
        motor_data_str = f'Linear Velocity: {msg.lin_vel}, Angular Velocity: {msg.ang_vel}'
        if self.client_connected:
            try:
                serialized_msg = motor_data_str.encode('utf-8')
                self.client_socket.sendall(serialized_msg)
                self.get_logger().info("Sent message to TCP client")
            except socket.error:
                self.get_logger().error("Error sending message to TCP client")

    def send_heartbeat(self):
        while True:
            if self.client_connected:
                current_time_ms = int(time.time() * 1000)
                time_str = str(current_time_ms)
                try:
                    self.client_socket.sendall(time_str.encode('utf-8'))
                except socket.error:
                    self.client_connected = False
                    self.get_logger().error("Error sending heartbeat to TCP client")

            time.sleep(1.0) 

def main(args=None):
    rclpy.init(args=args)
    ip = '192.168.0.28'  # Replace with the desired IP address
    port = 4040  # Replace with the desired port number
    motor_server = MotorServer(ip, port)
    rclpy.spin(motor_server)
    motor_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

