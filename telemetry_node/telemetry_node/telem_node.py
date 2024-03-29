#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from telemetry_interfaces.msg import MotorDrivers
from turtlesim.msg import Pose  # Assuming turtlesim Pose message for illustration
import socket
import threading

class MyNode(Node):
    def __init__(self):
        super().__init__('telem_node')
        self.motor_publisher = self.create_publisher(String, 'tcp_messages', 10)
        self.payload_publisher = self.create_publisher(String, 'tcp_messages', 10)
        self.create_timer(1.0, self.timer_callback)
        self.turtle_pose_subscription = self.create_subscription(
            Pose,
            'turtle1/pose',  # Replace with your actual topic for turtle pose
            self.turtle_pose_callback,
            10
        )
        self.msg = 'jimmy'
        self.connected_clients = []  # To store connected clients

    def timer_callback(self):
        # Send heartbeat to all connected clients
        for client_socket in self.connected_clients:
            try:
                client_socket.sendall(self.msg.encode('utf-8'))
            except socket.error:
                print("Error sending heartbeat to client")
                self.connected_clients.remove(client_socket)

    def turtle_pose_callback(self, msg):
        # Update the message to be sent
        self.msg = f'Turtle Pose: x={msg.x}, y={msg.y}, theta={msg.theta}'

        # Publish the received message as a ROS2 message
        message = String()
        message.data = self.msg
        if message.data.split(':')[0] in ['ATLAS', 'GAIA', 'PAVERS']:
            self.payload_publisher.publish(message)
            self.get_logger().info(f"payload {message.data}")
        md_cmd = MotorDrivers()
        if message.data.split(':')[-1] == 'FWD':
            md_cmd.lin_vel = 1
            md_cmd.motor_status = True
            self.motor_publisher.publish(md_cmd)
        self.get_logger().info(f"{message.data}")
        self.get_logger().info(f"{md_cmd}")

def handle_client(client_socket, ros_node):
    while True:
        data = client_socket.recv(1024)
        if not data:
            break

        # Publish the received message as a ROS2 message
        message = String()
        message.data = data.decode('utf-8')
        if message.data.split(':')[0] in ['ATLAS', 'GAIA', 'PAVERS']:
            ros_node.payload_publisher.publish(message)
            ros_node.get_logger().info(f"payload {message.data}")
        ros_node.motor_publisher.publish(message)
        ros_node.get_logger().info(f"motors {message.data}")

        print(f"Received from client: {data.decode('utf-8')}")

    client_socket.close()

def tcp_server(host, port, ros_node):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Server listening on {host}:{port}")

    while True:
        client_socket, client_address = server_socket.accept()
        print(f"Accepted connection from {client_address}")

        # Start a new thread to handle the client
        client_thread = threading.Thread(target=handle_client, args=(client_socket, ros_node))
        client_thread.start()

        # Add the client socket to the list of connected clients
        ros_node.connected_clients.append(client_socket)

def main(args=None):
    host = '192.168.0.25'  # localhost
    port = 4040

    # Create the ROS2 node
    rclpy.init(args=args)
    node = MyNode()

    # Start the TCP server in a separate thread
    server_thread = threading.Thread(target=tcp_server, args=(host, port, node))
    server_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()

    # Wait for the server thread to finish
    server_thread.join()

if __name__ == "__main__":
    main()

