#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import threading
from std_msgs.msg import String
import time

class MyNode(Node):
    def __init__(self):
        super().__init__('telem_node')
        self.motor_publisher = self.create_publisher(String, 'tcp_messages', 10)
        self.payload_publisher = self.create_publisher(String, 'tcp_messages', 10)
        self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        pass

def tcp_server(host, port, ros_node):
    server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server_socket.bind((host, port))
    server_socket.listen(1)
    print(f"Server listening on {host}:{port}")

    while True:
        client_socket, client_address = server_socket.accept()
        print(f"Accepted connection from {client_address}")

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

        client_socket.close()

def send_heartbeat(client_socket):
    while True:
        time.sleep(1)
        try:
            client_socket.sendall(b'heartbeat')
        except socket.error:
            print("Error sending heartbeat")
            break

def main(args=None):
    host = '192.168.0.24'  # localhost
    port = 4040

    # Create the ROS2 node
    rclpy.init(args=args)
    node = MyNode()

    # Start the TCP server in a separate thread
    server_thread = threading.Thread(target=tcp_server, args=(host, port, node))
    server_thread.start()

    # Connect to the server for sending heartbeat
    client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    client_socket.connect((host, port))
    
    # Start the send thread for heartbeat
    send_thread = threading.Thread(target=send_heartbeat, args=(client_socket,))
    send_thread.start()

    rclpy.spin(node)
    rclpy.shutdown()

    # Wait for the threads to finish
    server_thread.join()
    send_thread.join()

if __name__ == "__main__":
    main()

