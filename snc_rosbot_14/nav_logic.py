#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Empty, String
import subprocess

class NavLogicNode(Node):
    def __init__(self):
        super().__init__('nav_logic')
        self.get_logger().info("🧠 nav_logic node started.")
        
        # Publisher: current status ("Idle", "Exploring", "Returning")
        self.status_pub = self.create_publisher(String, '/snc_status', 10)
        
        # Subscriber: wait for exploration trigger
        self.create_subscription(Empty, '/trigger_start', self.start_exploration, 10)

        # Initial state
        self.status = "Idle"
        self.publish_status()

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.status_pub.publish(msg)

    def start_exploration(self, msg):
        if self.status != "Idle":
            self.get_logger().warn("🟡 Already running. Ignoring trigger.")
            return

        self.status = "Exploring"
        self.publish_status()
        self.get_logger().info("🟢 Exploration started!")

        # Launch explore-lite (or your own explorer)
        subprocess.Popen([
            "ros2", "launch", "explore_lite", "explore.launch.py"
        ])

def main(args=None):
    rclpy.init(args=args)
    node = NavLogicNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
