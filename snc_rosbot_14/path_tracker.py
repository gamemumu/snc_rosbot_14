#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty
from tf2_ros import Buffer, TransformListener, LookupException
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from rclpy.duration import Duration
import tf2_ros

class PathTracker(Node):
    def __init__(self):
        super().__init__('path_tracker')

        self.declare_parameter("explore_topic", "/path_explore")
        self.declare_parameter("return_topic", "/path_return")

        self.explore_path_pub = self.create_publisher(Path, self.get_parameter("explore_topic").value, 10)
        self.return_path_pub = self.create_publisher(Path, self.get_parameter("return_topic").value, 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.path = []
        self.explore_path_msg = Path()
        self.explore_path_msg.header.frame_id = 'map'

        self.return_path_msg = Path()
        self.return_path_msg.header.frame_id = 'map'

        self.create_timer(0.5, self.record_path)
        self.create_subscription(Empty, '/trigger_home', self.return_home, 10)

        self.nav_to_pose_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

    def record_path(self):
        try:
            # Wait until transform is available
            if not self.tf_buffer.can_transform('map', 'base_link', rclpy.time.Time(), timeout=Duration(seconds=1.0)):
                self.get_logger().warn("⏳ TF not ready yet: map → base_link")
                return

            now = self.get_clock().now().to_msg()

            trans = self.tf_buffer.lookup_transform(
                'map',
                'base_link',
                rclpy.time.Time(),  # latest
                timeout=Duration(seconds=1.0)
            )

            pose = PoseStamped()
            pose.header.stamp = now
            pose.header.frame_id = 'map'
            pose.pose.position.x = trans.transform.translation.x
            pose.pose.position.y = trans.transform.translation.y
            pose.pose.position.z = trans.transform.translation.z
            pose.pose.orientation = trans.transform.rotation

            self.path.append(pose)
            self.explore_path_msg.header.stamp = now
            self.explore_path_msg.poses = self.path
            self.explore_path_pub.publish(self.explore_path_msg)

        except (tf2_ros.LookupException,
                tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException) as e:
            self.get_logger().warn(f"TF Error: {e}")





    def return_home(self, _msg):
        self.get_logger().info("🔁 Trigger received: Returning home...")

        if not self.path:
            self.get_logger().warn("⚠ No path recorded, cannot return home.")
            return

        reversed_path = list(reversed(self.path))
        self.return_path_msg.header.stamp = self.get_clock().now().to_msg()
        self.return_path_msg.poses = reversed_path
        self.return_path_pub.publish(self.return_path_msg)

        # Send Nav2 goals one-by-one (coarsely spaced)
        self.send_goals(reversed_path[::10])  # Every 10th point for smoother nav

    def send_goals(self, waypoints):
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("❌ Nav2 action server not available")
            return

        for idx, pose in enumerate(waypoints):
            goal = NavigateToPose.Goal()
            goal.pose = pose

            self.get_logger().info(f"🚶 Sending goal {idx + 1}/{len(waypoints)}")
            send_goal_future = self.nav_to_pose_client.send_goal_async(goal)
            rclpy.spin_until_future_complete(self, send_goal_future)
            result_future = send_goal_future.result().get_result_async()
            rclpy.spin_until_future_complete(self, result_future)

            result = result_future.result()
            if result.error_code != 0:
                self.get_logger().warn(f"⚠ Goal {idx + 1} failed with error code: {result.error_code}")
                break

        self.get_logger().info("✅ Return home complete")

def main(args=None):
    rclpy.init(args=args)
    node = PathTracker()

    # ✅ Give TF time to initialize
    node.get_logger().info("⏳ Waiting for TF to initialize...")
    rclpy.spin_once(node, timeout_sec=3.0)

    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
