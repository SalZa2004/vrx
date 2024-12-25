#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped
from math import atan2, sqrt

class WaypointTracker(Node):
    def __init__(self):
        super().__init__('waypoint_tracker')  # Initialize the node with its name
        # Publishers for the left and right thrusters
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        # Subscriber for robot's pose
        self.pose_sub = self.create_subscription(PoseStamped, '/pose', self.pose_callback, 10)

        # Waypoints (as (x, y) coordinates)
        self.waypoints = [(544, 156), (528, 192)]
        self.current_waypoint_idx = 0
        self.current_pose = None

        # Timer for periodically updating movement
        self.timer = self.create_timer(0.1, self.move_to_waypoint)

        # Parameters for controlling robot speed
        self.linear_speed = 1000  # Adjust as needed
        self.angular_speed = 0  # Adjust as needed

        self.get_logger().info("Waypoint Tracker Node Initialized.")

    def pose_callback(self, msg):
        """Callback to update the current pose of the robot."""
        self.current_pose = msg.pose.position

    def move_to_waypoint(self):
        """Move the robot toward the current waypoint."""
        if self.current_pose is None or self.current_waypoint_idx >= len(self.waypoints):
            return

        # Get current position and target waypoint
        target_x, target_y = self.waypoints[self.current_waypoint_idx]
        current_x = self.current_pose.x
        current_y = self.current_pose.y

        # Calculate distance and angle to the waypoint
        distance = sqrt((target_x - current_x) ** 2 + (target_y - current_y) ** 2)
        angle_to_target = atan2(target_y - current_y, target_x - current_x)

        # Check if the robot is close to the target waypoint
        if distance < 0.1:  # Threshold for reaching the waypoint
            self.get_logger().info(f"Reached waypoint {self.current_waypoint_idx + 1}")
            self.current_waypoint_idx += 1
            if self.current_waypoint_idx >= len(self.waypoints):
                self.get_logger().info("All waypoints reached!")
                self.publish_thrust(0.0, 0.0)  # Stop thrusters
                return

        # Calculate thrust values for left and right thrusters
        left_thrust = self.linear_speed
        right_thrust = self.linear_speed

        # Adjust thrust for angular correction
        angular_error = angle_to_target
        left_thrust -= self.angular_speed * angular_error
        right_thrust += self.angular_speed * angular_error

        # Clamp thrust values to a valid range
        left_thrust = max(min(left_thrust, 1.0), -1.0)
        right_thrust = max(min(right_thrust, 1.0), -1.0)

        # Publish the thrust values to the corresponding topics
        self.publish_thrust(left_thrust, right_thrust)

    def publish_thrust(self, left_thrust, right_thrust):
        """Publish thrust values to the left and right thrusters."""
        left_thrust_msg = Float64()
        right_thrust_msg = Float64()

        left_thrust_msg.data = left_thrust
        right_thrust_msg.data = right_thrust

        self.left_thrust_pub.publish(left_thrust_msg)
        self.right_thrust_pub.publish(right_thrust_msg)

        self.get_logger().info(f"Left Thrust: {left_thrust:.2f}, Right Thrust: {right_thrust:.2f}")

def main(args=None):
    rclpy.init(args=args)
    waypoint_tracker = WaypointTracker()
    try:
        rclpy.spin(waypoint_tracker)
    except KeyboardInterrupt:
        waypoint_tracker.get_logger().info("Shutting down Waypoint Tracker Node.")
    finally:
        waypoint_tracker.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
