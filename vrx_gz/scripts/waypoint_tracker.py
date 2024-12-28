#!/usr/bin/env python3

import rclpy
from std_msgs.msg import Float64
from rclpy.node import Node

class ThrustPublisher(Node):
    def __init__(self):
        super().__init__('thrust_publisher')
        
        # Create publishers for both thrusters
        self.right_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.left_thruster_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        
        # Set a timer to call the publish method once
        self.timer = self.create_timer(1.0, self.publish_thrust)
        linear_speed= 100
        angular_speed = 1.0

    def publish_thrust(self):
        # Create the message
        right_thrust_msg = Float64()
        left_thrust_msg = Float64()
        
        # Set the thrust value
        right_thrust_msg.data = self.linear_speed
        left_thrust_msg.data = self.linear_speed
        
        # Publish to both thrusters
        self.right_thruster_pub.publish(right_thrust_msg)
        self.left_thruster_pub.publish(left_thrust_msg)
        
        # Log the published values
        self.get_logger().info(f"Publishing ", self.linear_speed ,"to /wamv/thrusters/right/thrust and /wamv/thrusters/left/thrust")

def main(args=None):
    rclpy.init(args=args)
    thrust_publisher = ThrustPublisher()

    try:
        rclpy.spin(thrust_publisher)
    except KeyboardInterrupt:
        pass
    finally:
        thrust_publisher.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
