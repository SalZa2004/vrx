import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
import sys
import termios
import tty
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='teleop_twist_keyboard',
            executable='teleop_twist_keyboard',
            name='keyboard_control',
            output='screen'
        ),
        # Add any additional nodes or parameters if needed
    ])


class KeyboardControlNode(Node):
    def __init__(self):
        super().__init__('keyboard_control')
        self.left_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/left/thrust', 10)
        self.right_thrust_pub = self.create_publisher(Float64, '/wamv/thrusters/right/thrust', 10)
        self.get_logger().info("Keyboard control for WAMV initialized. Use W/A/S/D to control.")

    def run(self):
        # Setup terminal for reading single characters
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(fd)
            while True:
                ch = sys.stdin.read(1)
                if ch == 'w':  # Increase left thrust
                    self.publish_thrust(1.0, 0.0)
                elif ch == 's':  # Decrease left thrust
                    self.publish_thrust(-1.0, 0.0)
                elif ch == 'a':  # Increase right thrust
                    self.publish_thrust(0.0, 1.0)
                elif ch == 'd':  # Decrease right thrust
                    self.publish_thrust(0.0, -1.0)
                elif ch == 'q':  # Exit
                    break
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)

    def publish_thrust(self, left, right):
        left_msg = Float64()
        right_msg = Float64()
        left_msg.data = left
        right_msg.data = right
        self.left_thrust_pub.publish(left_msg)
        self.right_thrust_pub.publish(right_msg)

def main(args=None):
    rclpy.init(args=args)
    keyboard_control_node = KeyboardControlNode()
    keyboard_control_node.run()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


