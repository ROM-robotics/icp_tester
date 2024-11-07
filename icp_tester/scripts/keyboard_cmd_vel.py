#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_srvs.srv import Trigger

class KeyboardCmdVel(Node):
    def __init__(self):
        super().__init__('keyboard_cmd_vel')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Register shutdown hook to turn off the motor
        self.create_timer(0.1, self.publish_cmd_vel)
    def publish_cmd_vel(self):
        vel = Twist()
        direction = input('k: forward, j: backward, h: left, l: right, return: stop > ')
        if 'k' in direction:
            vel.linear.x = 0.15
        if 'j' in direction:
            vel.linear.x = -0.15
        if 'h' in direction:
            vel.angular.z = 3.14 / 4  # pi/4 [rad/s]
        if 'l' in direction:
            vel.angular.z = -3.14 / 4
        self.get_logger().info(f'Publishing: {vel}')
        self.pub.publish(vel)

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardCmdVel()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()