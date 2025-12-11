#!/usr/bin/env python3
"""
Simple velocity command relay node.
Subscribes to /cmd_vel_nav and republishes to /cmd_vel.
This bridges Nav2 velocity commands to the Gazebo robot.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Subscribe to Nav2's velocity command topic
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10
        )
        
        # Publish to Gazebo's expected velocity topic
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('CmdVelRelay started: /cmd_vel_nav -> /cmd_vel')

    def cmd_vel_callback(self, msg):
        # Simply forward the message
        self.publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
