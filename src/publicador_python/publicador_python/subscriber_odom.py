#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

class OdomSubscriber(Node):
    def __init__(self):
        super().__init__('odom_subscriber')
        self.subscription = self.create_subscription(
            Odometry,
            'wheel/odometry',      # Ajusta este tópico al que realmente publiques en tu sistema
            self.listener_callback,
            10)
        self.subscription  # Para evitar warning de “variable no referenciada”

    def listener_callback(self, msg):
        self.get_logger().info('I heard: "%s"' % msg)

def main(args=None):
    rclpy.init(args=args)
    node = OdomSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
