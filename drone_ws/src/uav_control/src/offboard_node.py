#!/usr/bin/env python3

import rclpy
from rclpy.node import Node


class OffbaordNode(Node):
    def __init__(self):
        super().__init__("offboard_node")
        self.get_logger().info("Offboard node has been started.")


def main(args=None):
    rclpy.init(args=args)

    offboard_node = OffbaordNode()

    rclpy.spin(offboard_node)

    offboard_node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
