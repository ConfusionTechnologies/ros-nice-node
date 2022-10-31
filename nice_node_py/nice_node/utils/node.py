from rclpy.node import Node

__all__ = ["get_timestamp"]


def get_timestamp(node: Node):
    return node.get_clock().now().to_msg()
