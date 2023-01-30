from rclpy.node import Node

__all__ = ["get_timestamp", "rosimg2numpy"]


def get_timestamp(node: Node):
    return node.get_clock().now().to_msg()


cvbridge = None


def rosimg2numpy(msg, enc: str = "passthrough"):
    """Converts ROS CompressedImage/Image to numpy array.

    For `enc`, see https://github.com/ros2/common_interfaces/blob/rolling/sensor_msgs/include/sensor_msgs/image_encodings.hpp
    for available encodings.

    If msg is neither CompressedImage/Image, it will return the original msg.

    Args:
        msg (Any): CompressedImage/Image.
        enc (str, optional): Encoding for numpy array. Defaults to "passthrough".
    """
    global cvbridge
    # NOTE: cvbridge is optional dependency, imported here only when needed
    # dont ask me why cvbridge decided to use a singleton structure instead
    # of storing their conversion maps as globals
    from cv_bridge import CvBridge
    from sensor_msgs.msg import CompressedImage, Image

    if not cvbridge:
        cvbridge = CvBridge()

    if isinstance(msg, Image):
        return cvbridge.imgmsg_to_cv2(msg, enc)
    elif isinstance(msg, CompressedImage):
        return cvbridge.compressed_imgmsg_to_cv2(msg, enc)
    else:
        return msg
