from __future__ import annotations

import sys
from dataclasses import dataclass, field

import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from nicefaces.msg import BBox2Ds, ObjDet2Ds
from rclpy.node import Node
from ros2topic.api import get_msg_class
from sensor_msgs.msg import Image

from nicepynode import Job, JobCfg
from nicepynode.utils import (
    RT_PUB_PROFILE,
    RT_SUB_PROFILE,
    convert_bboxes,
    declare_parameters_from_dataclass,
)

NODE_NAME = "bbox_converter"

cv_bridge = CvBridge()


@dataclass
class BBoxConverterCfg(JobCfg):
    frames_in_topic: str = "~/frames_in"
    """Video frames to predict on."""
    bbox_in_topic: str = "~/bbox_in"
    """Input topic for bboxes to crop."""
    bbox_out_topic: str = "/bbox_out"
    """Output topic for predictions."""
    bbox_type: int = BBox2Ds.XYWH
    """Output type for bbox."""
    normalize_bbox: bool = True
    """Whether to normalize bbox."""


@dataclass
class BBoxConverter(Job[BBoxConverterCfg]):

    ini_cfg: BBoxConverterCfg = field(default_factory=BBoxConverterCfg)

    def attach_params(self, node: Node, cfg: BBoxConverterCfg):
        super(BBoxConverter, self).attach_params(node, cfg)

        declare_parameters_from_dataclass(node, cfg)

    def attach_behaviour(self, node: Node, cfg: BBoxConverterCfg):
        super(BBoxConverter, self).attach_behaviour(node, cfg)

        self.log.info(f"Waiting for publisher@{cfg.frames_in_topic}...")
        self._frames_sub = Subscriber(
            node,
            # blocks until image publisher is up!
            get_msg_class(node, cfg.frames_in_topic, blocking=True),
            cfg.frames_in_topic,
            qos_profile=RT_SUB_PROFILE,
        )

        self._bbox_sub = Subscriber(
            node, ObjDet2Ds, cfg.bbox_in_topic, qos_profile=RT_SUB_PROFILE
        )

        self._synch = ApproximateTimeSynchronizer(
            (self._frames_sub, self._bbox_sub),
            30,  # max 10 frame difference between pred & frame
            1 / 30,  # min 30 FPS waiting to sync
        )
        self._synch.registerCallback(self._on_input)

        self._bbox_pub = node.create_publisher(
            ObjDet2Ds, cfg.bbox_out_topic, RT_PUB_PROFILE
        )
        self.log.info("Ready")

    def detach_behaviour(self, node: Node):
        super().detach_behaviour(node)
        node.destroy_subscription(self._frames_sub.sub)
        node.destroy_subscription(self._bbox_sub.sub)
        node.destroy_publisher(self._bbox_pub)

    def on_params_change(self, node: Node, changes: dict):
        self.log.info(f"Config changed: {changes}.")
        if any(
            n in ("frames_in_topic", "bbox_in_topic", "bbox_out_topic") for n in changes
        ):
            self.log.info("Config change requires restart.")
            return True
        return False

    def _on_input(self, imgmsg: Image, detsmsg: ObjDet2Ds):
        if self._bbox_pub.get_subscription_count() < 1:
            return

        if isinstance(imgmsg, Image):
            h = imgmsg.height
            w = imgmsg.width
        else:
            img = cv_bridge.compressed_imgmsg_to_cv2(imgmsg, "rgb8")
            h, w = img.shape[:2]
        if h == 0 or w == 0:
            self.log.debug("Image has invalid shape!")
            return

        detsmsg.boxes = convert_bboxes(
            detsmsg.boxes,
            to_type=self.cfg.bbox_type,
            normalize=self.cfg.normalize_bbox,
            img_wh=(w, h),
        )

        self._bbox_pub.publish(detsmsg)


def main(args=None):
    if __name__ == "__main__" and args is None:
        args = sys.argv

    try:
        rclpy.init(args=args)

        node = Node(NODE_NAME)

        cfg = BBoxConverterCfg()
        BBoxConverter(node, cfg)

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
