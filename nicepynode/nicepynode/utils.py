from __future__ import annotations

import json
from array import array
from collections import deque
from copy import copy, deepcopy
from dataclasses import asdict, is_dataclass
from typing import Any, Mapping, Sequence

import cv2
import numpy as np
from nicefaces.msg import BBox2D, BBox2Ds
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles

# Realtime Profile: don't bog down publisher when model is slow
RT_SUB_PROFILE = copy(QoSPresetProfiles.SENSOR_DATA.value)
RT_SUB_PROFILE.depth = 0

# Realtime Profile: don't wait for slow subscribers
RT_PUB_PROFILE = copy(QoSPresetProfiles.SENSOR_DATA.value)
RT_PUB_PROFILE.depth = 15


class Symbol:
    def __init__(self, name=""):
        self.name = f"<symbol {name}>"

    def __repr__(self):
        return self.name


def append_array(arr: array, np_arr: np.ndarray, dtype=np.float32):
    """Assign to builtin array from numpy array, avoiding copy where possible."""
    arr.frombytes(np_arr.astype(dtype, order="C", copy=False).data.cast("b"))
    return arr


def to_dot_notation(*keys: Sequence[str]):
    return ".".join(k for k in keys if k != "")


def should_jsonify(v):
    return not isinstance(v, str) and (
        isinstance(v, Sequence) or isinstance(v, Mapping)
    )


def declare_parameters_from_dataclass(
    node: Node, obj: Any, namespace="", exclude_keys=[]
):
    """Recursively declare parameters based on a Dataclass. 
    Recursion is done when the Dataclass attribute is another Dataclass. 
    However, if the attribute is a `Sequence` (i.e. `list` or `tuple`) or `Mapping` (i.e. `dict`), 
    it will be transparently converted to JSON (Ignores) to allow easy editing.

    Args:
        node (Node): Node to declare parameters on.
        obj (Any): Instance of a dataclass.
        namespace (str, optional): Parameter namespace. Defaults to "".
        exclude_keys (list, optional): Keys to exclude using dot notation (i.e. `foo.bar`). Defaults to [].
    """
    assert is_dataclass(obj) and not isinstance(
        obj, type
    ), "Must use instance of a dataclass!"

    # exclude default declared parameters
    exclude_keys += ["rate", "use_sim_time"]

    # params to declare
    params = []
    for k, v in asdict(obj).items():
        ns = to_dot_notation(namespace, k)
        if ns in exclude_keys:
            continue

        # recursively declare parameters if is dataclass
        if is_dataclass(v):
            declare_parameters_from_dataclass(
                node, v, namespace=ns, exclude_keys=exclude_keys
            )
            continue
        # convert list & dicts to str for easy editing externally
        elif should_jsonify(v):
            v = json.dumps(v, skipkeys=True, sort_keys=True)

        params.append((k, v))

    node.declare_parameters(namespace, params)


def dataclass_from_parameters(cls: Any, cfgdict: dict):
    """Reconstructs Dataclass from dict of parameters.
    This is because ros Nodes use dot notation for parameters.
    Dataclass must not have any positional arguments.
    Use instance of Dataclass if aiming to merge configs (i.e apply changes.)
    """
    obj = cls() if isinstance(cls, type) else deepcopy(cls)

    for k, v in cfgdict.items():
        path = deque(k.split("."))

        nested = obj
        while len(path) > 1:
            nested = getattr(nested, path.popleft())

        # check if it was list or dict & convert back
        attr = getattr(nested, path[0])
        if should_jsonify(attr):
            v = json.loads(v)

        setattr(nested, path[0], v)

    return obj


# ensure images fit input shape and stride requirements
# btw img size must be multiple of stride in both directions
# for example:
# # imgs is array of cv2 imread HWC BGR images
# batch = np.stack([letterbox(x)[0] for x in imgs], 0)
# batch[..., ::-1].transpose(0, 3, 1, 2) # NCHW, RGB
# (dw, dh) is size of padding (one side-only so need x2)
# ratio (also w, h) is resizing ratio
# above 2 are needed to reconstruct the original image
# they are hence useless
# taken from https://github.com/ultralytics/yolov5/blob/master/utils/augmentations.py
def letterbox(
    im,
    new_shape=(640, 640),
    color=(114, 114, 114),
    auto=True,
    scaleFill=False,
    scaleup=True,
    stride=32,
):
    # Resize and pad image while meeting stride-multiple constraints
    oh, ow = im.shape[:2]  # current shape [height, width]
    nw, nh = new_shape

    # Scale ratio (new / old)
    r = min(nw / ow, nh / oh)
    if not scaleup:  # only scale down, do not scale up (for better val mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = round(ow * r), round(oh * r)
    dw, dh = nw - new_unpad[0], nh - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, stride), np.mod(dh, stride)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (nw, nh)
        ratio = (nw / ow, nh / oh)  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if (ow, oh) != new_unpad:  # resize
        im = cv2.resize(im, new_unpad, interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    im = cv2.copyMakeBorder(
        im, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color
    )  # add border
    return im, ratio, (dw, dh)


def convert_bbox(box: BBox2D, to_type=None, normalize=None, img_wh=None, inplace=True):
    if not inplace:
        # NOTE: cannot deepcopy ROS2 msg for some reason
        box = copy(box)
    if not to_type is None and box.type != to_type:
        # as not deepcopy, have to be careful
        b = box.rect
        if box.type == BBox2D.XYXY:
            if to_type == BBox2D.XYWH:
                box.rect = (b[0], b[1], b[2] - b[0], b[3] - b[1])
            elif to_type == BBox2D.CBOX:
                box.rect = (
                    (b[0] + b[2]) / 2,
                    (b[1] + b[3]) / 2,
                    b[2] - b[0],
                    b[3] - b[1],
                )
            else:
                raise AssertionError("Invalid to_type")
        elif box.type == BBox2D.XYWH:
            if to_type == BBox2D.XYXY:
                box.rect = (b[0], b[1], b[0] + b[2], b[1] + b[3])
            elif to_type == BBox2D.CBOX:
                box.rect = (b[0] + b[2] / 2, b[1] + b[3] / 2, b[2], b[3])
            else:
                raise AssertionError("Invalid to_type")
        elif box.type == BBox2D.CBOX:
            if to_type == BBox2D.XYWH:
                box.rect = (b[0] - b[2] / 2, b[1] - b[3] / 2, b[2], b[3])
            elif to_type == BBox2D.XYXY:
                box.rect = (
                    b[0] - b[2] / 2,
                    b[1] - b[3] / 2,
                    b[0] + b[2] / 2,
                    b[1] + b[3] / 2,
                )
            else:
                raise AssertionError("Invalid to_type")
        else:
            raise AssertionError("BBox2D has invalid type")
        box.type = to_type

    if not normalize is None and box.is_norm != normalize:
        assert img_wh, "img_wh cannot be None if normalize is True"
        w, h = img_wh
        if normalize:
            box.rect = (b[0] / w, b[1] / h, b[2] / w, b[3] / h)
        else:
            box.rect = (b[0] * w, b[1] * h, b[2] * w, b[3] * h)
        box.is_norm = normalize

    return box


# Used internally for convert_bboxes below as an alias. boxes is float32 btw.
_C = lambda a: np.frombuffer(a, dtype=np.float32)


def convert_bboxes(
    boxes: BBox2Ds, to_type=None, normalize=None, img_wh=None, inplace=True
):
    if not inplace:
        # NOTE: cannot deepcopy ROS2 msg for some reason
        boxes = copy(boxes)

    if not to_type is None and boxes.type != to_type:
        # as not deepcopy, have to be careful
        if boxes.type == BBox2D.XYXY:
            if to_type == BBox2D.XYWH:
                pass
            elif to_type == BBox2D.CBOX:
                boxes.a = append_array(array("f"), (_C(boxes.a) + _C(boxes.c)) / 2)
                boxes.b = append_array(array("f"), (_C(boxes.b) + _C(boxes.d)) / 2)
            else:
                raise AssertionError("Invalid to_type")

            boxes.c = append_array(array("f"), _C(boxes.c) - _C(boxes.a))
            boxes.d = append_array(array("f"), _C(boxes.d) - _C(boxes.b))

        elif boxes.type == BBox2D.XYWH:
            if to_type == BBox2D.XYXY:
                boxes.c = append_array(array("f"), _C(boxes.a) + _C(boxes.c))
                boxes.d = append_array(array("f"), _C(boxes.b) + _C(boxes.d))
            elif to_type == BBox2D.CBOX:
                boxes.a = append_array(array("f"), _C(boxes.a) + _C(boxes.c) / 2)
                boxes.b = append_array(array("f"), _C(boxes.b) + _C(boxes.d) / 2)
            else:
                raise AssertionError("Invalid to_type")
        elif boxes.type == BBox2D.CBOX:
            if to_type == BBox2D.XYWH:
                pass
            elif to_type == BBox2D.XYXY:
                boxes.c = append_array(array("f"), _C(boxes.a) + _C(boxes.c) / 2)
                boxes.d = append_array(array("f"), _C(boxes.b) + _C(boxes.d) / 2)
            else:
                raise AssertionError("Invalid to_type")

            boxes.a = append_array(array("f"), _C(boxes.a) - _C(boxes.c) / 2)
            boxes.b = append_array(array("f"), _C(boxes.b) - _C(boxes.d) / 2)

        else:
            raise AssertionError("BBox2D has invalid type")

        boxes.type = to_type

    if not normalize is None and boxes.is_norm != normalize:
        assert img_wh, "img_wh cannot be None if normalize is True"
        w, h = img_wh
        if normalize:
            boxes.a = append_array(array("f"), _C(boxes.a) / w)
            boxes.b = append_array(array("f"), _C(boxes.b) / h)
            boxes.c = append_array(array("f"), _C(boxes.c) / w)
            boxes.d = append_array(array("f"), _C(boxes.d) / h)
        else:
            boxes.a = append_array(array("f"), _C(boxes.a) * w)
            boxes.b = append_array(array("f"), _C(boxes.b) * h)
            boxes.c = append_array(array("f"), _C(boxes.c) * w)
            boxes.d = append_array(array("f"), _C(boxes.d) * h)

        boxes.is_norm = normalize

    return boxes


# TODO: BBox to points & vice-versa

# TODO: Conversion (name)Array <-> (name)s msgs
