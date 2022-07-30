from nicefaces.msg import BBox2D, BBox2Ds
from copy import copy
import numpy as np


class Symbol:
    def __init__(self, name=""):
        self.name = f"<symbol {name}>"

    def __repr__(self):
        return self.name


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

    if not normalize is None and box.is_norm != normalize:
        assert img_wh, "img_wh cannot be None if normalize is True"
        w, h = img_wh
        if normalize:
            box.rect = (b[0] / w, b[1] / h, b[2] / w, b[3] / h)
        else:
            box.rect = (b[0] * w, b[1] * h, b[2] * w, b[3] * h)

    return box


C = np.frombuffer


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
                boxes.a = (C(boxes.a) + C(boxes.c)) / 2
                boxes.b = (C(boxes.b) + C(boxes.d)) / 2
            else:
                raise AssertionError("Invalid to_type")

            boxes.c = C(boxes.c) - C(boxes.a)
            boxes.d = C(boxes.d) - C(boxes.b)

        elif boxes.type == BBox2D.XYWH:
            if to_type == BBox2D.XYXY:
                boxes.c = C(boxes.a) + C(boxes.c)
                boxes.d = C(boxes.b) + C(boxes.d)
            elif to_type == BBox2D.CBOX:
                boxes.a = C(boxes.a) + C(boxes.c) / 2
                boxes.b = C(boxes.b) + C(boxes.d) / 2
            else:
                raise AssertionError("Invalid to_type")
        elif boxes.type == BBox2D.CBOX:
            if to_type == BBox2D.XYWH:
                pass
            elif to_type == BBox2D.XYXY:
                boxes.c = C(boxes.a) + C(boxes.c) / 2
                boxes.d = C(boxes.b) + C(boxes.d) / 2
            else:
                raise AssertionError("Invalid to_type")

            boxes.a = C(boxes.a) - C(boxes.c) / 2
            boxes.b = C(boxes.b) - C(boxes.d) / 2

        else:
            raise AssertionError("BBox2D has invalid type")

    if not normalize is None and boxes.is_norm != normalize:
        assert img_wh, "img_wh cannot be None if normalize is True"
        w, h = img_wh
        if normalize:
            boxes.a = C(boxes.a) / w
            boxes.b = C(boxes.b) / h
            boxes.c = C(boxes.c) / w
            boxes.d = C(boxes.d) / h
        else:
            boxes.a = C(boxes.a) * w
            boxes.b = C(boxes.b) * h
            boxes.c = C(boxes.c) * w
            boxes.d = C(boxes.d) * h

    return boxes


# TODO: BBox to points & vice-versa

# TODO: Conversion (name)Array <-> (name)s msgs
