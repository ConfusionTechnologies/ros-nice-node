from nicefaces.msg import BBox2D
from copy import copy


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
        b = box.box
        if box.type == BBox2D.XYXY:
            if to_type == BBox2D.XYWH:
                box.box = (b[0], b[1], b[2] - b[0], b[3] - b[1])
            elif to_type == BBox2D.CBOX:
                box.box = (
                    (b[0] + b[2]) / 2,
                    (b[1] + b[3]) / 2,
                    b[2] - b[0],
                    b[3] - b[1],
                )
            else:
                raise AssertionError("Invalid to_type")
        elif box.type == BBox2D.XYWH:
            if to_type == BBox2D.XYXY:
                box.box = (b[0], b[1], b[0] + b[2], b[1] + b[3])
            elif to_type == BBox2D.CBOX:
                box.box = (b[0] + b[2] / 2, b[1] + b[3] / 2, b[2], b[3])
            else:
                raise AssertionError("Invalid to_type")
        elif box.type == BBox2D.CBOX:
            if to_type == BBox2D.XYWH:
                box.box = (b[0] - b[2] / 2, b[1] - b[3] / 2, b[2], b[3])
            elif to_type == BBox2D.XYXY:
                box.box = (
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
            box.box = (b[0] / w, b[1] / h, b[2] / w, b[3] / h)
        else:
            box.box = (b[0] * w, b[1] * h, b[2] * w, b[3] * h)

    if not inplace:
        return box

