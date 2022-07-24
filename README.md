# ros-nice-node

This repository contains 3 packages that forms a framework on top of ROS2. The goal is to:

- Provide common/base features for nodes
- Centralize utility functions & tools
- Provide a more powerful & easy API

## nicepynode

Python utilities and ROS2 node wrapper.

## (TBD) nicecppnode

C++ utilities and ROS2 node wrapper.

## nicefaces

Utility message, service & action types that cannot be found in <https://github.com/ros2/common_interfaces> or other similar utility interface packages. Certain aspects of DDS Design means some programmer comforts are unavailable like template/wrapper types... (Sorry, no monads, no matter how much you feel they make sense here)

Yes, I am aware <http://wiki.ros.org/vision_msgs> exists. But I already wrote these, and mine are more lax.

### BBox2D

Interface for a 2D bounding box. It may be used as a replacement for `sensor_msgs/RegionOfInterest`. Features:

- `XYXY`, `XYWH` & `CBOX` (XY of centre + width & height) mode
- Using normalized (relative) coordinates

## FeatureData

Interface for data that may be used to associate identity.

### ObjDet2D

Interface for a 2D Object Detection from most common 2D Object Detection Computer Vision models. Features:

- `header` property for synchronization via [`message_filters`](https://github.com/ros2/message_filters)
- Optional `feature` property for associating identity using identifiers such as feature vectors

#### ObjDet2DArray

Array of ObjDet2D for convenience. Features:

- `header` property for synchronization
- Timestamps for when inference started and ended for profiling

### WholeBody

Interface for COCO WholeBody annotations. Features:

- `header` property for synchronization
- Uses `geometry_msgs/Point` for 2D/3D interoperability

### WholeBodyArray

Array of WholeBody annotations. Features:

- `header` property for synchronization
- Timestamps for when inference started and ended or profiling

## AnyLabel

Escape hatch interface for annotations via encoding as a string. Features:

- `header` property for synchronization

## AnyLabelArray

Array of AnyLabel for convenience. Features:

- `header` property for synchronization
- Timestamps for when inference started and ended or profiling
