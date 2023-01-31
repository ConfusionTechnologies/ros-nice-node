# ros-nice-node

To generate documentation:

```sh
cd ros-nice-node/nice_node_py/docs
sphinx-apidoc -f -0 . ../
```

See <https://kanishkvarshney.medium.com/python-documentation-generating-html-using-sphinx-a0d909f5e963> for how to setup auto-generated documentation via Sphinx for other ROS packages.

This repository contains 3 packages that forms a framework on top of ROS2. The goal is to:

- Provide common/base features for nodes
- Centralize utility functions & tools
- Provide a more powerful & easy API

## (DEPRECATED) nicepynode

Python utilities and ROS2 node wrapper.

## (NEW) nice_node_py

Python ROS2 Node Wrapper inspired by the API of Python web servers and utilities.

## (NEW) nicefaces_utils_py

Message conversion and other utilities regarding message types.

## (TBD) nicecppnode

C++ utilities and ROS2 node wrapper. However, I am not familiar enough with the C++ api (plus my C++ is quite rusty), to judge whether this would actually be necessary.

## nicefaces

Utility message, service & action types that cannot be found in <https://github.com/ros2/common_interfaces> or other similar utility interface packages. Certain aspects of DDS Design means some programmer comforts are unavailable like template/wrapper types... (Sorry, no monads, no matter how much you feel they make sense here)

Yes, I am aware <http://wiki.ros.org/vision_msgs> exists. But I already wrote these, and mine are more lax.

I created another series, (name)s, that splits out each array in order to minimize the looping needed in Python... (Yes the performance hit is real when dealing with 10+ poses each with 133 keypoints)

### BBox2D

Interface for a 2D bounding box. It may be used as a replacement for `sensor_msgs/RegionOfInterest`. Features:

- `XYXY`, `XYWH` & `CBOX` (XY of centre + width & height) mode
- Using normalized (relative) coordinates

### BBox2DArray

It exists.

### TrackData

Interface for tracking detections (e.g. ObjDet2D, Wholebody, etc). Should contain additional data (i.e. Feature Embeddings) for associating tracks across frames. TrackData also has an `id` property which contains the track id of the object (an empty string indicates the object is not being tracked).

### ProfilingData

Profiling and timing stuff.

### ObjDet2D

Interface for a 2D Object Detection from most common 2D Object Detection Computer Vision models. Features:

- `header` property for synchronization via [`message_filters`](https://github.com/ros2/message_filters)
- Optional `feature` property for associating identity using identifiers such as feature vectors

### ObjDet2DArray

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

### AnyLabel

Escape hatch interface for annotations via encoding as a string. Features:

- `header` property for synchronization

### AnyLabelArray

Array of AnyLabel for convenience. Features:

- `header` property for synchronization
- Timestamps for when inference started and ended or profiling
