# timestamp & coordinate frame id (aka camera id or stream id)
# NOTE: should be same header as image frame. timestamp is of frame, not prediction
std_msgs/Header header

nicefaces/BBox2D pred

float32 score
string label

# TODO: Create msg for FeatureData
# FeatureData contains feature vector, binary string, etc values for associating identity

# TODO: Record down when prediction started & ended for profiling purposes