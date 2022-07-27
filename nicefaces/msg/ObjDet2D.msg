# timestamp & coordinate frame id (aka camera id or stream id)
# NOTE: should be same header as image frame. timestamp is of frame, not prediction
std_msgs/Header header

nicefaces/BBox2D box

float32 score
string label
nicefaces/TrackData track