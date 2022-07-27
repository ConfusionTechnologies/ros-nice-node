# TODO: think about other types... what exactly is a binary string?
uint8 VECTOR=0  # TrackData exists as feature vector
uint8 type 0
byte[] data

# Track ID (set once message is tracked)
string id

# Keypoints used to identify
geometry_msgs/Point[] keypoints
float32[] keypoint_scores