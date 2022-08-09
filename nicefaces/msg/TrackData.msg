# TODO: think about other types... what exactly is a binary string?
uint8 VECTOR=0  # TrackData exists as feature vector
uint8 type 0
byte[] data

# Track ID (set once message is tracked)
int32 id -1
string label

# Keypoints used to identify
float32[] x
float32[] y
float32[] scores