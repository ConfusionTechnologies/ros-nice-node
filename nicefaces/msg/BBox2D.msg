# I felt common_interfaces' RegionOfInterest.msg was insufficient

uint8 XYXY=0  # top-left point followed by bottom-right
uint8 XYWH=1  # top-left point followed by width & height
uint8 CBOX=2  # center point followed by width & height

bool is_norm false

uint8 type 0

float32[4] rect

# TODO: do you want to add quaternion property for skewed bboxes?