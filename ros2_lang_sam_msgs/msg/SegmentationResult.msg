# SegmentationResult message for topic communication
std_msgs/Header          header           # Header with timestamp and frame_id
sensor_msgs/Image[]      masks            # Generated masks for the detected objects
sensor_msgs/RegionOfInterest[] boxes      # Detected bounding boxes
float32[]                scores           # Confidence scores for the masks
string                   text_prompt      # Original text prompt used for segmentation 