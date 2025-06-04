# SegmentationRequest message for topic communication
sensor_msgs/Image        image            # Input image
string                   text_prompt      # Text prompt for object detection
float32                  box_threshold    # Threshold for box predictions (default: 0.3)
float32                  text_threshold   # Threshold for text predictions (default: 0.25) 