"""
LangSAM server node for ROS 2.
This node provides a service for text-prompted object segmentation using LangSAM.
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node

from ros2_lang_sam.lang_sam import LangSAM
from ros2_lang_sam_msgs.srv import TextSegmentation


class LangSAMServer(Node):
    """
    ROS 2 server node for text-prompted object segmentation using LangSAM.
    """

    def __init__(self, node_name: str = "lang_sam_server") -> None:
        """
        Initialize the LangSAM server node.
        
        Args:
            node_name: Name of the node (default: "lang_sam_server")
        """
        super().__init__(node_name)
        self.get_logger().info("Starting LangSAM server...")
        
        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("sam_type", "sam2.1_hiera_small"),
                ("checkpoint_path", ""),
                ("device", "cuda"),
                ("box_threshold", 0.3),
                ("text_threshold", 0.25),
            ],
        )
        
        # Get parameters
        self._sam_type = self.get_parameter("sam_type").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value
        self._device = self.get_parameter("device").value
        self._box_threshold = self.get_parameter("box_threshold").value
        self._text_threshold = self.get_parameter("text_threshold").value
        
        # Initialize CV bridge
        self._bridge = CvBridge()
        
        # Load LangSAM model
        self.get_logger().info(
            f"Loading LangSAM model '{self._sam_type}' on device '{self._device}'. This may take some time..."
        )
        
        try:
            self._lang_sam = LangSAM(
                sam_type=self._sam_type,
                ckpt_path=self._checkpoint_path if self._checkpoint_path else None,
                device=self._device,
            )
            self.get_logger().info("LangSAM model loaded successfully.")
        except Exception as e:
            error_msg = f"Failed to load LangSAM model: {e}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
        
        # Create service
        self._segment_service = self.create_service(
            TextSegmentation, "~/text_segment", self._on_text_segment
        )
        
        self.get_logger().info("LangSAM server is ready.")
    
    def _on_text_segment(
        self, request: TextSegmentation.Request, response: TextSegmentation.Response
    ) -> TextSegmentation.Response:
        """
        Process text segmentation service request.
        
        Args:
            request: Service request containing image and text prompt
            response: Service response to be filled
            
        Returns:
            Filled service response
        """
        self.get_logger().info(f"Received text segmentation request with prompt: '{request.text_prompt}'")
        
        try:
            # Convert ROS Image to OpenCV image
            img = self._bridge.imgmsg_to_cv2(request.image, desired_encoding="rgb8")
            
            # Get box and text thresholds from request or use default values
            box_threshold = request.box_threshold if request.box_threshold > 0.0 else self._box_threshold
            text_threshold = request.text_threshold if request.text_threshold > 0.0 else self._text_threshold
            
            self.get_logger().info(
                f"Segmenting image of shape {img.shape} with text prompt: '{request.text_prompt}'"
            )
            
            # Measure performance
            start = self.get_clock().now().nanoseconds
            
            # Perform segmentation
            result = self._lang_sam.segment(
                img, 
                request.text_prompt,
                box_threshold=box_threshold,
                text_threshold=text_threshold
            )
            
            self.get_logger().info(
                f"Segmentation completed in {round((self.get_clock().now().nanoseconds - start)/1.e9, 2)}s."
            )
            
            # Extract masks, boxes and scores from result
            masks = result.get("masks", [])
            boxes = result.get("boxes", [])
            scores = result.get("scores", [])
            
            # Convert masks to ROS Image messages
            response.masks = [
                self._bridge.cv2_to_imgmsg(mask.astype(np.uint8), encoding="mono8") 
                for mask in masks
            ]
            
            # Convert boxes to ROS RegionOfInterest messages
            response.boxes = []
            for box in boxes:
                from sensor_msgs.msg import RegionOfInterest
                roi = RegionOfInterest()
                roi.x_offset = int(box[0])
                roi.y_offset = int(box[1])
                roi.width = int(box[2] - box[0])
                roi.height = int(box[3] - box[1])
                response.boxes.append(roi)
            
            # Set scores
            response.scores = scores.tolist() if isinstance(scores, np.ndarray) else scores
            
            return response
            
        except Exception as e:
            error_msg = f"Error during text segmentation: {e}"
            self.get_logger().error(error_msg)
            raise RuntimeError(error_msg)
