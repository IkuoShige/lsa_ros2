"""
LangSAM client for ROS 2.
This module provides a client for the text-prompted object segmentation service.
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

from ros2_lang_sam_msgs.srv import TextSegmentation


class LangSAMClient(Node):
    """
    ROS 2 client for text-prompted object segmentation using LangSAM.
    """

    def __init__(self, node_name: str = "lang_sam_client") -> None:
        """
        Initialize the LangSAM client node.
        
        Args:
            node_name: Name of the node (default: "lang_sam_client")
        """
        super().__init__(node_name)
        
        # Initialize CV bridge
        self._bridge = CvBridge()
        
        # Create client for the text segmentation service
        self._client = self.create_client(
            TextSegmentation, "/lang_sam_server/text_segment"
        )
        
        # Wait for service to be available
        while not self._client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("Service not available, waiting...")
    
    def segment_image(
        self, 
        image: np.ndarray, 
        text_prompt: str, 
        box_threshold: float = 0.3, 
        text_threshold: float = 0.25
    ) -> dict:
        """
        Send a segmentation request to the server.
        
        Args:
            image: Input image as a numpy array (RGB format)
            text_prompt: Text prompt describing the object to segment
            box_threshold: Threshold for box predictions (default: 0.3)
            text_threshold: Threshold for text predictions (default: 0.25)
            
        Returns:
            Tuple of (result placeholder, future object)
            Note: The actual result will be available via future.result() after completion
        """
        # Create request
        request = TextSegmentation.Request()
        request.image = self._bridge.cv2_to_imgmsg(image, encoding="rgb8")
        request.text_prompt = text_prompt
        request.box_threshold = box_threshold
        request.text_threshold = text_threshold
        
        # Send request
        self.get_logger().info(f"Sending segmentation request with prompt: '{text_prompt}'")
        future = self._client.call_async(request)
        
        # Process result
        result = {
            "masks": [],
            "boxes": [],
            "scores": []
        }
        
        return result, future
    
    def process_segmentation_result(self, future):
        """
        Process the segmentation result from the server.
        
        Args:
            future: Future object from the service call
            
        Returns:
            Dictionary containing masks, bounding boxes, and scores
        """
        response = future.result()
        
        # Log response details
        self.get_logger().info(f"Received response with {len(response.masks)} masks, {len(response.boxes)} boxes, and {len(response.scores)} scores")
        
        # Convert masks to numpy arrays
        masks = [
            self._bridge.imgmsg_to_cv2(mask_msg) 
            for mask_msg in response.masks
        ]
        
        # Log mask details
        if masks:
            for i, mask in enumerate(masks):
                self.get_logger().info(f"Mask {i}: shape={mask.shape}, type={mask.dtype}, min={mask.min()}, max={mask.max()}")
        else:
            self.get_logger().warning("No masks received in response")
        boxes = [
            [
                box.x_offset,
                box.y_offset,
                box.x_offset + box.width,
                box.y_offset + box.height
            ]
            for box in response.boxes
        ]
        
        # Extract scores
        scores = response.scores
        
        return {
            "masks": masks,
            "boxes": boxes,
            "scores": scores
        }
    
    def visualize_result(self, image: np.ndarray, result: dict) -> np.ndarray:
        """
        Visualize the segmentation result on the input image.
        
        Args:
            image: Input image as a numpy array
            result: Segmentation result from process_segmentation_result
            
        Returns:
            Visualization image with masks and bounding boxes
        """
        # Create a copy of the input image
        vis_image = image.copy()
        
        # Draw bounding boxes
        for box, score in zip(result["boxes"], result["scores"]):
            x1, y1, x2, y2 = [int(coord) for coord in box]
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(
                vis_image,
                f"Score: {score:.2f}",
                (x1, y1 - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.5,
                (0, 255, 0),
                2
            )
        
        # Draw masks with random colors
        for mask in result["masks"]:
            # Generate random color
            color = np.random.randint(0, 255, 3).tolist()
            
            # Create colored mask
            colored_mask = np.zeros_like(vis_image)
            colored_mask[mask > 0] = color
            
            # Blend with original image
            alpha = 0.5
            vis_image = cv2.addWeighted(
                vis_image, 1.0, colored_mask, alpha, 0.0
            )
        
        return vis_image
