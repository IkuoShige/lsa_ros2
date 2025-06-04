"""
Real-time segmentation node for ROS 2.
This node subscribes to image topics and text prompts, then publishes segmentation results.
"""

import cv2
import numpy as np
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from ros2_lang_sam.lang_sam import LangSAM
from ros2_lang_sam_msgs.msg import SegmentationRequest, SegmentationResult


class SegmentationNode(Node):
    """
    ROS 2 node for real-time text-prompted object segmentation.
    """

    def __init__(self, node_name: str = "segmentation_node") -> None:
        """
        Initialize the segmentation node.
        
        Args:
            node_name: Name of the node (default: "segmentation_node")
        """
        super().__init__(node_name)
        self.get_logger().info("Starting Segmentation Node...")
        
        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("sam_type", "sam2.1_hiera_small"),
                ("checkpoint_path", ""),
                ("device", "cuda"),
                ("box_threshold", 0.3),
                ("text_threshold", 0.25),
                ("image_topic", "/camera/image_raw"),
                ("request_topic", "/segmentation/request"),
                ("result_topic", "/segmentation/result"),
            ],
        )
        
        # Get parameters
        self._sam_type = self.get_parameter("sam_type").value
        self._checkpoint_path = self.get_parameter("checkpoint_path").value
        self._device = self.get_parameter("device").value
        self._box_threshold = self.get_parameter("box_threshold").value
        self._text_threshold = self.get_parameter("text_threshold").value
        self._image_topic = self.get_parameter("image_topic").value
        self._request_topic = self.get_parameter("request_topic").value
        self._result_topic = self.get_parameter("result_topic").value
        
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
        
        # Store latest image
        self._latest_image = None
        self._latest_image_header = None
        
        # Create subscribers
        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            10
        )
        
        self._request_sub = self.create_subscription(
            SegmentationRequest,
            self._request_topic,
            self._on_segmentation_request,
            10
        )
        
        # Create publisher
        self._result_pub = self.create_publisher(
            SegmentationResult,
            self._result_topic,
            10
        )
        
        self.get_logger().info(f"Segmentation Node is ready.")
        self.get_logger().info(f"  - Image topic: {self._image_topic}")
        self.get_logger().info(f"  - Request topic: {self._request_topic}")
        self.get_logger().info(f"  - Result topic: {self._result_topic}")
    
    def _on_image(self, msg: Image) -> None:
        """
        Callback for image messages.
        
        Args:
            msg: Image message
        """
        self._latest_image = msg
        self._latest_image_header = msg.header
    
    def _on_segmentation_request(self, msg: SegmentationRequest) -> None:
        """
        Callback for segmentation request messages.
        
        Args:
            msg: SegmentationRequest message
        """
        self.get_logger().info(f"Received segmentation request with prompt: '{msg.text_prompt}'")
        
        # Use image from request if provided, otherwise use latest subscribed image
        if msg.image.height > 0 and msg.image.width > 0:
            image_msg = msg.image
            header = msg.image.header
        elif self._latest_image is not None:
            image_msg = self._latest_image
            header = self._latest_image_header
        else:
            self.get_logger().warn("No image available for segmentation")
            return
        
        try:
            # Convert ROS Image to OpenCV image
            img = self._bridge.imgmsg_to_cv2(image_msg, desired_encoding="rgb8")
            
            # Get box and text thresholds from request or use default values
            box_threshold = msg.box_threshold if msg.box_threshold > 0.0 else self._box_threshold
            text_threshold = msg.text_threshold if msg.text_threshold > 0.0 else self._text_threshold
            
            self.get_logger().info(
                f"Segmenting image of shape {img.shape} with text prompt: '{msg.text_prompt}'"
            )
            
            # Measure performance
            start = self.get_clock().now().nanoseconds
            
            # Perform segmentation
            result = self._lang_sam.segment(
                img, 
                msg.text_prompt,
                box_threshold=box_threshold,
                text_threshold=text_threshold
            )
            
            self.get_logger().info(
                f"Segmentation completed in {round((self.get_clock().now().nanoseconds - start)/1.e9, 2)}s."
            )
            
            # Create result message
            result_msg = SegmentationResult()
            result_msg.header = header
            result_msg.text_prompt = msg.text_prompt
            
            # Extract masks, boxes and scores from result
            masks = result.get("masks", [])
            boxes = result.get("boxes", [])
            scores = result.get("scores", [])
            
            # Convert masks to ROS Image messages
            result_msg.masks = [
                self._bridge.cv2_to_imgmsg(mask.astype(np.uint8), encoding="mono8") 
                for mask in masks
            ]
            
            # Convert boxes to ROS RegionOfInterest messages
            result_msg.boxes = []
            for box in boxes:
                from sensor_msgs.msg import RegionOfInterest
                roi = RegionOfInterest()
                roi.x_offset = int(box[0])
                roi.y_offset = int(box[1])
                roi.width = int(box[2] - box[0])
                roi.height = int(box[3] - box[1])
                result_msg.boxes.append(roi)
            
            # Set scores
            result_msg.scores = scores.tolist() if isinstance(scores, np.ndarray) else scores
            
            # Publish result
            self._result_pub.publish(result_msg)
            self.get_logger().info(f"Published segmentation result with {len(result_msg.masks)} masks")
            
        except Exception as e:
            error_msg = f"Error during segmentation: {e}"
            self.get_logger().error(error_msg)


def main(args=None):
    """Main entry point for the segmentation node."""
    rclpy.init(args=args)
    node = SegmentationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 