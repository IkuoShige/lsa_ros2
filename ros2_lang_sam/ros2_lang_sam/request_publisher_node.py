"""
Segmentation request publisher node.
This node publishes segmentation requests for testing.
"""

import cv2
import os
import time
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, String

from ros2_lang_sam_msgs.msg import SegmentationRequest


class RequestPublisherNode(Node):
    """
    ROS 2 node for publishing segmentation requests.
    """

    def __init__(self, node_name: str = "request_publisher_node") -> None:
        """
        Initialize the request publisher node.
        
        Args:
            node_name: Name of the node (default: "request_publisher_node")
        """
        super().__init__(node_name)
        self.get_logger().info("Starting Request Publisher Node...")
        
        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("request_topic", "/segmentation/request"),
                ("text_prompts", ["car", "person", "dog"]),
                ("publish_rate", 0.2),  # Lower rate for testing
                ("box_threshold", 0.3),
                ("text_threshold", 0.25),
                ("include_image", False),
                ("image_path", ""),
            ],
        )
        
        # Get parameters
        self._request_topic = self.get_parameter("request_topic").value
        self._text_prompts = self.get_parameter("text_prompts").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._box_threshold = self.get_parameter("box_threshold").value
        self._text_threshold = self.get_parameter("text_threshold").value
        self._include_image = self.get_parameter("include_image").value
        self._image_path = self.get_parameter("image_path").value
        
        # Initialize CV bridge
        self._bridge = CvBridge()
        
        # Current prompt index
        self._current_prompt_index = 0
        
        # Load image if specified
        self._image_msg = None
        if self._include_image and self._image_path and os.path.isfile(self._image_path):
            try:
                img = cv2.imread(self._image_path)
                if img is not None:
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    self._image_msg = self._bridge.cv2_to_imgmsg(img_rgb, encoding="rgb8")
                    self.get_logger().info(f"Loaded image: {self._image_path}")
                else:
                    self.get_logger().warn(f"Failed to load image: {self._image_path}")
            except Exception as e:
                self.get_logger().error(f"Error loading image: {e}")
        
        # Create publisher
        self._request_pub = self.create_publisher(
            SegmentationRequest,
            self._request_topic,
            10
        )
        
        # Create timer for publishing
        self._timer = self.create_timer(
            1.0 / self._publish_rate,
            self._publish_request
        )
        
        self.get_logger().info(f"Request Publisher Node is ready.")
        self.get_logger().info(f"  - Publishing to: {self._request_topic}")
        self.get_logger().info(f"  - Rate: {self._publish_rate} Hz")
        self.get_logger().info(f"  - Text prompts: {self._text_prompts}")
        self.get_logger().info(f"  - Include image: {self._include_image}")
    
    def _publish_request(self) -> None:
        """Publish a segmentation request."""
        try:
            # Create request message
            request_msg = SegmentationRequest()
            
            # Set text prompt (cycle through prompts)
            if self._text_prompts:
                request_msg.text_prompt = self._text_prompts[self._current_prompt_index]
                self._current_prompt_index = (self._current_prompt_index + 1) % len(self._text_prompts)
            else:
                request_msg.text_prompt = "object"
            
            # Set thresholds
            request_msg.box_threshold = self._box_threshold
            request_msg.text_threshold = self._text_threshold
            
            # Set image if available
            if self._image_msg is not None:
                request_msg.image = self._image_msg
                # Update header timestamp
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "camera_link"
                request_msg.image.header = header
            else:
                # Empty image message
                empty_image = Image()
                request_msg.image = empty_image
            
            # Publish request
            self._request_pub.publish(request_msg)
            self.get_logger().info(f"Published segmentation request: '{request_msg.text_prompt}'")
            
        except Exception as e:
            self.get_logger().error(f"Error publishing request: {e}")


def main(args=None):
    """Main entry point for the request publisher node."""
    rclpy.init(args=args)
    node = RequestPublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 