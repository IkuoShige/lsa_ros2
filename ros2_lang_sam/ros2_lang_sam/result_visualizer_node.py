"""
Segmentation result visualizer node.
This node subscribes to segmentation results and visualizes them.
Supports both OpenCV-based visualization and Rviz2 visualization.
"""

import cv2
import numpy as np
import os
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header, ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point

from ros2_lang_sam_msgs.msg import SegmentationResult
from ros2_lang_sam.utils import visualize_masks


class ResultVisualizerNode(Node):
    """
    ROS 2 node for visualizing segmentation results.
    """

    def __init__(self, node_name: str = "result_visualizer_node") -> None:
        """
        Initialize the result visualizer node.
        
        Args:
            node_name: Name of the node (default: "result_visualizer_node")
        """
        super().__init__(node_name)
        self.get_logger().info("Starting Result Visualizer Node...")
        
        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("result_topic", "/segmentation/result"),
                ("image_topic", "/camera/image_raw"),
                ("output_topic", "/segmentation/visualization"),
                ("save_results", False),
                ("output_directory", "/tmp/segmentation_results"),
                ("show_boxes", True),
                ("show_scores", True),
                ("enable_rviz", True),
                ("frame_id", "camera_link"),
                ("marker_topic", "/segmentation/markers"),
            ],
        )
        
        # Get parameters
        self._result_topic = self.get_parameter("result_topic").value
        self._image_topic = self.get_parameter("image_topic").value
        self._output_topic = self.get_parameter("output_topic").value
        self._save_results = self.get_parameter("save_results").value
        self._output_directory = self.get_parameter("output_directory").value
        self._show_boxes = self.get_parameter("show_boxes").value
        self._show_scores = self.get_parameter("show_scores").value
        self._enable_rviz = self.get_parameter("enable_rviz").value
        self._frame_id = self.get_parameter("frame_id").value
        self._marker_topic = self.get_parameter("marker_topic").value
        
        # Initialize CV bridge
        self._bridge = CvBridge()
        
        # Store latest image
        self._latest_image = None
        
        # Create output directory if needed
        if self._save_results:
            os.makedirs(self._output_directory, exist_ok=True)
            self.get_logger().info(f"Saving results to: {self._output_directory}")
        
        # Create subscribers
        self._result_sub = self.create_subscription(
            SegmentationResult,
            self._result_topic,
            self._on_segmentation_result,
            10
        )
        
        self._image_sub = self.create_subscription(
            Image,
            self._image_topic,
            self._on_image,
            10
        )
        
        # Create publisher for visualization
        self._vis_pub = self.create_publisher(
            Image,
            self._output_topic,
            10
        )
        
        # Create Rviz2 publishers if enabled
        if self._enable_rviz:
            self._marker_pub = self.create_publisher(
                MarkerArray,
                self._marker_topic,
                10
            )
        
        self.get_logger().info(f"Result Visualizer Node is ready.")
        self.get_logger().info(f"  - Result topic: {self._result_topic}")
        self.get_logger().info(f"  - Image topic: {self._image_topic}")
        self.get_logger().info(f"  - Output topic: {self._output_topic}")
        if self._enable_rviz:
            self.get_logger().info(f"  - Rviz2 enabled:")
            self.get_logger().info(f"    - Marker topic: {self._marker_topic}")
            self.get_logger().info(f"    - Frame ID: {self._frame_id}")
    
    def _on_image(self, msg: Image) -> None:
        """
        Callback for image messages.
        
        Args:
            msg: Image message
        """
        self._latest_image = msg
    
    def _create_bounding_box_markers(self, boxes, scores, text_prompt, header):
        """
        Create MarkerArray for bounding boxes visualization in Rviz2.
        
        Args:
            boxes: Array of bounding boxes [x1, y1, x2, y2]
            scores: Array of confidence scores
            text_prompt: Text prompt used for segmentation
            header: Header for the markers
            
        Returns:
            MarkerArray message
        """
        marker_array = MarkerArray()
        
        for i, (box, score) in enumerate(zip(boxes, scores)):
            # Create line strip marker for bounding box
            marker = Marker()
            marker.header = header
            marker.id = i
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.pose.orientation.w = 1.0
            marker.scale.x = 0.02  # Line width
            
            # Set color based on score (red to green)
            marker.color.r = float(1.0 - score)
            marker.color.g = float(score)
            marker.color.b = 0.0
            marker.color.a = 0.8
            
            # Create box points (assuming z=0 for 2D image)
            x1, y1, x2, y2 = box
            # Convert pixel coordinates to meters (assuming 1 pixel = 0.001 meters)
            scale = 0.001
            points = [
                Point(x=x1*scale, y=y1*scale, z=0.0),
                Point(x=x2*scale, y=y1*scale, z=0.0),
                Point(x=x2*scale, y=y2*scale, z=0.0),
                Point(x=x1*scale, y=y2*scale, z=0.0),
                Point(x=x1*scale, y=y1*scale, z=0.0),  # Close the box
            ]
            marker.points = points
            marker_array.markers.append(marker)
            
            # Create text marker for score and prompt
            if self._show_scores:
                text_marker = Marker()
                text_marker.header = header
                text_marker.id = i + 1000  # Offset ID to avoid conflicts
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = (x1 + x2) * 0.5 * scale
                text_marker.pose.position.y = y1 * scale - 0.05
                text_marker.pose.position.z = 0.0
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.05  # Text height
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.color.a = 1.0
                text_marker.text = f"{text_prompt}: {score:.2f}"
                marker_array.markers.append(text_marker)
        
        return marker_array
    
    def _on_segmentation_result(self, msg: SegmentationResult) -> None:
        """
        Callback for segmentation result messages.
        
        Args:
            msg: SegmentationResult message
        """
        self.get_logger().info(
            f"Received segmentation result for prompt: '{msg.text_prompt}' "
            f"with {len(msg.masks)} masks"
        )
        
        try:
            # Get the base image
            base_image = None
            image_shape = None
            if self._latest_image is not None:
                base_image = self._bridge.imgmsg_to_cv2(self._latest_image, desired_encoding="rgb8")
                image_shape = base_image.shape
            
            # Convert masks to numpy arrays
            masks = []
            for mask_msg in msg.masks:
                mask = self._bridge.imgmsg_to_cv2(mask_msg, desired_encoding="mono8")
                masks.append(mask.astype(bool))
            
            # Convert boxes to numpy arrays
            boxes = []
            for box in msg.boxes:
                x1 = box.x_offset
                y1 = box.y_offset
                x2 = x1 + box.width
                y2 = y1 + box.height
                boxes.append([x1, y1, x2, y2])
            boxes = np.array(boxes) if boxes else np.array([]).reshape(0, 4)
            
            # Get scores
            scores = np.array(msg.scores) if msg.scores else np.array([])
            
            # Create header for Rviz2 messages
            rviz_header = Header()
            rviz_header.stamp = msg.header.stamp
            rviz_header.frame_id = self._frame_id
            
            # Publish Rviz2 visualizations if enabled
            if self._enable_rviz and len(masks) > 0:
                # Publish bounding box markers
                if len(boxes) > 0 and len(scores) > 0:
                    marker_array = self._create_bounding_box_markers(
                        boxes, scores, msg.text_prompt, rviz_header
                    )
                    self._marker_pub.publish(marker_array)
                
                self.get_logger().info(f"Published Rviz2 visualizations with {len(masks)} masks")
            
            # Create OpenCV visualization
            if base_image is not None and len(masks) > 0:
                # Only pass boxes and scores if the respective show flags are enabled
                display_boxes = boxes if self._show_boxes else None
                display_scores = scores if self._show_scores else None
                
                vis_image = visualize_masks(
                    base_image,
                    masks,
                    boxes=display_boxes,
                    scores=display_scores
                )
                
                # Add text prompt to visualization
                text = f"Prompt: {msg.text_prompt}"
                cv2.putText(
                    vis_image, text, (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2
                )
                
                # Publish visualization
                vis_msg = self._bridge.cv2_to_imgmsg(vis_image, encoding="rgb8")
                vis_msg.header = msg.header
                self._vis_pub.publish(vis_msg)
                
                # Save result if requested
                if self._save_results:
                    timestamp = self.get_clock().now().nanoseconds
                    filename = f"segmentation_{timestamp}_{msg.text_prompt.replace(' ', '_')}.jpg"
                    filepath = os.path.join(self._output_directory, filename)
                    
                    # Convert RGB to BGR for saving
                    vis_bgr = cv2.cvtColor(vis_image, cv2.COLOR_RGB2BGR)
                    cv2.imwrite(filepath, vis_bgr)
                    self.get_logger().info(f"Saved visualization: {filepath}")
                
                self.get_logger().info(f"Published OpenCV visualization with {len(masks)} masks")
            else:
                self.get_logger().warn("No base image or masks available for visualization")
                
        except Exception as e:
            self.get_logger().error(f"Error visualizing segmentation result: {e}")
            import traceback
            self.get_logger().error(traceback.format_exc())


def main(args=None):
    """Main entry point for the result visualizer node."""
    rclpy.init(args=args)
    node = ResultVisualizerNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 