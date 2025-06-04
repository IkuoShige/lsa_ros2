"""
Image publisher node for testing segmentation.
This node publishes images from files or a camera to test the segmentation node.
"""

import cv2
import os
import time
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Header


class ImagePublisherNode(Node):
    """
    ROS 2 node for publishing images for testing segmentation.
    """

    def __init__(self, node_name: str = "image_publisher_node") -> None:
        """
        Initialize the image publisher node.
        
        Args:
            node_name: Name of the node (default: "image_publisher_node")
        """
        super().__init__(node_name)
        self.get_logger().info("Starting Image Publisher Node...")
        
        # Declare parameters
        self.declare_parameters(
            namespace="",
            parameters=[
                ("image_path", ""),
                ("image_topic", "/camera/image_raw"),
                ("publish_rate", 1.0),
                ("use_camera", False),
                ("camera_index", 0),
                ("image_directory", ""),
                ("loop_images", True),
            ],
        )
        
        # Get parameters
        self._image_path = self.get_parameter("image_path").value
        self._image_topic = self.get_parameter("image_topic").value
        self._publish_rate = self.get_parameter("publish_rate").value
        self._use_camera = self.get_parameter("use_camera").value
        self._camera_index = self.get_parameter("camera_index").value
        self._image_directory = self.get_parameter("image_directory").value
        self._loop_images = self.get_parameter("loop_images").value
        
        # Initialize CV bridge
        self._bridge = CvBridge()
        
        # Create publisher
        self._image_pub = self.create_publisher(
            Image,
            self._image_topic,
            10
        )
        
        # Initialize image source
        self._image_list = []
        self._current_image_index = 0
        self._cap = None
        
        if self._use_camera:
            self._init_camera()
        else:
            self._init_image_files()
        
        # Create timer for publishing
        self._timer = self.create_timer(
            1.0 / self._publish_rate,
            self._publish_image
        )
        
        self.get_logger().info(f"Image Publisher Node is ready.")
        self.get_logger().info(f"  - Publishing to: {self._image_topic}")
        self.get_logger().info(f"  - Rate: {self._publish_rate} Hz")
        if self._use_camera:
            self.get_logger().info(f"  - Source: Camera {self._camera_index}")
        else:
            self.get_logger().info(f"  - Source: {len(self._image_list)} image files")
    
    def _init_camera(self) -> None:
        """Initialize camera capture."""
        try:
            self._cap = cv2.VideoCapture(self._camera_index)
            if not self._cap.isOpened():
                raise RuntimeError(f"Failed to open camera {self._camera_index}")
            self.get_logger().info(f"Camera {self._camera_index} opened successfully")
        except Exception as e:
            self.get_logger().error(f"Failed to initialize camera: {e}")
            raise
    
    def _init_image_files(self) -> None:
        """Initialize image file list."""
        if self._image_path:
            # Single image file
            if os.path.isfile(self._image_path):
                self._image_list = [self._image_path]
            else:
                self.get_logger().error(f"Image file not found: {self._image_path}")
                return
        elif self._image_directory:
            # Directory of images
            if os.path.isdir(self._image_directory):
                supported_formats = ('.jpg', '.jpeg', '.png', '.bmp', '.tiff', '.tif')
                self._image_list = [
                    os.path.join(self._image_directory, f)
                    for f in os.listdir(self._image_directory)
                    if f.lower().endswith(supported_formats)
                ]
                self._image_list.sort()
            else:
                self.get_logger().error(f"Image directory not found: {self._image_directory}")
                return
        else:
            self.get_logger().error("No image source specified. Set either image_path or image_directory parameter.")
            return
        
        if not self._image_list:
            self.get_logger().error("No valid image files found")
            return
        
        self.get_logger().info(f"Found {len(self._image_list)} image files")
        for img_path in self._image_list:
            self.get_logger().info(f"  - {img_path}")
    
    def _publish_image(self) -> None:
        """Publish an image."""
        try:
            img = None
            
            if self._use_camera:
                # Read from camera
                ret, img = self._cap.read()
                if not ret:
                    self.get_logger().warn("Failed to read from camera")
                    return
                # Convert BGR to RGB
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            else:
                # Read from file
                if not self._image_list:
                    return
                
                image_path = self._image_list[self._current_image_index]
                img = cv2.imread(image_path)
                if img is None:
                    self.get_logger().warn(f"Failed to load image: {image_path}")
                    self._next_image()
                    return
                
                # Convert BGR to RGB
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                # Move to next image
                self._next_image()
            
            if img is not None:
                # Create ROS Image message
                header = Header()
                header.stamp = self.get_clock().now().to_msg()
                header.frame_id = "camera_link"
                
                img_msg = self._bridge.cv2_to_imgmsg(img, encoding="rgb8")
                img_msg.header = header
                
                # Publish image
                self._image_pub.publish(img_msg)
                
                if not self._use_camera:
                    self.get_logger().info(f"Published image: {self._image_list[self._current_image_index - 1]}")
                
        except Exception as e:
            self.get_logger().error(f"Error publishing image: {e}")
    
    def _next_image(self) -> None:
        """Move to next image in the list."""
        self._current_image_index += 1
        if self._current_image_index >= len(self._image_list):
            if self._loop_images:
                self._current_image_index = 0
            else:
                self.get_logger().info("Finished publishing all images")
                self._timer.cancel()
    
    def destroy_node(self) -> None:
        """Clean up resources."""
        if self._cap is not None:
            self._cap.release()
        super().destroy_node()


def main(args=None):
    """Main entry point for the image publisher node."""
    rclpy.init(args=args)
    node = ImagePublisherNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main() 