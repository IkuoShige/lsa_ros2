"""
Launch file for testing segmentation with mock data.
This launch file starts all nodes needed for testing the segmentation pipeline.
"""

# import launch
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for segmentation test."""
    # Get the package path
    package_dir = get_package_share_directory('ros2_lang_sam')
    perspective_path = os.path.join(package_dir, 'config', 'segment_visualization.perspective')
    
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run inference on (cuda/cpu)'
    )

    sam_type_arg = DeclareLaunchArgument(
        'sam_type',
        default_value='sam2.1_hiera_small',
        description='SAM model type to use'
    )

    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    image_path_arg = DeclareLaunchArgument(
        'image_path',
        default_value='',
        description='Path to test image file'
    )
    
    image_directory_arg = DeclareLaunchArgument(
        'image_directory',
        default_value='',
        description='Path to directory with test images'
    )
    
    use_camera_arg = DeclareLaunchArgument(
        'use_camera',
        default_value='false',
        description='Use camera instead of image files'
    )
    
    # Image publisher node
    image_publisher_node = Node(
        package='ros2_lang_sam',
        executable='image_publisher_node',
        name='image_publisher_node',
        parameters=[{
            'image_path': LaunchConfiguration('image_path'),
            'image_directory': LaunchConfiguration('image_directory'),
            'use_camera': LaunchConfiguration('use_camera'),
            'image_topic': LaunchConfiguration('image_topic'),
            'publish_rate': 0.5,
            'loop_images': True,
        }],
        output='screen'
    )
    
    # Segmentation node
    segmentation_node = Node(
        package='ros2_lang_sam',
        executable='segmentation_node',
        name='segmentation_node',
        parameters=[{
            'device': LaunchConfiguration('device'),
            'sam_type': LaunchConfiguration('sam_type'),
            'image_topic': LaunchConfiguration('image_topic'),
            'request_topic': '/segmentation/request',
            'result_topic': '/segmentation/result',
        }],
        output='screen'
    )
    
    # Request publisher node
    request_publisher_node = Node(
        package='ros2_lang_sam',
        executable='request_publisher_node',
        name='request_publisher_node',
        parameters=[{
            'request_topic': '/segmentation/request',
            'text_prompts': ['car', 'person', 'dog', 'cat', 'chair'],
            'publish_rate': 0.2,
            'include_image': False,  # Use image from camera topic
        }],
        output='screen'
    )
    
    # Result visualizer node
    visualizer_node = Node(
        package='ros2_lang_sam',
        executable='result_visualizer_node',
        name='result_visualizer_node',
        parameters=[{
            'result_topic': '/segmentation/result',
            'image_topic': LaunchConfiguration('image_topic'),
            'output_topic': '/segmentation/visualization',
            'save_results': True,
            'output_directory': '/tmp/segmentation_results',
            'show_boxes': True,
            'show_scores': True,
        }],
        output='screen'
    )

    # Launch rqt with pre-configured perspective for dual image view
    rqt_multiview = ExecuteProcess(
        cmd=[
            'ros2', 'run', 'rqt_gui', 'rqt_gui', 
            '--perspective-file', 
            perspective_path
        ],
        name='rqt_image_comparison',
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        sam_type_arg,
        image_path_arg,
        image_directory_arg,
        use_camera_arg,
        image_publisher_node,
        segmentation_node,
        request_publisher_node,
        visualizer_node,
        rqt_multiview,
    ]) 