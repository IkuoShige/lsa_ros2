"""
Launch file for real-time segmentation demo.
This launch file starts all nodes needed for the segmentation demo.
"""

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for segmentation demo."""
    
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to run inference on (cuda/cpu)'
    )
    
    sam_type_arg = DeclareLaunchArgument(
        'sam_type',
        default_value='sam2.1_hiera_small',
        description='SAM model type'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
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
    
    # Result visualizer node
    visualizer_node = Node(
        package='ros2_lang_sam',
        executable='result_visualizer_node',
        name='result_visualizer_node',
        parameters=[{
            'result_topic': '/segmentation/result',
            'image_topic': LaunchConfiguration('image_topic'),
            'output_topic': '/segmentation/visualization',
            'save_results': False,
            'show_boxes': True,
            'show_scores': True,
        }],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        sam_type_arg,
        image_topic_arg,
        segmentation_node,
        visualizer_node,
    ]) 