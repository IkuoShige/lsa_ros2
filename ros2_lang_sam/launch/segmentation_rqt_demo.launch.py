#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Get the package path
    package_dir = get_package_share_directory('ros2_lang_sam')
    perspective_path = os.path.join(package_dir, 'config', 'segment_visualization.perspective')
    # Launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to use for inference (cuda/cpu)'
    )

    sam_type_arg = DeclareLaunchArgument(
        'sam_type',
        default_value='sam2.1_hiera_large',
        description='SAM model type to use'
    )
    
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='Input image topic'
    )
    
    # Get launch configurations
    device = LaunchConfiguration('device')
    sam_type = LaunchConfiguration('sam_type')
    image_topic = LaunchConfiguration('image_topic')
    
    # Segmentation node
    segmentation_node = Node(
        package='ros2_lang_sam',
        executable='segmentation_node',
        name='segmentation_node',
        parameters=[{
            'device': device,
            'sam_type': sam_type,
            'image_topic': image_topic,
        }],
        output='screen'
    )
    
    # Result visualizer node
    result_visualizer_node = Node(
        package='ros2_lang_sam',
        executable='result_visualizer_node',
        name='result_visualizer_node',
        parameters=[{
            'save_results': False,
            'show_boxes': True,
            'show_scores': True,
            'enable_rviz': False,  # Disable Rviz since we're using rqt_image_view
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
        image_topic_arg,
        segmentation_node,
        result_visualizer_node,
        rqt_multiview,
    ])
