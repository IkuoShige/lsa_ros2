"""
Launch file for segmentation demo with Rviz2 visualization.
This launch file starts the segmentation system with Rviz2 for visualization.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('ros2_lang_sam')
    
    # Declare launch arguments
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device to use for inference (cuda or cpu)'
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
    
    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='camera_link',
        description='Frame ID for Rviz2 visualization'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Enable Rviz2 visualization'
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=os.path.join(pkg_dir, 'config', 'segmentation_visualization.rviz'),
        description='Path to Rviz2 configuration file'
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
        }],
        output='screen'
    )
    
    # Result visualizer node
    visualizer_node = Node(
        package='ros2_lang_sam',
        executable='result_visualizer_node',
        name='result_visualizer_node',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'frame_id': LaunchConfiguration('frame_id'),
            'enable_rviz': LaunchConfiguration('enable_rviz'),
        }],
        output='screen'
    )
    
    # Rviz2 node
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_config')],
        output='screen'
    )
    
    return LaunchDescription([
        device_arg,
        sam_type_arg,
        image_topic_arg,
        frame_id_arg,
        enable_rviz_arg,
        rviz_config_arg,
        segmentation_node,
        visualizer_node,
        rviz_node,
    ])
