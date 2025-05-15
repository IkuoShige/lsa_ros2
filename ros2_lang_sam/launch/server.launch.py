"""
Launch file for the LangSAM server node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    """
    Generate launch description for the LangSAM server node.
    """
    # Declare launch arguments
    sam_type_arg = DeclareLaunchArgument(
        "sam_type",
        default_value=TextSubstitution(text="sam2.1_hiera_small"),
        description="Type of SAM model to use"
    )
    
    checkpoint_path_arg = DeclareLaunchArgument(
        "checkpoint_path",
        default_value=TextSubstitution(text=""),
        description="Path to checkpoint file (default: None)"
    )
    
    device_arg = DeclareLaunchArgument(
        "device",
        default_value=TextSubstitution(text="cuda"),
        description="Device to run inference on (default: cuda)"
    )
    
    box_threshold_arg = DeclareLaunchArgument(
        "box_threshold",
        default_value=TextSubstitution(text="0.3"),
        description="Threshold for box predictions (default: 0.3)"
    )
    
    text_threshold_arg = DeclareLaunchArgument(
        "text_threshold",
        default_value=TextSubstitution(text="0.25"),
        description="Threshold for text predictions (default: 0.25)"
    )
    
    # Create node
    lang_sam_server_node = Node(
        package="ros2_lang_sam",
        executable="lang_sam_server_node",
        name="lang_sam_server",
        parameters=[{
            "sam_type": LaunchConfiguration("sam_type"),
            "checkpoint_path": LaunchConfiguration("checkpoint_path"),
            "device": LaunchConfiguration("device"),
            "box_threshold": LaunchConfiguration("box_threshold"),
            "text_threshold": LaunchConfiguration("text_threshold"),
        }],
        output="screen",
    )
    
    # Create launch description
    ld = LaunchDescription()
    
    # Add launch arguments
    ld.add_action(sam_type_arg)
    ld.add_action(checkpoint_path_arg)
    ld.add_action(device_arg)
    ld.add_action(box_threshold_arg)
    ld.add_action(text_threshold_arg)
    
    # Add node
    ld.add_action(lang_sam_server_node)
    
    return ld
