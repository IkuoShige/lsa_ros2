# ros2_lang_sam_msgs

Message and service definitions package for ROS 2 Lang-SAM.

## Overview

This package provides service definitions used for text-based segmentation services.

## Service Definitions

### TextSegmentation.srv

Service definition for segmenting objects in images using text prompts.

**Request:**
- `sensor_msgs/Image image`: Input image (RGB format)
- `string text_prompt`: Text describing the object to segment
- `float32 box_threshold`: Threshold for box predictions (default: 0.3)
- `float32 text_threshold`: Threshold for text predictions (default: 0.25)

**Response:**
- `sensor_msgs/Image[] masks`: Array of segmentation masks
- `sensor_msgs/RegionOfInterest[] boxes`: Array of bounding boxes
- `float32[] scores`: Array of detection scores

## Usage

This package is used together with the `ros2_lang_sam` package.

### Dependencies

- `std_msgs`
- `sensor_msgs`

## Building

```bash
cd /path/to/your/ros2_ws
colcon build --packages-select ros2_lang_sam_msgs
```
