# ROS 2 Lang-SAM

A ROS 2 package for text-prompted object segmentation. This package provides a wrapper for [Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything) to use it in a ROS 2 environment.

## Overview

This package provides functionality to segment objects in images based on text input. It combines SAM (Segment Anything Model) and GroundedDino to generate masks from images and natural language input.

**Key Features:**
- Text-prompted object segmentation
- ROS 2 service interface
- Result visualization and saving

## Installation

### Prerequisites

- ROS 2 Humble Hawksbill
- Python 3.10 or higher
- CUDA-capable GPU (recommended, CPU execution also possible)

### Dependencies

- OpenCV
- NumPy
- PyTorch
- ROS 2 base packages
- lang-segment-anything

### Installation Steps

1. Navigate to your ROS 2 workspace:

```bash
cd /path/to/your/ros2_ws/src
```

2. Clone this repository:

```bash
git clone https://github.com/your-username/ros2_lang_sam.git
```

3. Install the dependencies:

```bash
pip3 install -r ros2_lang_sam/ros2_lang_sam/requirements.txt
```

4. Build the package:

```bash
cd /path/to/your/ros2_ws
colcon build --packages-select ros2_lang_sam ros2_lang_sam_msgs
```

5. Setup the environment:

```bash
source /path/to/your/ros2_ws/install/setup.bash
```

## Usage

### Starting the Server

To start the Lang-SAM server:

```bash
ros2 launch ros2_lang_sam server.launch.py device:=cuda
```

### Running the Client

Run the client with command line arguments:

```bash
ros2 run ros2_lang_sam lang_sam_client_node --image /path/to/image.jpg --prompt "car" --output /path/to/result.jpg
```

### Parameters

**Server Parameters:**
- `sam_type`: Type of SAM model (default: "sam2.1_hiera_small")
- `checkpoint_path`: Path to checkpoint file (empty for automatic download)
- `device`: Inference device (default: "cuda")
- `box_threshold`: Threshold for box predictions (default: 0.3)
- `text_threshold`: Threshold for text predictions (default: 0.25)

**Client Parameters:**
- `--image`: Path to the input image file
- `--prompt`: Text describing the object to segment
- `--box-threshold`: Threshold for box predictions (default: 0.3)
- `--text-threshold`: Threshold for text predictions (default: 0.25)
- `--output`: Path to save the visualization result
- `--debug`: Enable debug logging

## Service Interface

This package provides the following ROS 2 service:

### TextSegmentation Service

**Request:**
- `image`: Input image (RGB format)
- `text_prompt`: Text describing the object to segment
- `box_threshold`: Threshold for box predictions
- `text_threshold`: Threshold for text predictions

**Response:**
- `masks`: Array of segmentation masks
- `boxes`: Array of bounding boxes
- `scores`: Array of detection scores

## Troubleshooting

### Common Issues

**Q: Server fails to start with CUDA error**
A: GPU memory may be insufficient. Try terminating other processes or use the `device:=cpu` option.

**Q: Segmentation results are not accurate**
A: Try adjusting the `box_threshold` and `text_threshold` parameters.

**Q: Failed to save image file**
A: Check write permissions for the output directory. Using absolute paths is recommended.

### Logging and Debugging

To enable detailed logging, use the `--debug` flag:

```bash
ros2 run ros2_lang_sam lang_sam_client_node --image input.jpg --prompt "person" --output result.jpg --debug
```

## Development Information

### File Structure

- `ros2_lang_sam/`: Main package
  - `ros2_lang_sam/`: Python implementation modules
    - `lang_sam_server_node.py`: Server node main
    - `lang_sam_server.py`: Server implementation
    - `lang_sam_client_node.py`: Client node main
    - `lang_sam_client.py`: Client implementation
  - `launch/`: Launch files
  - `data/`: Sample data
- `ros2_lang_sam_msgs/`: Message definition package
  - `srv/`: Service definitions

### Related Projects

- [Lang-SAM (Language Segmentation)](https://github.com/luca-medeiros/lang-segment-anything)
- [Segment Anything Model (SAM)](https://segment-anything.com)
- [GroundedDINO](https://github.com/IDEA-Research/GroundingDINO)

## License

This project is distributed under the Apache License 2.0.

## Acknowledgements

- Lang-SAM project developers
- SAM and GroundedDINO developers
- ROS 2 community
