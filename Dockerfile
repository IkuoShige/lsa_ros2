# ROS 2 Lang-SAM Dockerfile
# Based on lang-segment-anything Dockerfile
FROM nvidia/cuda:12.4.1-cudnn-devel-ubuntu22.04

# Set environment variables
ENV DEBIAN_FRONTEND=noninteractive
ENV PYTHONUNBUFFERED=1
ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8
ENV ROS_DISTRO=humble

# Install necessary packages
RUN apt-get update && apt-get install -y \
    python3 \
    python3-pip \
    python3-dev \
    openssh-client \
    build-essential \
    git \
    curl \
    wget \
    gnupg2 \
    lsb-release \
    software-properties-common \
    && rm -rf /var/lib/apt/lists/*

# Setup sources.list for ROS 2 Humble
RUN curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg \
    && echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(lsb_release -cs) main" > /etc/apt/sources.list.d/ros2.list

# Install ROS 2 Humble
RUN apt-get update && apt-get install -y \
    ros-humble-desktop \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool \
    libgl1-mesa-glx \
    libgl1-mesa-dri \
    libglu1-mesa \
    mesa-utils \
    libxrender1 \
    libxtst6 \
    libxi6 \
    && rm -rf /var/lib/apt/lists/*

# Initialize rosdep
RUN rosdep init && rosdep update

# Setup Python environment
RUN pip install --upgrade pip && \
    pip install \
    "numpy<2" \
    torch==2.4.1 \
    torchvision==0.19.1 \
    --extra-index-url https://download.pytorch.org/whl/cu124

RUN pip install \
    groundingdino-py==0.1.0 \
    opencv-python \
    pillow \
    lark

# Clone lang-segment-anything source code
WORKDIR /opt
RUN git clone https://github.com/luca-medeiros/lang-segment-anything.git

# Modify python version requirement in lang-segment-anything to work with Python 3.10
RUN if [ -f /opt/lang-segment-anything/pyproject.toml ]; then \
    sed -i 's/>=3.11/>=3.10/' /opt/lang-segment-anything/pyproject.toml; \
    else \
    echo "No pyproject.toml found or no Python version specified"; \
    fi

# Add lang-segment-anything to PYTHONPATH
ENV PYTHONPATH=$PYTHONPATH:/opt/lang-segment-anything
ENV LD_LIBRARY_PATH=$LD_LIBRARY_PATH:/usr/local/cuda/lib64
ENV PATH=$PATH:/usr/local/cuda/bin

# # Modify python version requirement in lang-segment-anything
# RUN if [ -f /opt/lang-segment-anything/pyproject.toml ]; then \
#     sed -i 's/>=3.11/>=3.10/' /opt/lang-segment-anything/pyproject.toml; \
#     else \
#     echo "No pyproject.toml found or no Python version specified"; \
#     fi

# # Add lang-segment-anything to PYTHONPATH
# ENV PYTHONPATH=$PYTHONPATH:/opt/lang-segment-anything

# Create ROS 2 workspace
WORKDIR /ros2_ws
RUN mkdir -p /ros2_ws/src

# Copy ros2_lang_sam packages
COPY ./ros2_lang_sam /ros2_ws/src/ros2_lang_sam
COPY ./ros2_lang_sam_msgs /ros2_ws/src/ros2_lang_sam_msgs

# Install Python dependencies for ros2_lang_sam
RUN pip install -r /ros2_ws/src/ros2_lang_sam/requirements.txt

# Build ROS 2 packages
RUN . /opt/ros/humble/setup.sh && \
    cd /ros2_ws && \
    rm -rf build/ install/ log/ && \
    colcon build --symlink-install

# Setup ROS 2 environment
RUN echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc && \
    echo 'source /ros2_ws/install/setup.bash' >> ~/.bashrc && \
    echo 'export PYTHONPATH=$PYTHONPATH:/opt/lang-segment-anything' >> ~/.bashrc

# Setup demo script
COPY ./test_ros2_lang_sam.sh /ros2_ws/
RUN chmod +x /ros2_ws/test_ros2_lang_sam.sh

# Create a directory for sample images
RUN mkdir -p /ros2_ws/sample_images

# Copy demo images
RUN mkdir -p /tmp/sample_images

COPY ./assets/car.jpeg /tmp/sample_images/
COPY ./assets/person.jpg /tmp/sample_images/

RUN cp -r /tmp/sample_images/* /ros2_ws/sample_images/

# Setup entrypoint script
COPY ./docker_entrypoint.sh /
RUN chmod +x /docker_entrypoint.sh

# Expose ports for ROS 2
EXPOSE 9090

# Set working directory
WORKDIR /ros2_ws

# Set entrypoint
ENTRYPOINT ["/docker_entrypoint.sh"]
CMD ["bash"]
