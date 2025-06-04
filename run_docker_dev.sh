#!/bin/bash
# ROS 2 Lang-SAMをボリュームマウントを使用して実行するスクリプト

# ホストのソースコードディレクトリ
HOST_SRC_DIR="$(pwd)"
# ホストの結果ディレクトリ
HOST_RESULTS_DIR="$(pwd)/results"
# ホストのテストスクリプトディレクトリ
HOST_TEST_DIR="$(pwd)"

# コンテナ内のディレクトリ
CONTAINER_SRC_DIR="/ros2_ws/src/"
CONTAINER_RESULTS_DIR="/ros2_ws/results"

# 必要なディレクトリが存在するか確認
mkdir -p "$HOST_RESULTS_DIR"
chmod 777 "$HOST_RESULTS_DIR"

echo "=== ROS 2 Lang-SAM Docker実行 (ボリュームマウント使用) ==="
echo "ホストのソースディレクトリ: $HOST_SRC_DIR"
echo "ホストの結果ディレクトリ: $HOST_RESULTS_DIR"
echo "コンテナの対応ディレクトリ: $CONTAINER_SRC_DIR, $CONTAINER_RESULTS_DIR"

export DISPLAY=:1
export XAUTHORITY=/run/user/1000/gdm/Xauthority

# Dockerコンテナを実行
# --mount オプションを使用してソースコードとテストスクリプトをマウント
docker run -it --rm \
    --gpus all \
    --network host \
    --name ros2_lang_sam_dev \
    --env="DISPLAY=$DISPLAY" \
    --env="QT_X11_NO_MITSHM=1" \
    --env="XAUTHORITY=$XAUTHORITY" \
    --env="NVIDIA_VISIBLE_DEVICES=all" \
    --env="NVIDIA_DRIVER_CAPABILITIES=all" \
    --mount type=bind,source=/dev,target=/dev \
    --mount type=bind,source="$XAUTHORITY",target="$XAUTHORITY",readonly \
    --mount type=bind,source="/tmp/.X11-unix",target="/tmp/.X11-unix",readonly \
    --runtime=nvidia \
    -v "$HOST_SRC_DIR/ros2_lang_sam:$CONTAINER_SRC_DIR/ros2_lang_sam" \
    -v "$HOST_SRC_DIR/ros2_lang_sam_msgs:$CONTAINER_SRC_DIR/ros2_lang_sam_msgs" \
    -v "$HOST_RESULTS_DIR:$CONTAINER_RESULTS_DIR" \
    -v "$HOST_TEST_DIR/test_ros2_lang_sam.sh:/ros2_ws/test_ros2_lang_sam.sh" \
    -v "$HOST_TEST_DIR/rebuild_in_docker.sh:/ros2_ws/rebuild_in_docker.sh" \
    -v "$HOST_TEST_DIR/test_real_time_segmentation.sh:/ros2_ws/test_real_time_segmentation.sh" \
    lsa_ros2:latest \
    bash

# docker run --gpus all -it --rm \
#   --privileged \
#   --net=host \
#   --ipc=host \
#   --env="DISPLAY=$DISPLAY" \
#   --env="QT_X11_NO_MITSHM=1" \
#   --env="XAUTHORITY=$XAUTHORITY" \
#   --env="NVIDIA_VISIBLE_DEVICES=all" \
#   --env="NVIDIA_DRIVER_CAPABILITIES=all" \
#   --mount type=bind,source=/dev,target=/dev \
#   --mount type=bind,source="$XAUTHORITY",target="$XAUTHORITY",readonly \
#   --mount type=bind,source="/tmp/.X11-unix",target="/tmp/.X11-unix",readonly \
#   -v "$(pwd):/home/ikuo/value_iteration2" \
#   --mount type=bind,source="/home/$USER/.ssh",target="/home/$USER/.ssh",readonly \
#   --runtime=nvidia \
#   --name zen_bartik \
#   cuda-ros2-humble:latest

