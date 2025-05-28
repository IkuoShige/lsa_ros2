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

# Dockerコンテナを実行
# --mount オプションを使用してソースコードとテストスクリプトをマウント
docker run -it --rm \
    --gpus all \
    --network host \
    --name ros2_lang_sam_dev \
    -v "$HOST_SRC_DIR/ros2_lang_sam:$CONTAINER_SRC_DIR/ros2_lang_sam" \
    -v "$HOST_SRC_DIR/ros2_lang_sam_msgs:$CONTAINER_SRC_DIR/ros2_lang_sam_msgs" \
    -v "$HOST_RESULTS_DIR:$CONTAINER_RESULTS_DIR" \
    -v "$HOST_TEST_DIR/test_ros2_lang_sam.sh:/ros2_ws/test_ros2_lang_sam.sh" \
    -v "$HOST_TEST_DIR/rebuild_in_docker.sh:/ros2_ws/rebuild_in_docker.sh" \
    lsa_ros2:latest \
    bash

    # -v "$HOST_TEST_DIR/comprehensive_test.sh:/ros2_ws/comprehensive_test.sh" \
    # -v "$HOST_TEST_DIR/test_file_write.sh:/ros2_ws/test_file_write.sh" \
    # -v "$HOST_TEST_DIR/test_opencv_save.py:/ros2_ws/test_opencv_save.py" \
    # -v "$HOST_TEST_DIR/simple_lang_sam_test.py:/ros2_ws/simple_lang_sam_test.py" \
