#!/bin/bash
# Docker entrypoint for ROS 2 Lang-SAM

set -e

# ROS 2環境をセットアップ
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "ROS 2 Lang-SAM環境が準備完了しました。"
echo "テストスクリプトを実行するには: ./test_ros2_lang_sam.sh"
echo "サーバを起動するには: ros2 launch ros2_lang_sam server.launch.py"
echo "クライアントを実行するには: ros2 run ros2_lang_sam lang_sam_client_node --image /path/to/image.jpg --prompt \"object\""

exec "$@"
