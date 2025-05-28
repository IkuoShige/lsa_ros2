#!/bin/bash
# ROS 2 Lang-SAMコンテナ内でのビルドスクリプト

set -e

echo "=== ROS 2 Lang-SAMをビルドしています ==="
cd /ros2_ws/src/ros2_lang_sam

# パッケージの依存関係をインストール
echo "Python依存関係をインストールしています..."
pip install -r requirements.txt

# ROS 2ワークスペースに戻ってビルド
cd /ros2_ws
echo "ROS 2パッケージをビルドしています..."
colcon build --symlink-install

# 環境をセットアップ
echo "ROS 2環境をセットアップしています..."
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

echo "ビルド完了！"
echo "テストを実行するには："
echo "  ./test_ros2_lang_sam.sh または ./comprehensive_test.sh"
