#!/bin/bash
# テスト用スクリプト for ROS 2 Lang-SAM

set -e

# システム情報を表示
echo "=== システム情報 ==="
echo "OS: $(cat /etc/os-release | grep PRETTY_NAME | cut -d= -f2 | tr -d '"')"
echo "Python: $(python3 --version)"
echo "OpenCV: $(python3 -c "import cv2; print(cv2.__version__)" 2>/dev/null || echo "Not installed")"
echo "Disk space: $(df -h / | tail -1 | awk '{print $4}') available"
echo "Current working directory: $(pwd)"
echo "Current user: $(whoami)"
echo "==================="

# ROS 2環境をセットアップ
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# ROS 2 Lang-SAMサーバを起動（バックグラウンド）
echo "ROS 2 Lang-SAMサーバを起動しています..."
ros2 launch ros2_lang_sam server.launch.py device:=cuda &
SERVER_PID=$!

# サーバが起動するまで少し待機
sleep 10

# 結果保存用のディレクトリを作成
mkdir -p /ros2_ws/results

# テスト画像を使用してクライアントを実行
echo "クライアントテストを実行しています..."
ros2 run ros2_lang_sam lang_sam_client_node --image /ros2_ws/sample_images/car.jpeg --prompt "wheel" --output /ros2_ws/results/result_car.jpg

# 別のテスト画像で試す
echo "別のテスト例を実行しています..."
ros2 run ros2_lang_sam lang_sam_client_node --image /ros2_ws/sample_images/person.jpg --prompt "person" --output /ros2_ws/results/result_person.jpg

# サーバプロセスを終了
if [ -n "$SERVER_PID" ]; then
  echo "ROS 2 Lang-SAMサーバを停止しています..."
  kill $SERVER_PID
fi

# 結果の確認
echo "テスト完了！"
echo "サーバを再度起動する場合: ros2 launch ros2_lang_sam server.launch.py"
echo "クライアントの実行例: ros2 run ros2_lang_sam lang_sam_client_node --image /path/to/image.jpg --prompt \"object\""
