# ROS 2 Lang-SAM

テキストプロンプトによる物体セグメンテーションをROS 2で実現するパッケージです。[Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything) をROS 2環境で使用するためのラッパーを提供します。

## 概要

このパッケージは、テキスト入力から画像内の対象物をセグメンテーションする機能を提供します。SAM（Segment Anything Model）とGroundedDinoを組み合わせ、画像と自然言語の入力からマスク生成を行います。

**主な機能:**
- テキストプロンプトによる物体セグメンテーション
- ROS 2サービスインターフェース
- 実環境でのリアルタイムセグメンテーション
- 結果の可視化と保存

## Quick start: Docker

```bash
docker build -t lsa_ros2 .
./run_docker_dev.sh
./rebuild_in_docker.sh
./test_ros2_lang_sam.sh
```

## インストール方法

### 前提条件

- ROS 2 Humble Hawksbill
- Python 3.10以上
- CUDA対応GPU（推奨、CPU実行も可能）

### 依存パッケージ

- OpenCV
- NumPy
- PyTorch
- ROS 2基本パッケージ
- lang-segment-anything

### インストール手順

1. Clone:

```bash
git clone https://github.com/IkuoShige/lsa_ros2.git
```

2. ROS 2ワークスペースのsrcにコピー:

```bash
cp -r ./ros2_lang_sam /path/to/your/ros2_ws/src/ros2_lang_sam
cp -r ./ros2_lang_sam_msgs /path/to/your/ros2_ws/src/ros2_lang_sam_msgs
```

3. 依存パッケージのインストール:

```bash
pip3 install -r ros2_lang_sam/ros2_lang_sam/requirements.txt
```

4. パッケージのビルド:

```bash
cd /path/to/your/ros2_ws
colcon build --symlink-install --packages-select ros2_lang_sam ros2_lang_sam_msgs
```

5. 環境のセットアップ:

```bash
source /path/to/your/ros2_ws/install/setup.bash
```

## 使用方法

### 1. サービスベースの使用方法

#### サーバーの起動

Lang-SAMサーバーを起動するには:

```bash
ros2 launch ros2_lang_sam server.launch.py device:=cuda
```

#### クライアントの実行

コマンドライン引数でクライアントを実行:

```bash
ros2 run ros2_lang_sam lang_sam_client_node --image /path/to/image.jpg --prompt "car" --output /path/to/result.jpg
```

### 2. 実環境でのリアルタイムセグメンテーション

#### 基本的な実行

実環境でカメラ画像をsubscribeしてセグメンテーションを行う：

```bash
# セグメンテーションノードとビジュアライザーを起動
ros2 launch ros2_lang_sam segmentation_demo.launch.py device:=cuda image_topic:=/camera/image_raw
```

#### rqt_image_viewを使用した統合デモ

rqt_image_viewで元画像と結果画像を自動的に並べて表示：

```bash
# 画像パブリッシャー 
ros2 run ros2_lang_sam image_publisher_node --ros-args -p image_path:=./sample_images/car.jpeg -p publish_rate:=1.0
# リクエストパブリッシャー
ros2 run ros2_lang_sam request_publisher_node --ros-args -p text_prompts:="['wheel']"
# セグメンテーションシステムとrqt_image_viewを同時に起動
ros2 launch ros2_lang_sam segmentation_rqt_demo.launch.py device:=cuda image_topic:=/camera/image_raw
```

このlaunchファイルは以下を同時に起動します：
- セグメンテーションノード
- 結果ビジュアライザーノード
- rqt GUI（事前設定された横並びレイアウト）

**自動横並び画像表示：**
- rqt GUIが起動すると、自動的に2つのImage Viewが横並びで表示されます
- **左側**: `/camera/image_raw`（元画像）が事前に設定済み
- **右側**: `/segmentation/visualization`（セグメンテーション結果）が事前に設定済み
- 手動設定は不要で、すぐに画像比較が可能
- Topic monitorの`/segmentation/result`にチェックを入れて広げると, text_prompt(`string`)とscores(`sequence<float>`)

**手動でトピックを変更したい場合：**
1. 各Image Viewの上部ドロップダウンでトピックを変更可能

**利点：**
- **完全自動化**: 起動と同時に理想的なレイアウトが表示
- **即座に比較開始**: 手動設定なしで画像比較が可能
- **1つのウィンドウ**: 複数の画像を効率的に表示
- **高速表示**: 軽量で応答性の高い画像表示
- **操作性**: ズーム、パンなどの基本操作が可能

#### Rviz2を使用した可視化

Rviz2でセグメンテーション結果を可視化：

```bash
# セグメンテーションシステムをRviz2と一緒に起動
ros2 launch ros2_lang_sam segmentation_rviz_demo.launch.py device:=cuda image_topic:=/camera/image_raw
```

このlaunchファイルは以下を同時に起動します：
- セグメンテーションノード
- 結果ビジュアライザーノード（Rviz2対応）
- Rviz2（設定済み）

**Rviz2で表示される可視化要素：**
- **Image**: OpenCVベースの可視化画像（`/segmentation/visualization`）
- **MarkerArray**: バウンディングボックスと信頼度スコア（`/segmentation/markers`）

別のターミナルで、セグメンテーションリクエストを送信：

```bash
ros2 run ros2_lang_sam request_publisher_node --ros-args -p text_prompts:="['person', 'car', 'chair']"
```

#### テスト環境での実行

画像ファイルを使用してテスト：

```bash
# 単一画像ファイルでテスト
ros2 launch ros2_lang_sam test_segmentation.launch.py device:=cuda image_path:=/path/to/test_image.jpg

# 画像ディレクトリでテスト
ros2 launch ros2_lang_sam test_segmentation.launch.py device:=cuda image_directory:=/path/to/test_images/

# カメラでテスト
ros2 launch ros2_lang_sam test_segmentation.launch.py device:=cuda use_camera:=true
```

#### 個別ノードの実行

各ノードを個別に実行することも可能：

```bash
# 画像パブリッシャー
ros2 run ros2_lang_sam image_publisher_node --ros-args -p image_path:=/path/to/image.jpg -p publish_rate:=1.0

# セグメンテーションノード
ros2 run ros2_lang_sam segmentation_node --ros-args -p device:=cuda -p image_topic:=/camera/image_raw

# リクエストパブリッシャー
ros2 run ros2_lang_sam request_publisher_node --ros-args -p text_prompts:="['person', 'car']" -p publish_rate:=0.5

# 結果ビジュアライザー
ros2 run ros2_lang_sam result_visualizer_node --ros-args -p save_results:=true -p output_directory:=/tmp/results
```

### パラメータ

**サーバーパラメータ:**
- `sam_type`: SAMモデルのタイプ (デフォルト: "sam2.1_hiera_small")
- `checkpoint_path`: チェックポイントファイルのパス（空の場合は自動ダウンロード）
- `device`: 推論デバイス (デフォルト: "cuda")
- `box_threshold`: ボックス予測の閾値 (デフォルト: 0.3)
- `text_threshold`: テキスト予測の閾値 (デフォルト: 0.25)

**セグメンテーションノードパラメータ:**
- `image_topic`: 入力画像トピック (デフォルト: "/camera/image_raw")
- `request_topic`: セグメンテーションリクエストトピック (デフォルト: "/segmentation/request")
- `result_topic`: セグメンテーション結果トピック (デフォルト: "/segmentation/result")

**画像パブリッシャーパラメータ:**
- `image_path`: 単一画像ファイルのパス
- `image_directory`: 画像ディレクトリのパス
- `use_camera`: カメラを使用するかどうか (デフォルト: false)
- `publish_rate`: パブリッシュ頻度 (Hz)

**結果ビジュアライザーパラメータ:**
- `save_results`: 結果を保存するかどうか (デフォルト: false)
- `output_directory`: 保存先ディレクトリ
- `show_boxes`: バウンディングボックスを表示するかどうか
- `show_scores`: 信頼度スコアを表示するかどうか
- `enable_rviz`: Rviz2可視化を有効にするかどうか (デフォルト: true)
- `frame_id`: Rviz2用のフレームID (デフォルト: "camera_link")
- `marker_topic`: マーカートピック (デフォルト: "/segmentation/markers")

**クライアントパラメータ:**
- `--image`: 入力画像ファイルのパス
- `--prompt`: セグメンテーション対象を説明するテキスト
- `--box-threshold`: ボックス予測の閾値 (デフォルト: 0.3)
- `--text-threshold`: テキスト予測の閾値 (デフォルト: 0.25)
- `--output`: 結果画像の保存先パス
- `--debug`: デバッグログを有効化

## メッセージとサービス

### サービス

#### TextSegmentation サービス

**リクエスト:**
- `image`: 入力画像 (RGB形式)
- `text_prompt`: セグメンテーション対象を説明するテキスト
- `box_threshold`: ボックス予測の閾値
- `text_threshold`: テキスト予測の閾値

**レスポンス:**
- `masks`: セグメンテーションマスクの配列
- `boxes`: 境界ボックスの配列
- `scores`: 検出スコアの配列

### メッセージ

#### SegmentationRequest メッセージ

- `image`: 入力画像
- `text_prompt`: テキストプロンプト
- `box_threshold`: ボックス予測の閾値
- `text_threshold`: テキスト予測の閾値

#### SegmentationResult メッセージ

- `header`: ヘッダー情報
- `masks`: セグメンテーションマスクの配列
- `boxes`: 境界ボックスの配列
- `scores`: 検出スコアの配列
- `text_prompt`: 使用されたテキストプロンプト

## トピック構成

実環境でのトピック構成例：

```
/camera/image_raw              → 入力画像
/segmentation/request          → セグメンテーションリクエスト
/segmentation/result           → セグメンテーション結果
/segmentation/visualization    → 可視化された結果
```

## トラブルシューティング

### 一般的な問題

**Q: サーバーがCUDAエラーで起動しない**
A: GPUメモリ不足の可能性があります。他のプロセスを終了するか、`device:=cpu`オプションを使用してください。

**Q: セグメンテーション結果が適切でない**
A: `box_threshold`と`text_threshold`パラメータを調整してみてください。

**Q: 画像保存に失敗する**
A: 出力ディレクトリの書き込み権限を確認してください。絶対パスの使用を推奨します。

**Q: リアルタイム処理が遅い**
A: GPU使用を確認し、画像サイズやパブリッシュ頻度を調整してください。

### ログとデバッグ

詳細なログを有効にするには、`--debug`フラグを使用します:

```bash
ros2 run ros2_lang_sam lang_sam_client_node --image input.jpg --prompt "person" --output result.jpg --debug
```

実環境ノードのデバッグ：

```bash
ros2 run ros2_lang_sam segmentation_node --ros-args --log-level DEBUG
```

## 開発情報

### ファイル構成

- `lsa_ros2/`: メインパッケージ
  - `ros2_lang_sam/`: Python実装モジュール
    - `lang_sam_server_node.py`: サーバーノードメイン
    - `lang_sam_server.py`: サーバー実装
    - `lang_sam_client_node.py`: クライアントノードメイン
    - `lang_sam_client.py`: クライアント実装
    - `segmentation_node.py`: 実環境セグメンテーションノード
    - `image_publisher_node.py`: 画像パブリッシャーノード
    - `result_visualizer_node.py`: 結果ビジュアライザーノード
    - `request_publisher_node.py`: リクエストパブリッシャーノード
  - `launch/`: launchファイル
    - `server.launch.py`: サーバー起動用
    - `segmentation_demo.launch.py`: 実環境デモ用
    - `segmentation_rviz_demo.launch.py`: Rviz2付きデモ用
    - `segmentation_rqt_demo.launch.py`: rqt_image_view付きデモ用
    - `test_segmentation.launch.py`: テスト用
  - `data/`: サンプルデータ
- `ros2_lang_sam_msgs/`: メッセージ定義パッケージ
  - `msg/`: メッセージ定義
    - `SegmentationRequest.msg`: リクエストメッセージ
    - `SegmentationResult.msg`: 結果メッセージ
  - `srv/`: サービス定義
    - `TextSegmentation.srv`: セグメンテーションサービス

### 関連プロジェクト

- [Lang-SAM (言語セグメンテーション)](https://github.com/luca-medeiros/lang-segment-anything)
- [Segment Anything Model (SAM)](https://segment-anything.com)
- [GroundedDINO](https://github.com/IDEA-Research/GroundingDINO)

## ライセンス

このプロジェクトはApache License 2.0の下で配布されています。

## 謝辞

- Lang-SAMプロジェクトの開発者
- SAMおよびGroundedDINOの開発者
- ROS 2コミュニティ 