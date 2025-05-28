# ROS 2 Lang-SAM

テキストプロンプトによる物体セグメンテーションをROS 2で実現するパッケージです。[Lang-SAM](https://github.com/luca-medeiros/lang-segment-anything) をROS 2環境で使用するためのラッパーを提供します。

## 概要

このパッケージは、テキスト入力から画像内の対象物をセグメンテーションする機能を提供します。SAM（Segment Anything Model）とGroundedDinoを組み合わせ、画像と自然言語の入力からマスク生成を行います。

**主な機能:**
- テキストプロンプトによる物体セグメンテーション
- ROS 2サービスインターフェース
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

### サーバーの起動

Lang-SAMサーバーを起動するには:

```bash
ros2 launch ros2_lang_sam server.launch.py device:=cuda
```

### クライアントの実行

コマンドライン引数でクライアントを実行:

```bash
ros2 run ros2_lang_sam lang_sam_client_node --image /path/to/image.jpg --prompt "car" --output /path/to/result.jpg
```

### パラメータ

**サーバーパラメータ:**
- `sam_type`: SAMモデルのタイプ (デフォルト: "sam2.1_hiera_small")
- `checkpoint_path`: チェックポイントファイルのパス（空の場合は自動ダウンロード）
- `device`: 推論デバイス (デフォルト: "cuda")
- `box_threshold`: ボックス予測の閾値 (デフォルト: 0.3)
- `text_threshold`: テキスト予測の閾値 (デフォルト: 0.25)

**クライアントパラメータ:**
- `--image`: 入力画像ファイルのパス
- `--prompt`: セグメンテーション対象を説明するテキスト
- `--box-threshold`: ボックス予測の閾値 (デフォルト: 0.3)
- `--text-threshold`: テキスト予測の閾値 (デフォルト: 0.25)
- `--output`: 結果画像の保存先パス
- `--debug`: デバッグログを有効化

## サービスインターフェース

このパッケージは以下のROS 2サービスを提供します:

### TextSegmentation サービス

**リクエスト:**
- `image`: 入力画像 (RGB形式)
- `text_prompt`: セグメンテーション対象を説明するテキスト
- `box_threshold`: ボックス予測の閾値
- `text_threshold`: テキスト予測の閾値

**レスポンス:**
- `masks`: セグメンテーションマスクの配列
- `boxes`: 境界ボックスの配列
- `scores`: 検出スコアの配列

## トラブルシューティング

### 一般的な問題

**Q: サーバーがCUDAエラーで起動しない**
A: GPUメモリ不足の可能性があります。他のプロセスを終了するか、`device:=cpu`オプションを使用してください。

**Q: セグメンテーション結果が適切でない**
A: `box_threshold`と`text_threshold`パラメータを調整してみてください。

**Q: 画像保存に失敗する**
A: 出力ディレクトリの書き込み権限を確認してください。絶対パスの使用を推奨します。

### ログとデバッグ

詳細なログを有効にするには、`--debug`フラグを使用します:

```bash
ros2 run ros2_lang_sam lang_sam_client_node --image input.jpg --prompt "person" --output result.jpg --debug
```

## 開発情報

### ファイル構成

- `lsa_ros2/`: メインパッケージ
  - `ros2_lang_sam/`: Python実装モジュール
    - `lang_sam_server_node.py`: サーバーノードメイン
    - `lang_sam_server.py`: サーバー実装
    - `lang_sam_client_node.py`: クライアントノードメイン
    - `lang_sam_client.py`: クライアント実装
  - `launch/`: launchファイル
  - `data/`: サンプルデータ
- `ros2_lang_sam_msgs/`: メッセージ定義パッケージ
  - `srv/`: サービス定義

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
