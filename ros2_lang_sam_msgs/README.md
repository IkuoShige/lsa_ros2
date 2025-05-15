# ros2_lang_sam_msgs

ROS 2 Lang-SAMパッケージで使用するメッセージとサービスの定義パッケージです。

## 概要

このパッケージは、テキストベースのセグメンテーションサービスで使用するサービス定義を提供します。

## サービス定義

### TextSegmentation.srv

テキストプロンプトを使用して画像の対象物をセグメンテーションするためのサービス定義です。

**リクエスト:**
- `sensor_msgs/Image image`: 入力画像 (RGB形式)
- `string text_prompt`: セグメンテーション対象を説明するテキスト
- `float32 box_threshold`: ボックス予測の閾値 (デフォルト: 0.3)
- `float32 text_threshold`: テキスト予測の閾値 (デフォルト: 0.25)

**レスポンス:**
- `sensor_msgs/Image[] masks`: セグメンテーションマスクの配列
- `sensor_msgs/RegionOfInterest[] boxes`: 境界ボックスの配列
- `float32[] scores`: 検出スコアの配列

## 使用方法

このパッケージは、`ros2_lang_sam`パッケージと一緒に使用されます。

### 依存パッケージ

- `std_msgs`
- `sensor_msgs`

## ビルド方法

```bash
cd /path/to/your/ros2_ws
colcon build --packages-select ros2_lang_sam_msgs
```
