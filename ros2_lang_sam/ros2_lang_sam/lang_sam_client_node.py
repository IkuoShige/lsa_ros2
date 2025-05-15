#!/usr/bin/env python3
"""
LangSAM client node script.
This script provides a simple command-line client for text-prompted object segmentation.
"""
import argparse
import logging
import os
import sys
import time
import traceback
from typing import List

import cv2
import numpy as np
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.task import Future

from ros2_lang_sam.lang_sam_client import LangSAMClient


def load_image(image_path, logger):
    """画像を読み込む"""
    logger.info(f"画像を読み込みます: {image_path}")
    if not os.path.exists(image_path):
        logger.error(f"画像が見つかりません: {image_path}")
        return None
    
    image = cv2.imread(image_path)
    if image is None:
        logger.error(f"画像の読み込みに失敗しました: {image_path}")
        return None
    
    logger.info(f"画像を読み込みました: 形状={image.shape}, 種類={image.dtype}")
    return image


def save_image(output_path, image, logger):
    """画像を保存する"""
    # 出力ディレクトリの作成
    output_dir = os.path.dirname(output_path)
    if output_dir and not os.path.exists(output_dir):
        os.makedirs(output_dir, exist_ok=True)
        logger.info(f"ディレクトリを作成しました: {output_dir}")
    
    # 絶対パスを表示
    abs_path = os.path.abspath(output_path)
    logger.info(f"画像を保存します: {abs_path}")
    
    # 画像を保存
    result = cv2.imwrite(output_path, image)
    
    # 保存結果を確認
    if result:
        logger.info(f"✅ 画像を保存しました: {output_path}")
        if os.path.exists(output_path):
            file_size = os.path.getsize(output_path)
            logger.info(f"ファイルサイズ: {file_size} バイト")
            # ファイルの権限を確認
            try:
                file_stat = os.stat(output_path)
                logger.info(f"ファイル権限: {oct(file_stat.st_mode)}")
            except Exception as e:
                logger.warning(f"ファイル権限の取得に失敗: {e}")
        else:
            logger.error(f"⚠️ 保存成功と報告されましたが、ファイルが存在しません")
    else:
        logger.error(f"❌ 画像の保存に失敗しました: {output_path}")
        # 書き込み権限の確認
        try:
            parent_dir = os.path.dirname(output_path) or '.'
            if not os.access(parent_dir, os.W_OK):
                logger.error(f"ディレクトリ {parent_dir} に書き込み権限がありません")
            # ディレクトリの情報を表示
            logger.error(f"ディレクトリの情報: {os.stat(parent_dir)}")
            # ディレクトリの内容を表示
            logger.error(f"ディレクトリの内容: {os.listdir(parent_dir)}")
        except Exception as e:
            logger.error(f"権限チェック中にエラー: {e}")
    
    return result


def visualize_result(image, masks, boxes, scores, alpha=0.5):
    """マスク可視化"""
    # 画像のコピーを作成
    vis_image = image.copy()
    
    # マスクを描画
    for i, mask in enumerate(masks):
        # ランダムカラーを生成
        color = np.random.randint(0, 255, 3).tolist()
        
        # マスクの適用
        colored_mask = np.zeros_like(vis_image)
        colored_mask[mask > 0] = color
        
        # 元画像とブレンド
        vis_image = cv2.addWeighted(
            vis_image, 1.0, colored_mask, alpha, 0.0
        )
    
    # ボックス描画
    if boxes:
        for i, box in enumerate(boxes):
            x1, y1, x2, y2 = [int(coord) for coord in box]
            cv2.rectangle(vis_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # スコア表示
            if scores and i < len(scores):
                score_text = f"Score: {scores[i]:.2f}"
                cv2.putText(
                    vis_image,
                    score_text,
                    (x1, y1 - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.5,
                    (0, 255, 0),
                    2
                )
    
    return vis_image


def main(args: List[str] = None):
    parser = argparse.ArgumentParser(
        description="LangSAM client for text-prompted object segmentation."
    )
    parser.add_argument("--image", type=str, required=True, help="Path to the input image file.")
    parser.add_argument("--prompt", type=str, required=True, help="Text prompt describing the object to segment.")
    parser.add_argument("--box-threshold", type=float, default=0.3, help="Threshold for box predictions (default: 0.3).")
    parser.add_argument("--text-threshold", type=float, default=0.25, help="Threshold for text predictions (default: 0.25).")
    parser.add_argument("--output", type=str, default=None, help="Path to save the visualization result (default: None).")
    parser.add_argument("--debug", action="store_true", help="Enable debug logging.")
    
    rclpy.init(args=args)
    client = LangSAMClient()
    cmd_args = parser.parse_args(args=args[1:] if args else None)

    logger = client.get_logger()
    if cmd_args.debug:
        logger.set_level(logging.DEBUG)
        logger.debug("デバッグモードが有効になりました")

    try:
        original_image = load_image(cmd_args.image, logger)
        if original_image is None:
            logger.error("画像の読み込みに失敗しました。終了します。")
            client.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

        image_rgb = cv2.cvtColor(original_image, cv2.COLOR_BGR2RGB)
        logger.info("画像をRGBに変換しました----")

        executor = SingleThreadedExecutor()
        executor.add_node(client)

        logger.info(f"Sending segmentation request with prompt: '{cmd_args.prompt}'")
        _, future = client.segment_image(
            image_rgb,
            cmd_args.prompt,
            box_threshold=cmd_args.box_threshold,
            text_threshold=cmd_args.text_threshold
        )

        logger.info("セグメンテーション結果を待機中...")
        executor.spin_until_future_complete(future)

        logger.info("セグメンテーション結果を処理中...")
        result = client.process_segmentation_result(future)
        logger.info(f"結果: {len(result['masks'])}個のマスク, {len(result['boxes'])}個のボックス")

        if result["masks"]:
            vis_image = visualize_result(
                original_image,
                result["masks"],
                result["boxes"],
                result["scores"]
            )

            if cmd_args.output:
                logger.info(f"保存準備: 出力パス={cmd_args.output}")
                saved = save_image(cmd_args.output, vis_image, logger)

                if saved and os.path.exists(cmd_args.output):
                    logger.info(f"確認: ファイルが正常に保存されました: {cmd_args.output}")
                    file_size = os.path.getsize(cmd_args.output)
                    logger.info(f"確認: ファイルサイズ={file_size}バイト")
                else:
                    logger.error("ファイルの保存に失敗した可能性があります")
            else:
                logger.info("出力パスが指定されていないため、保存をスキップします")
        else:
            logger.warning(f"対象が見つかりませんでした: '{cmd_args.prompt}'")

    except Exception as e:
        logger.error(f"エラーが発生しました: {e}")
        logger.error(traceback.format_exc())
    finally:
        logger.info("ノードをクリーンアップしています...")
        client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("ユーザーによって中断されました")
        sys.exit(130)
    except Exception as e:
        print(f"予期しないエラー: {e}")
        traceback.print_exc()
        sys.exit(1)
