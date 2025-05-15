#!/usr/bin/env python3
"""
LangSAM server node script.
This script starts a ROS 2 node for text-prompted object segmentation using LangSAM.
"""

import logging
import sys
import traceback
from typing import List

import rclpy
from rclpy.executors import MultiThreadedExecutor

from ros2_lang_sam.lang_sam_server import LangSAMServer


def main(args: List[str] = None) -> None:
    """
    Main function for the LangSAM server node.
    
    Args:
        args: Command line arguments
    """
    # ROS 2初期化（引数をそのまま渡す）
    rclpy.init(args=args)
    
    # サーバーの作成
    try:
        # launch ファイルから渡されるパラメータで初期化
        lang_sam_server = LangSAMServer()
        
        # デバイス情報をログに出力
        device = lang_sam_server.get_parameter('device').value
        lang_sam_server.get_logger().info(f"LangSAM サーバーをデバイス '{device}' で初期化します")
        
        # マルチスレッドエグゼキュータを使用
        executor = MultiThreadedExecutor()
        executor.add_node(lang_sam_server)
        
        try:
            lang_sam_server.get_logger().info(
                f"LangSAM サーバーを起動しました"
            )
            executor.spin()
        except KeyboardInterrupt:
            lang_sam_server.get_logger().info("キーボード割り込みにより停止します")
        except Exception as e:
            lang_sam_server.get_logger().error(f"サーバー実行中にエラーが発生しました: {e}")
            lang_sam_server.get_logger().error(traceback.format_exc())
        finally:
            # クリーンアップ
            executor.shutdown()
            lang_sam_server.destroy_node()
            rclpy.shutdown()
    except Exception as e:
        print(f"サーバーの初期化中にエラーが発生しました: {e}", file=sys.stderr)
        traceback.print_exc()
        rclpy.shutdown()
        sys.exit(1)


if __name__ == "__main__":
    main()
