#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
CL-500W 可调电源控制器 - 主入口

启动方式:
    python main.py              # 同时启动桌面UI + Web服务
    python main.py --no-web     # 仅启动桌面UI
    python main.py --web-only   # 仅启动Web服务 (无Qt界面)
    python main.py --port 8080  # 指定Web端口

作者: AI协作团队
日期: 2026-02-05
更新: 2026-02-15 - 增加 Web 远程控制服务
"""

import sys
import argparse
from pathlib import Path

# 确保项目路径在 Python 路径中
project_root = Path(__file__).parent
sys.path.insert(0, str(project_root))


def main():
    parser = argparse.ArgumentParser(description="电源温控系统")
    parser.add_argument("--no-web", action="store_true", help="不启动 Web 服务器")
    parser.add_argument("--web-only", action="store_true", help="仅启动 Web 服务器 (无桌面UI)")
    parser.add_argument("--host", default="0.0.0.0", help="Web 服务监听地址 (默认 0.0.0.0)")
    parser.add_argument("--port", type=int, default=8080, help="Web 服务端口 (默认 8080)")
    args = parser.parse_args()

    if args.web_only:
        # 仅 Web 模式
        from src.server.web_server import run_server
        print(f"🌐 仅Web模式 - 启动在 http://{args.host}:{args.port}")
        print("⚠ 注意：仅Web模式下无法连接硬件，需配合桌面客户端使用")
        run_server(host=args.host, port=args.port)
    else:
        # 启动 Web 服务器（后台线程）
        if not args.no_web:
            from src.server.web_server import start_server_thread
            start_server_thread(host=args.host, port=args.port)
            print(f"🌐 Web 控制面板: http://localhost:{args.port}")

        # 启动 Qt 桌面 UI
        from src.ui.main_window import main as ui_main
        ui_main()


if __name__ == "__main__":
    main()
