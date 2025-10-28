#!/usr/bin/env python3
"""
Semantic Map Platform Server
RoboCup@Home DSPL向けの意味地図プラットフォームのメインサーバー
"""

import os
import sys
import subprocess
import time
from pathlib import Path


def check_python_version():
    """Python バージョンチェック"""
    if sys.version_info < (3, 8):
        print("❌ Python 3.8以上が必要です")
        sys.exit(1)
    print(f"✅ Python {sys.version_info.major}.{sys.version_info.minor}.{sys.version_info.micro}")


def check_dependencies():
    """依存関係のチェック"""
    try:
        import flask
        import flask_cors
        print("✅ 依存関係がインストールされています")
        return True
    except ImportError:
        print("⚠️  依存関係がインストールされていません")
        print("\n以下のコマンドでインストールしてください:")
        print("  pip install -r requirements.txt")
        return False


def start_backend():
    """バックエンドサーバーを起動"""
    backend_path = Path(__file__).parent / "apps" / "backend"

    if not backend_path.exists():
        print("⚠️  バックエンドディレクトリが存在しません。作成します...")
        backend_path.mkdir(parents=True, exist_ok=True)

    server_file = backend_path / "server.py"

    if not server_file.exists():
        print("⚠️  バックエンドサーバーファイルが存在しません")
        return None

    print(f"🚀 バックエンドサーバーを起動中... (http://localhost:3000)")

    # バックエンドサーバーをサブプロセスで起動
    backend_process = subprocess.Popen(
        [sys.executable, str(server_file)],
        cwd=str(backend_path)
    )

    return backend_process


def start_frontend():
    """フロントエンドサーバーを起動"""
    frontend_path = Path(__file__).parent / "apps" / "frontend"

    if not frontend_path.exists():
        print("⚠️  フロントエンドディレクトリが存在しません。作成します...")
        frontend_path.mkdir(parents=True, exist_ok=True)

    server_file = frontend_path / "server.py"

    if not server_file.exists():
        print("⚠️  フロントエンドサーバーファイルが存在しません")
        return None

    print(f"🚀 フロントエンドサーバーを起動中... (http://localhost:5173)")

    # フロントエンドサーバーをサブプロセスで起動
    frontend_process = subprocess.Popen(
        [sys.executable, str(server_file)],
        cwd=str(frontend_path)
    )

    return frontend_process


def main():
    """メインエントリーポイント"""
    print("=" * 60)
    print("🗺️  Semantic Map Platform for HSR")
    print("   RoboCup@Home DSPL 意味地図プラットフォーム")
    print("=" * 60)
    print()

    # バージョンチェック
    check_python_version()

    # 依存関係チェック
    if not check_dependencies():
        sys.exit(1)

    print()
    print("📦 サーバーを起動しています...")
    print()

    # バックエンドとフロントエンドを起動
    backend_process = start_backend()
    time.sleep(1)  # バックエンドの起動を待つ

    frontend_process = start_frontend()
    time.sleep(1)  # フロントエンドの起動を待つ

    if not backend_process and not frontend_process:
        print("\n❌ サーバーファイルが見つかりません")
        print("   apps/backend/server.py と apps/frontend/server.py を確認してください")
        sys.exit(1)

    print()
    print("=" * 60)
    print("✨ サーバーが起動しました!")
    print()

    if backend_process:
        print("📡 バックエンドAPI: http://localhost:3000")
    if frontend_process:
        print("🌐 フロントエンド:   http://localhost:5173")

    print()
    print("終了するには Ctrl+C を押してください")
    print("=" * 60)

    try:
        # プロセスの監視
        while True:
            time.sleep(1)

            # プロセスが終了していないかチェック
            if backend_process and backend_process.poll() is not None:
                print("\n⚠️  バックエンドサーバーが終了しました")
                break

            if frontend_process and frontend_process.poll() is not None:
                print("\n⚠️  フロントエンドサーバーが終了しました")
                break

    except KeyboardInterrupt:
        print("\n\n🛑 サーバーを停止しています...")

        # プロセスを終了
        if backend_process:
            backend_process.terminate()
        if frontend_process:
            frontend_process.terminate()

        # プロセスの終了を待つ
        if backend_process:
            backend_process.wait(timeout=5)
        if frontend_process:
            frontend_process.wait(timeout=5)

        print("✅ サーバーを停止しました")


if __name__ == "__main__":
    main()
