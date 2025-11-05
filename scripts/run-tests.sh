#!/bin/bash
# テスト実行スクリプト - すべてのテストを実行

set -e  # エラーが発生したら即座に終了

echo "======================================"
echo "🧪 Semantic Map Platform - Test Runner"
echo "======================================"
echo ""

# 色付き出力
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# テスト結果を保存するディレクトリ
REPORTS_DIR="tests/reports"
mkdir -p "$REPORTS_DIR"

# テスト開始時刻
START_TIME=$(date +%s)

# エラーハンドリング
handle_error() {
    echo -e "${RED}❌ テストが失敗しました${NC}"
    exit 1
}

trap handle_error ERR

echo -e "${YELLOW}📦 依存関係のインストール${NC}"
echo "--------------------------------------"

# Python依存関係
if [ -f "requirements.txt" ]; then
    echo "Installing Python dependencies..."
    pip install -q -r requirements.txt
fi

# Node.js依存関係
if [ -f "package.json" ]; then
    echo "Installing Node.js dependencies..."
    npm install --silent
fi

echo ""
echo -e "${GREEN}✅ 依存関係のインストール完了${NC}"
echo ""

# Python バックエンドテスト
echo -e "${YELLOW}🐍 Pythonバックエンドテスト実行${NC}"
echo "--------------------------------------"
pytest tests/unit/backend/ -v --tb=short || handle_error
echo ""
echo -e "${GREEN}✅ Pythonユニットテスト完了${NC}"
echo ""

# JavaScript フロントエンドテスト
echo -e "${YELLOW}🌐 JavaScriptフロントエンドテスト実行${NC}"
echo "--------------------------------------"
npm run test:frontend || handle_error
echo ""
echo -e "${GREEN}✅ JavaScriptユニットテスト完了${NC}"
echo ""

# E2Eテスト（オプション）
if [ "$RUN_E2E" = "true" ]; then
    echo -e "${YELLOW}🎭 E2Eテスト実行${NC}"
    echo "--------------------------------------"

    # バックエンドサーバーをバックグラウンドで起動
    echo "Starting backend server..."
    python apps/backend/server.py &
    BACKEND_PID=$!

    # フロントエンドサーバーをバックグラウンドで起動
    echo "Starting frontend server..."
    python apps/frontend/server.py &
    FRONTEND_PID=$!

    # サーバーが起動するまで待機
    echo "Waiting for servers to start..."
    sleep 5

    # Playwrightのインストール確認
    npx playwright install --with-deps chromium

    # E2Eテスト実行
    npm run test:e2e || {
        kill $BACKEND_PID $FRONTEND_PID
        handle_error
    }

    # サーバーを停止
    kill $BACKEND_PID $FRONTEND_PID

    echo ""
    echo -e "${GREEN}✅ E2Eテスト完了${NC}"
    echo ""
fi

# カバレッジレポート生成
if [ "$GENERATE_COVERAGE" = "true" ]; then
    echo -e "${YELLOW}📊 カバレッジレポート生成${NC}"
    echo "--------------------------------------"

    # Pythonカバレッジ
    pytest --cov=apps/backend --cov-report=html:tests/coverage/backend/html tests/unit/backend/

    # JavaScriptカバレッジ
    npm run test:coverage:frontend

    echo ""
    echo -e "${GREEN}✅ カバレッジレポート生成完了${NC}"
    echo ""
    echo "📁 カバレッジレポート:"
    echo "  - Python: tests/coverage/backend/html/index.html"
    echo "  - JavaScript: tests/coverage/frontend/index.html"
fi

# テスト終了時刻
END_TIME=$(date +%s)
DURATION=$((END_TIME - START_TIME))

echo ""
echo "======================================"
echo -e "${GREEN}✨ すべてのテストが成功しました! ✨${NC}"
echo "======================================"
echo "⏱  実行時間: ${DURATION}秒"
echo ""
echo "📊 テストレポート: ${REPORTS_DIR}/"
echo ""
