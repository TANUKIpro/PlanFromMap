# テストガイド

このドキュメントでは、Semantic Map Platformのテスト戦略、テストの実行方法、および包括的なテストガイドラインについて説明します。

## 目次

- [テスト概要](#テスト概要)
- [セットアップ](#セットアップ)
- [テストの実行](#テストの実行)
- [テストの種類](#テストの種類)
- [テストカバレッジ](#テストカバレッジ)
- [ベストプラクティス](#ベストプラクティス)
- [CI/CD統合](#cicd統合)
- [トラブルシューティング](#トラブルシューティング)

## テスト概要

本プロジェクトでは、以下の3種類のテストを実施しています：

| テストタイプ | フレームワーク | 対象 | 実行時間 |
|------------|------------|------|---------|
| ユニットテスト | pytest, Vitest | 個別の関数・モジュール | 短い（秒） |
| 統合テスト | pytest, Vitest | 複数モジュールの連携 | 中程度（数秒） |
| E2Eテスト | Playwright | ブラウザでのユーザー操作 | 長い（数分） |

### テストカバレッジ目標

- **バックエンド:** 80%以上
- **フロントエンド:** 70%以上
- **統合テスト:** 主要ワークフローの100%

## セットアップ

### 依存関係のインストール

```bash
# Python依存関係
pip install -r requirements.txt

# Node.js依存関係
npm install

# Playwrightブラウザ（E2Eテスト用）
npx playwright install --with-deps
```

### ディレクトリ構造

```
tests/
├── unit/                    # ユニットテスト
│   ├── backend/            # Pythonバックエンド
│   │   ├── test_health.py
│   │   ├── test_operations.py
│   │   ├── test_maps.py
│   │   ├── test_mapql.py
│   │   └── test_stats.py
│   └── frontend/           # JavaScriptフロントエンド
│       ├── utils/
│       │   ├── formatting.test.js
│       │   └── coordinates.test.js
│       └── state/
│           └── mapState.test.js
├── integration/            # 統合テスト
│   ├── backend/
│   └── frontend/
├── e2e/                    # E2Eテスト
│   ├── map-viewer.spec.js
│   ├── drawing-tools.spec.js
│   └── api-integration.spec.js
├── fixtures/               # テストフィクスチャ
│   ├── maps/
│   └── profiles/
├── coverage/              # カバレッジレポート
└── reports/               # テストレポート
```

## テストの実行

### すべてのテスト

```bash
# すべてのテスト（バックエンド + フロントエンド）
npm run test

# テスト実行スクリプト（カバレッジ付き）
./scripts/run-tests.sh
```

### ユニットテスト

#### バックエンド

```bash
# すべてのバックエンドユニットテスト
pytest tests/unit/backend/

# 特定のテストファイル
pytest tests/unit/backend/test_operations.py

# 特定のテストクラス
pytest tests/unit/backend/test_operations.py::TestOperationsAPI

# 特定のテストケース
pytest tests/unit/backend/test_operations.py::TestOperationsAPI::test_get_operations_returns_200

# verbose出力
pytest tests/unit/backend/ -v

# カバレッジ付き
pytest tests/unit/backend/ --cov=apps/backend --cov-report=html
```

#### フロントエンド

```bash
# すべてのフロントエンドユニットテスト
npm run test:unit:frontend

# ウォッチモード（開発時）
npm run test:watch

# UIモード（ブラウザでテスト結果を確認）
npm run test:ui

# 特定のファイルのみ
npx vitest run tests/unit/frontend/utils/formatting.test.js
```

### 統合テスト

```bash
# バックエンド統合テスト
pytest tests/integration/backend/

# フロントエンド統合テスト
npm run test:integration:frontend
```

### E2Eテスト

```bash
# すべてのE2Eテスト
npm run test:e2e

# 特定のブラウザのみ
npx playwright test --project=chromium
npx playwright test --project=firefox
npx playwright test --project=webkit

# 特定のテストファイル
npx playwright test tests/e2e/map-viewer.spec.js

# ヘッドレスモードOFF（ブラウザを表示）
npx playwright test --headed

# デバッグモード
npx playwright test --debug
```

## テストの種類

### 1. ユニットテスト

#### バックエンド（Python + pytest）

**例: APIエンドポイントのテスト**

```python
def test_get_operations_returns_200(client):
    """操作カタログ一覧取得が200を返すことを確認"""
    response = client.get('/api/operations')
    assert response.status_code == 200

def test_get_operations_has_correct_structure(client):
    """操作カタログ一覧のレスポンス構造を確認"""
    response = client.get('/api/operations')
    data = json.loads(response.data)

    assert 'operations' in data
    assert 'count' in data
    assert isinstance(data['operations'], list)
```

**フィクスチャの使用:**

```python
@pytest.fixture
def sample_operation():
    """サンプル操作データを返す"""
    return {
        "id": "test_door_001",
        "name": "テストドア",
        "type": "door",
        "location": {"x": 1.0, "y": 2.0, "z": 0.0}
    }

def test_create_operation(client, sample_operation):
    response = client.post('/api/operations', json=sample_operation)
    assert response.status_code == 201
```

#### フロントエンド（JavaScript + Vitest）

**例: ユーティリティ関数のテスト**

```javascript
import { describe, it, expect } from 'vitest';
import { formatDistance } from '@utils/formatting.js';

describe('formatDistance', () => {
  it('1500mを"1.50 km"にフォーマットする', () => {
    expect(formatDistance(1500)).toBe('1.50 km');
  });

  it('25.5mを整数"26 m"にフォーマットする', () => {
    expect(formatDistance(25.5)).toBe('26 m');
  });
});
```

**モックの使用:**

```javascript
import { describe, it, expect, vi, beforeEach } from 'vitest';
import { mapState } from '@state/mapState.js';

describe('worldToCanvas', () => {
  beforeEach(() => {
    // モック設定
    mapState.metadata = {
      resolution: 0.05,
      origin: [-10.0, -10.0, 0.0]
    };
  });

  it('ワールド座標をキャンバス座標に変換する', () => {
    const result = worldToCanvas(0, 0, 100, 50);
    expect(result).not.toBeNull();
  });
});
```

### 2. 統合テスト

複数のモジュールが連携して動作することを確認します。

```python
@pytest.mark.integration
def test_create_and_retrieve_operation(client, save_test_output):
    """操作仕様の作成と取得のワークフローを確認"""
    # 1. 新しい操作仕様を作成
    new_operation = {
        'name': 'ワークフローテストドア',
        'type': 'door',
        'location': {'x': 5.5, 'y': 3.3, 'z': 0.0}
    }
    create_response = client.post('/api/operations', json=new_operation)
    created_data = json.loads(create_response.data)
    operation_id = created_data['id']

    # 2. 作成された操作仕様を取得
    get_response = client.get(f'/api/operations/{operation_id}')
    retrieved_data = json.loads(get_response.data)

    # 3. データが一致することを確認
    assert retrieved_data['id'] == operation_id
    assert retrieved_data['name'] == 'ワークフローテストドア'

    # 4. テスト結果を保存
    save_test_output('operation_workflow_test', {
        'created': created_data,
        'retrieved': retrieved_data
    })
```

### 3. E2Eテスト

ブラウザでユーザーの実際の操作フローをテストします。

```javascript
import { test, expect } from '@playwright/test';

test('マップファイルを読み込んでプロファイルを保存できる', async ({ page }) => {
  // 1. ページを開く
  await page.goto('/');

  // 2. YAMLファイルを読み込む
  const yamlChooserPromise = page.waitForEvent('filechooser');
  await page.click('button:has-text("Load YAML")');
  const yamlChooser = await yamlChooserPromise;
  await yamlChooser.setFiles('tests/fixtures/maps/test_map.yaml');

  // 3. プロファイル管理を開く
  await page.click('button:has-text("Profiles")');

  // 4. プロファイル名を入力して保存
  const profileNameInput = page.locator('input[type="text"]').first();
  await profileNameInput.fill('e2e-test-profile');

  const saveButton = page.locator('button:has-text("Save")').first();
  await saveButton.click();

  // 5. 成功を確認
  await page.waitForTimeout(1000);
  const toast = page.locator('.toast');
  await expect(toast).toBeVisible();
});
```

## テストカバレッジ

### カバレッジレポートの生成

```bash
# すべてのカバレッジレポート生成
npm run test:coverage

# バックエンドのみ
npm run test:coverage:backend

# フロントエンドのみ
npm run test:coverage:frontend

# 環境変数を使ったスクリプト実行
GENERATE_COVERAGE=true ./scripts/run-tests.sh
```

### カバレッジレポートの確認

#### バックエンド（Python）

- HTMLレポート: `tests/coverage/backend/html/index.html`
- JSONレポート: `tests/coverage/backend/coverage.json`

```bash
# ブラウザで確認
open tests/coverage/backend/html/index.html
```

#### フロントエンド（JavaScript）

- HTMLレポート: `tests/coverage/frontend/index.html`
- JSONレポート: `tests/coverage/frontend/coverage-final.json`

```bash
# ブラウザで確認
open tests/coverage/frontend/index.html
```

### カバレッジ目標

```ini
# pytest.ini の設定
[coverage:report]
precision = 2
show_missing = True
skip_covered = False
```

```javascript
// vitest.config.js の設定
coverage: {
  lines: 70,
  functions: 70,
  branches: 70,
  statements: 70
}
```

## ベストプラクティス

### 1. テストの独立性

各テストは他のテストに依存しないようにしてください。

```javascript
// ✅ 良い例
describe('layerManager', () => {
  beforeEach(() => {
    resetState(); // 各テスト前に状態をリセット
  });

  it('レイヤーを追加できる', () => {
    addLayerToStack({ id: 'layer1', name: 'Test Layer' });
    expect(mapState.layerStack).toHaveLength(1);
  });
});

// ❌ 悪い例
describe('layerManager', () => {
  it('レイヤーを追加できる', () => {
    addLayerToStack({ id: 'layer1', name: 'Test Layer' });
    expect(mapState.layerStack).toHaveLength(1);
  });

  it('レイヤーを削除できる', () => {
    // 前のテストの状態に依存している
    removeLayerFromStack('layer1');
    expect(mapState.layerStack).toHaveLength(0);
  });
});
```

### 2. AAA形式（Arrange-Act-Assert）

```javascript
it('ズーム操作が正しく動作する', () => {
  // Arrange: テストの準備
  const initialScale = mapState.scale;

  // Act: 実際の操作
  zoomIn();

  // Assert: 結果の検証
  expect(mapState.scale).toBeGreaterThan(initialScale);
});
```

### 3. わかりやすいテスト名

```python
# ✅ 良い例
def test_get_operations_returns_200(client):
    """操作カタログ一覧取得が200を返すことを確認"""
    response = client.get('/api/operations')
    assert response.status_code == 200

# ❌ 悪い例
def test_operations(client):
    response = client.get('/api/operations')
    assert response.status_code == 200
```

### 4. エッジケースのテスト

正常系だけでなく、エッジケースや異常系もテストしてください。

```javascript
describe('formatDistance', () => {
  it('正の値を正しくフォーマットする', () => {
    expect(formatDistance(1500)).toBe('1.50 km');
  });

  it('負の値を正しくフォーマットする', () => {
    expect(formatDistance(-1500)).toBe('-1.50 km');
  });

  it('Infinityを空文字列にする', () => {
    expect(formatDistance(Infinity)).toBe('');
  });

  it('NaNを空文字列にする', () => {
    expect(formatDistance(NaN)).toBe('');
  });
});
```

### 5. クリーンアップ

テスト後は必ず状態をクリーンアップしてください。

```javascript
afterEach(() => {
  // モックをリセット
  vi.clearAllMocks();

  // localStorageをクリア
  localStorage.clear();

  // sessionStorageをクリア
  sessionStorage.clear();
});
```

## CI/CD統合

### GitHub Actions

プッシュ時やプルリクエスト時に自動的にテストが実行されます。

**ワークフロー:** `.github/workflows/test.yml`

```yaml
name: Tests

on:
  push:
    branches: [ main, develop, "claude/**" ]
  pull_request:
    branches: [ main, develop ]

jobs:
  backend-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run backend tests
        run: pytest tests/unit/backend/

  frontend-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run frontend tests
        run: npm run test:frontend

  e2e-tests:
    runs-on: ubuntu-latest
    steps:
      - uses: actions/checkout@v4
      - name: Run E2E tests
        run: npm run test:e2e
```

### ローカルでのCI シミュレーション

```bash
# CI環境を模擬してテストを実行
CI=true npm run test
```

## トラブルシューティング

### テストが失敗する場合

#### 1. エラーメッセージを確認

```bash
# verbose出力でエラーの詳細を確認
pytest tests/unit/backend/ -v

# Vitestのデバッグモード
npm run test:ui
```

#### 2. 特定のテストのみ実行

```bash
# Pythonの場合
pytest tests/unit/backend/test_operations.py::test_specific_case -v

# JavaScriptの場合
npx vitest run tests/unit/frontend/utils/formatting.test.js
```

#### 3. デバッグモードで実行

```bash
# Playwrightのデバッグモード
npx playwright test --debug

# ブラウザを表示
npx playwright test --headed
```

### カバレッジが低い場合

1. **カバレッジレポートを確認**
   - どの部分がテストされていないかを特定

2. **不足しているテストを追加**
   - エッジケースや例外処理のテストを追加

3. **不要なコードを削除**
   - デッドコードがあれば削除

### E2Eテストがタイムアウトする場合

```javascript
// タイムアウトを延長
test('長時間かかる操作', async ({ page }) => {
  test.setTimeout(60000); // 60秒に延長

  // テストコード
});
```

### テストデータの確認

テスト実行中に生成されたデータは `tests/reports/` に保存されます。

```bash
# テストレポートディレクトリを確認
ls -la tests/reports/
```

## まとめ

- **常にテストを実行**: 新機能実装やリファクタリング時は必ずテストを実行
- **カバレッジを確認**: 定期的にカバレッジレポートを確認し、目標値を維持
- **CI/CDを活用**: GitHub Actionsで自動テストを実行し、品質を保証
- **テストデータを保存**: テスト結果を保存し、問題の再現と解析を容易に

詳細は[AI_GUIDELINES.md](./AI_GUIDELINES.md)のテスト戦略セクションも参照してください。
