# 生成AI向け開発ガイドライン

## プロジェクト概要

### アプリケーション名
Semantic Map Platform for HSR

### 主要機能
1. マップビューア: 2D/3D地図の表示・編集
2. レイヤー管理: 複数レイヤーの重ね合わせ
3. 描画ツール: 手動アノテーション
4. メタデータ表示: YAML形式の地図メタデータ
5. 操作カタログ: ロボット操作仕様のDB
6. MapQLクエリ: 地図情報の自然言語的クエリ

### 技術スタック
- フロントエンド: Vanilla JavaScript (ES6+), HTML5 Canvas, CSS3
- バックエンド: Python (Flask)
- データ形式: YAML, PGM (Portable Gray Map), JSON

## プロジェクト構造

```
apps/frontend/static/
├── index.html              # メインHTML (184行)
├── styles/                 # CSSモジュール
│   ├── base.css           # リセット、基本スタイル
│   ├── layout.css         # レイアウト
│   ├── components.css     # 共通コンポーネント
│   ├── map-viewer.css     # マップビューア
│   ├── drawing-tools.css  # 描画ツール
│   ├── layers-panel.css   # レイヤーパネル
│   ├── metadata-overlay.css # メタデータオーバーレイ
│   ├── menu-bar.css       # メニューバー
│   └── tabs.css           # タブUI
└── js/                    # JavaScriptモジュール
    ├── main.js            # エントリーポイント (起動時の自動プロファイル読み込み)
    ├── config.js          # 設定・定数
    ├── state/
    │   └── mapState.js    # グローバル状態管理
    ├── modules/           # コア機能
    │   ├── layerManager.js
    │   ├── drawingTools.js
    │   ├── fileLoader.js
    │   ├── metadataDisplay.js
    │   ├── overlayRenderer.js
    │   ├── viewportControl.js
    │   ├── historyManager.js
    │   ├── profileManager.js
    │   └── apiClient.js
    ├── utils/             # ユーティリティ
    │   ├── coordinates.js
    │   ├── canvas.js
    │   ├── imageProcessing.js
    │   └── formatting.js
    └── ui/                # UI制御
        ├── tabs.js
        ├── controls.js
        ├── toast.js       # トーストメッセージ (再利用可能な通知システム)
        └── events.js
```

## モジュール設計原則

### 1. 単一責任の原則 (SRP)
各ファイルは1つの明確な責務を持つ。

**良い例:**
```javascript
// layerManager.js - レイヤー管理のみ
export function createLayer(id, name, type) { ... }
export function deleteLayer(layerId) { ... }
```

**悪い例:**
```javascript
// utils.js - 責務が不明確
export function createLayer() { ... }
export function formatDate() { ... }
export function makeApiCall() { ... }
```

### 2. 適切なファイルサイズ
- 理想: 300-500行
- 最大: 1000行
- 理由: 生成AIの1リクエストで全体を把握可能

### 3. 明確な命名規則

#### ファイル名
- 機能を表す名詞: `layerManager.js`, `fileLoader.js`
- 複数形は避ける: `layer.js` より `layerManager.js`
- キャメルケース: `drawingTools.js` (小文字始まり)

#### 関数名
- 動詞 + 名詞: `createLayer`, `deleteLayer`
- Boolean は is/has 接頭辞: `isVisible`, `hasPermission`
- イベントハンドラは handle 接頭辞: `handleClick`, `handleFileSelect`

### 4. 依存関係の明示化

各ファイルのヘッダーに依存関係を記載:

```javascript
/**
 * @file layerManager.js
 * @description レイヤーの作成、削除、表示管理
 *
 * @requires state/mapState.js - グローバル状態
 * @requires utils/canvas.js - Canvas操作
 * @requires utils/coordinates.js - 座標変換
 *
 * @exports createLayer
 * @exports deleteLayer
 * @exports toggleLayerVisibility
 * @exports updateLayersPanel
 */
```

## コーディング規約

### JavaScript

#### 1. モジュール化
ES6 Modules を使用:

```javascript
// export
export function createLayer(id, name) { ... }

// import
import { createLayer } from './modules/layerManager.js';
```

#### 2. JSDoc コメント
すべての public 関数に JSDoc を記載:

```javascript
/**
 * 新しいレイヤーを作成する
 *
 * @param {string} id - レイヤーの一意識別子
 * @param {string} name - レイヤーの表示名
 * @param {('image'|'metadata'|'drawing')} type - レイヤータイプ
 * @param {boolean} [permanent=false] - 恒久レイヤーかどうか
 * @returns {Object} 作成されたレイヤーオブジェクト
 *
 * @example
 * const layer = createLayer('map-1', '地図画像', 'image', true);
 */
export function createLayer(id, name, type, permanent = false) {
    // 実装
}
```

#### 3. エラーハンドリング
明示的なエラー処理:

```javascript
export function loadImageFile(file) {
    if (!file) {
        console.error('loadImageFile: ファイルが指定されていません');
        return null;
    }

    try {
        // ファイル読み込み処理
    } catch (error) {
        console.error('loadImageFile: 読み込みエラー', error);
        throw error;
    }
}
```

#### 4. 定数の管理
マジックナンバーを避け、定数化:

```javascript
// 悪い例
if (scale > 10) { ... }

// 良い例
const MAX_SCALE = 10;
if (scale > MAX_SCALE) { ... }
```

### CSS

#### 1. BEM命名規則
```css
/* Block */
.layer-panel { }

/* Element */
.layer-panel__header { }
.layer-panel__item { }

/* Modifier */
.layer-panel__item--active { }
.layer-panel__item--hidden { }
```

#### 2. セクション分け
```css
/* ======================
   レイアウト
   ====================== */

.container { ... }
.grid { ... }

/* ======================
   コンポーネント
   ====================== */

.button { ... }
.input { ... }
```

## AIエージェント向け指示

### コード読み取り時

#### 最小限のファイル取得
特定の機能を理解する際は、関連ファイルのみを取得:

**例: レイヤー機能を理解したい**
```
必要なファイル:
1. js/modules/layerManager.js
2. js/state/mapState.js
3. js/utils/canvas.js
合計: 約800行、約8,000トークン
```

**不要なファイル:**
- drawingTools.js
- fileLoader.js
- その他の無関係なモジュール

#### ファイルの優先順位
1. ヘッダーコメント: ファイルの目的と依存関係を確認
2. export 文: 公開APIを把握
3. 主要関数: JSDocから機能を理解
4. 実装詳細: 必要に応じて

### コード生成時

#### テンプレート
新しい機能を追加する際のテンプレート:

```javascript
/**
 * @file newFeature.js
 * @description [機能の説明]
 *
 * @requires [依存モジュール]
 *
 * @exports [エクスポートする関数]
 */

// ================
// インポート
// ================

import { mapState } from '../state/mapState.js';

// ================
// 定数
// ================

const CONSTANT_NAME = 'value';

// ================
// 公開関数
// ================

/**
 * [関数の説明]
 *
 * @param {type} param - [パラメータ説明]
 * @returns {type} [戻り値説明]
 */
export function publicFunction(param) {
    // 実装
}

// ================
// 内部関数
// ================

/**
 * [内部関数の説明]
 *
 * @private
 */
function privateFunction() {
    // 実装
}
```

### コード修正時

#### 1. 影響範囲の確認
修正前に依存関係を確認:

```javascript
// layerManager.js を修正する前に確認すべきファイル:
// - drawingTools.js (レイヤー操作を使用)
// - fileLoader.js (レイヤー作成を使用)
// - viewportControl.js (レイヤー描画を使用)
```

#### 2. 後方互換性の維持
既存のAPIを変更する場合は、deprecated警告を追加:

```javascript
/**
 * @deprecated v0.2.0 以降は createLayer を使用してください
 */
export function addLayer(name) {
    console.warn('addLayer is deprecated. Use createLayer instead.');
    return createLayer(generateId(), name, 'drawing');
}
```

## モジュール索引

### コア機能

| モジュール | 責務 | 主要関数 | 依存 |
|-----------|------|---------|------|
| layerManager.js | レイヤー CRUD | `createLayer`, `deleteLayer`, `toggleLayerVisibility` | mapState, canvas |
| drawingTools.js | 描画操作 | `selectTool`, `performDrawing` | mapState, layerManager |
| fileLoader.js | ファイル I/O | `loadPGMImageFile`, `loadYAMLMetadataFile` | mapState, imageProcessing |
| metadataDisplay.js | メタデータ UI | `displayMetadata`, `toggleMetadataMinimize` | mapState |
| overlayRenderer.js | オーバーレイ描画 | `drawGridOverlay`, `drawOriginOverlay` | mapState, coordinates |
| viewportControl.js | ズーム/パン | `zoomIn`, `zoomOut`, `resetView` | mapState, layerManager |
| historyManager.js | アンドゥ/リドゥ | `undo`, `redo`, `saveToHistory` | mapState |
| profileManager.js | プロファイル管理 | `saveProfile`, `loadProfile`, `getLastProfile` | mapState, toast |
| apiClient.js | API 通信 | `loadStats`, `loadOperations`, `executeQuery` | config |

### ユーティリティ

| モジュール | 責務 | 主要関数 |
|-----------|------|---------|
| coordinates.js | 座標変換 | `worldToCanvas`, `canvasToImagePixel` |
| canvas.js | Canvas 操作 | `clearCanvas`, `resizeCanvas` |
| imageProcessing.js | 画像処理 | `parsePGM`, `pgmToImage` |
| formatting.js | フォーマット | `formatDistance`, `getNiceNumber` |

### UI制御

| モジュール | 責務 | 主要関数 |
|-----------|------|---------|
| tabs.js | タブ切り替え | `switchTab` |
| controls.js | UI 制御・プロファイル管理 | `loadImageFile`, `clearMap`, `loadSelectedProfile` |
| toast.js | トーストメッセージ | `showSuccess`, `showError`, `showInfo`, `showWarning` |
| events.js | イベント管理 | `setupEventListeners` |

## トークン消費の最適化

### Before（リファクタリング前）
```
index.html: 32,645トークン
AIが全体を読み込む必要がある
```

### After（モジュール化）
```
必要なモジュールのみ読み込み:
layerManager.js: 4,000トークン
mapState.js: 1,500トークン
canvas.js: 800トークン
合計: 6,300トークン（80%削減）
```

### ファイルサイズの目安

| ファイルタイプ | 理想サイズ | 最大サイズ | トークン目安 |
|--------------|-----------|----------|------------|
| CSS | 100-200行 | 500行 | 1,000-2,000 |
| JavaScript | 300-500行 | 1,000行 | 3,000-5,000 |
| HTML | 100-200行 | 500行 | 1,000-2,000 |

## トラブルシューティング

### よくある問題

#### 1. モジュールが読み込めない
症状: `Uncaught TypeError: Failed to resolve module specifier`

原因: 相対パスが不正

解決策:
```javascript
// 間違い
import { createLayer } from 'layerManager.js';

// 正しい
import { createLayer } from './modules/layerManager.js';
```

#### 2. 状態が更新されない
症状: レイヤーの変更がUIに反映されない

原因: `mapState` の直接変更

解決策:
```javascript
// 間違い
mapState.layerStack.push(newLayer);

// 正しい
import { addLayer } from '../state/mapState.js';
addLayer(newLayer);
```

#### 3. 循環依存
症状: モジュールが undefined

原因: A → B → A の循環参照

解決策:
- 共通処理を別モジュールに抽出
- イベント駆動アーキテクチャの採用

## テスト戦略

### テストの重要性

新機能の実装やリファクタリング時には、必ずテストを実行して既存機能が破壊されていないことを確認してください。

### テストの種類

#### 1. ユニットテスト
個々のモジュール・関数の動作をテストします。

**Python（バックエンド）:**
```bash
# すべてのバックエンドユニットテスト
pytest tests/unit/backend/

# 特定のテストファイル
pytest tests/unit/backend/test_operations.py

# 特定のテストケース
pytest tests/unit/backend/test_operations.py::TestOperationsAPI::test_get_operations_returns_200
```

**JavaScript（フロントエンド）:**
```bash
# すべてのフロントエンドユニットテスト
npm run test:unit:frontend

# ウォッチモード（開発時）
npm run test:watch

# UIモード
npm run test:ui
```

#### 2. 統合テスト
複数のモジュール間の連携をテストします。

```bash
# バックエンド統合テスト
pytest tests/integration/backend/

# フロントエンド統合テスト
npm run test:integration:frontend
```

#### 3. E2Eテスト
実際のブラウザ環境で、ユーザーの操作フローをテストします。

```bash
# すべてのE2Eテスト
npm run test:e2e

# 特定のブラウザのみ
npx playwright test --project=chromium
```

### テストカバレッジ

テストカバレッジレポートを生成して、テストされていないコードを特定します。

```bash
# すべてのカバレッジレポート生成
npm run test:coverage

# バックエンドのみ
npm run test:coverage:backend

# フロントエンドのみ
npm run test:coverage:frontend
```

**カバレッジレポート:**
- Python: `tests/coverage/backend/html/index.html`
- JavaScript: `tests/coverage/frontend/index.html`

### テストデータとプロファイルの管理

#### テストデータの保存

テスト実行中に生成されたデータやプロファイルは、`tests/reports/` ディレクトリに自動的に保存されます。これにより、テスト結果の解析や問題の再現が容易になります。

**例:**
```python
# Pythonテストでデータを保存
def test_create_operation(client, save_test_output):
    response = client.post('/api/operations', json=data)
    result = response.get_json()

    # テスト結果を保存
    save_test_output('operation_creation_test', result)
```

#### テストフィクスチャ

テスト用のフィクスチャは `tests/fixtures/` に配置されています:

- `tests/fixtures/maps/` - テスト用マップデータ（YAML, PGM）
- `tests/fixtures/profiles/` - テスト用プロファイルJSON

### テスト実行のベストプラクティス

#### 1. テストの独立性
各テストは他のテストに依存しないように作成してください。

```javascript
// 良い例
describe('layerManager', () => {
  beforeEach(() => {
    // 各テスト前に状態をリセット
    resetState();
  });

  it('レイヤーを追加できる', () => {
    // テストコード
  });
});
```

#### 2. AAA形式
テストコードは Arrange-Act-Assert 形式で記述してください。

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

#### 3. わかりやすいテスト名
テスト名は「何をテストするか」が明確にわかるようにしてください。

```python
# 良い例
def test_get_operations_returns_200(client):
    """操作カタログ一覧取得が200を返すことを確認"""
    # テストコード

# 悪い例
def test_operations(client):
    # テストコード
```

#### 4. クリーンアップ
テスト後は必ず状態をクリーンアップしてください。

```javascript
afterEach(() => {
  // モックをリセット
  vi.clearAllMocks();

  // localStorageをクリア
  localStorage.clear();
});
```

### 新機能実装時のテスト手順

1. **ユニットテストを先に書く（TDD推奨）**
   ```bash
   # テストを先に書いて実行（失敗することを確認）
   npm run test:watch
   ```

2. **機能を実装する**
   - テストが通るように実装

3. **統合テストを追加する**
   - 複数のモジュールが連携して動作することを確認

4. **E2Eテストを追加する（必要に応じて）**
   - ユーザー視点での動作を確認

5. **すべてのテストを実行して確認**
   ```bash
   npm run test
   ```

### リファクタリング時のテスト手順

1. **既存のテストをすべて実行**
   ```bash
   npm run test
   ```
   - すべてのテストが通ることを確認

2. **リファクタリングを実行**

3. **再度すべてのテストを実行**
   - リファクタリング後もすべてのテストが通ることを確認

4. **カバレッジを確認**
   ```bash
   npm run test:coverage
   ```
   - カバレッジが低下していないことを確認

### CI/CD統合

GitHub Actionsを使用して、プッシュ時やプルリクエスト時に自動的にテストが実行されます。

**ワークフロー:** `.github/workflows/test.yml`

テストが失敗した場合、マージはブロックされます。

### テストレポート

テスト実行後、以下のレポートが生成されます:

- **HTMLレポート:** `tests/reports/`
- **カバレッジレポート:** `tests/coverage/`
- **Playwrightレポート:** `tests/reports/playwright/`

これらのレポートを確認して、テストの実行状況やカバレッジを把握してください。

### トラブルシューティング

#### テストが失敗する場合

1. **エラーメッセージを確認**
   - どのテストが失敗しているかを特定

2. **該当するコードを確認**
   - 最近の変更が影響していないか確認

3. **テストを個別に実行**
   ```bash
   # 特定のテストのみ実行
   pytest tests/unit/backend/test_operations.py::test_specific_case -v
   ```

4. **デバッグモードで実行**
   ```bash
   # Vitestのデバッグモード
   npm run test:ui
   ```

#### カバレッジが低い場合

1. **カバレッジレポートを確認**
   - どの部分がテストされていないかを特定

2. **不足しているテストを追加**
   - エッジケースや例外処理のテストを追加

3. **不要なコードを削除**
   - デッドコードがあれば削除

## 関連ドキュメント

- [MODULE_INDEX.md](./MODULE_INDEX.md) - モジュール詳細
- [README.md](./README.md) - プロジェクト概要
- [TESTING.md](./TESTING.md) - テスト詳細ドキュメント
