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

## 関連ドキュメント

- [MODULE_INDEX.md](./MODULE_INDEX.md) - モジュール詳細
- [README.md](./README.md) - プロジェクト概要
