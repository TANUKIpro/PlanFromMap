# モジュール索引

## ディレクトリ構造

```
apps/frontend/static/
├── index.html                      # メインHTML (184行)
├── index.html.backup               # 元のバックアップ (3,238行)
├── styles/                         # CSSモジュール (8ファイル)
│   ├── base.css                   # 23行 - リセット、基本スタイル
│   ├── layout.css                 # 40行 - レイアウト
│   ├── components.css             # 123行 - 共通コンポーネント
│   ├── map-viewer.css             # 127行 - マップビューア
│   ├── drawing-tools.css          # 116行 - 描画ツール
│   ├── layers-panel.css           # 183行 - レイヤーパネル
│   ├── metadata-overlay.css       # 188行 - メタデータオーバーレイ
│   ├── tabs.css                   # 128行 - タブUI
│   ├── object-property-panel.css  # 300行 - オブジェクトプロパティパネル
│   └── view-3d.css                # 80行 - 3Dビュー
├── js/                            # JavaScriptモジュール (17ファイル)
│   ├── main.js                    # 98行 - エントリーポイント
│   ├── config.js                  # 150行 - 設定・定数
│   ├── state/
│   │   └── mapState.js            # 250行 - 状態管理
│   ├── models/                    # データモデル (1ファイル)
│   │   └── objectTypes.js         # 500行 - オブジェクトタイプ定義
│   ├── modules/                   # コア機能 (14ファイル)
│   │   ├── layerManager.js        # 488行 - レイヤー管理
│   │   ├── historyManager.js      # 175行 - アンドゥ/リドゥ
│   │   ├── viewportControl.js     # 117行 - ズーム/パン
│   │   ├── drawingTools.js        # 463行 - 描画ツール
│   │   ├── overlayRenderer.js     # 532行 - オーバーレイ描画
│   │   ├── metadataDisplay.js     # 225行 - メタデータ表示
│   │   ├── fileLoader.js          # 259行 - ファイル読み込み
│   │   ├── apiClient.js           # 139行 - API通信
│   │   ├── profileManager.js      # 400行 - プロファイル管理
│   │   ├── rectangleManager.js    # 350行 - 四角形管理
│   │   ├── rectangleRenderer.js   # 520行 - 四角形2D描画
│   │   ├── objectPropertyManager.js # 350行 - オブジェクトプロパティ管理
│   │   └── threeDRenderer.js      # 1250行 - 3D描画（等角投影、カテゴリ別モデル）
│   ├── utils/                     # ユーティリティ (4ファイル)
│   │   ├── coordinates.js         # 133行 - 座標変換
│   │   ├── formatting.js          # 94行 - フォーマット
│   │   ├── canvas.js              # 54行 - Canvas操作
│   │   └── imageProcessing.js     # 190行 - 画像処理
│   └── ui/                        # UI制御 (5ファイル)
│       ├── tabs.js                # 48行 - タブ切り替え
│       ├── objectPropertyPanel.js # 600行 - オブジェクトプロパティパネル
│       ├── controls.js            # 163行 - UIコントロール
│       ├── toast.js               # 143行 - トーストメッセージ
│       └── events.js              # 404行 - イベント管理
└── assets/
    └── icons/                     # アイコン
```

## コアモジュール

### config.js
**責務**: アプリケーション全体の設定と定数管理

**エクスポート**:
- `API_BASE_URL` - APIサーバーのベースURL
- `CANVAS_DEFAULTS` - Canvas設定
- `DRAWING_DEFAULTS` - 描画ツール設定
- `LAYER_DEFAULTS` - レイヤー設定
- `OVERLAY_DEFAULTS` - オーバーレイ設定
- `HISTORY_DEFAULTS` - 履歴管理設定
- `UI_DEFAULTS` - UI設定

### state/mapState.js
**責務**: アプリケーション全体の状態管理

**エクスポート**:
- `mapState` - グローバル状態オブジェクト
- `getState(key)` - 状態の取得
- `updateState(updates)` - 状態の更新
- `resetState()` - 状態のリセット
- `addLayerToStack(layer)` - レイヤー追加
- `removeLayerFromStack(layerId)` - レイヤー削除
- `getLayerById(layerId)` - レイヤー取得
- `getSelectedLayer()` - 選択中レイヤー取得
- `getNextLayerId()` - 次のレイヤーID生成

### models/objectTypes.js (500行)
**責務**: オブジェクトタイプとプロパティスキーマの定義

**エクスポート**:
- `OBJECT_TYPES` - オブジェクトタイプ定数（NONE, SHELF, BOX, TABLE, DOOR, WALL）
- `OBJECT_TYPE_LABELS` - タイプごとの日本語ラベル
- `OBJECT_TYPE_COLORS` - タイプごとの表示色
- `PROPERTY_SCHEMAS` - カテゴリ別プロパティスキーマ
- `DEFAULT_PROPERTIES` - カテゴリ別デフォルト値
- `getObjectTypeOptions()` - カテゴリ選択肢取得
- `getFrontDirectionOptions()` - 前面方向選択肢取得
- `getPropertySchema(objectType)` - プロパティスキーマ取得
- `getDefaultProperties(objectType)` - デフォルトプロパティ取得
- `validatePropertyValue(objectType, key, value)` - プロパティ値検証

**スキーマ構造**:
```javascript
{
  label: "段数",
  type: "number",     // number | checkbox | select | radio
  min: 1,
  max: 10,
  default: 3,
  hasSlider: true,
  options: [...]      // selectまたはradioの場合
}
```

**カテゴリ別プロパティ**:
- **棚（SHELF）**: 段数、仕切り数、棚タイプ
- **箱（BOX）**: 蓋、取っ手、材質
- **テーブル（TABLE）**: 脚の数、天板形状、引き出し
- **扉（DOOR）**: 開き方、取っ手側、自動ドア
- **壁（WALL）**: 壁厚、材質、窓

**依存**: なし

## モジュール層

### modules/layerManager.js (488行)
**責務**: レイヤーの作成、削除、表示管理

**主要関数**:
- `initializeLayers()` - レイヤーシステムの初期化
- `createLayer(id, name, type, permanent)` - レイヤー作成
- `deleteLayer(layerId)` - レイヤー削除
- `toggleLayerVisibility(layerId, visible)` - 表示切替
- `updateLayersPanel()` - UIパネル更新
- `redrawAllLayers()` - 全レイヤー再描画

**依存**: mapState.js, canvas.js

### modules/historyManager.js (175行)
**責務**: アンドゥ/リドゥ機能

**主要関数**:
- `saveToHistory()` - 履歴保存
- `undo()` - 元に戻す
- `redo()` - やり直す
- `updateUndoRedoButtons()` - ボタン状態更新

**依存**: mapState.js

### modules/viewportControl.js (117行)
**責務**: ズーム/パン操作

**主要関数**:
- `resetView()` - 表示リセット
- `zoomIn()` - ズームイン
- `zoomOut()` - ズームアウト
- `updateZoomInfo()` - ズーム情報表示更新

**依存**: mapState.js

### modules/drawingTools.js (463行)
**責務**: 描画ツールの管理と描画処理

**主要関数**:
- `selectTool(toolName)` - ツール選択
- `performDrawing(tool, stroke, color, size)` - 描画実行
- `performBucketFill(x, y)` - 塗りつぶし

**依存**: mapState.js, layerManager.js, historyManager.js, imageProcessing.js

### modules/overlayRenderer.js (532行)
**責務**: グリッド、原点、スケールバーの描画

**主要関数**:
- `drawGridOverlay()` - グリッド描画
- `drawOriginOverlay()` - 原点マーカー描画
- `drawScaleBar()` - スケールバー描画

**依存**: mapState.js, coordinates.js, formatting.js

### modules/metadataDisplay.js (225行)
**責務**: メタデータの表示管理

**主要関数**:
- `displayMetadata(metadata)` - メタデータ表示
- `toggleLayer(layerKey, enabled)` - レイヤー切替

**依存**: mapState.js

### modules/fileLoader.js (259行)
**責務**: ファイルの読み込み処理

**主要関数**:
- `handleImageFileSelect(event)` - 画像ファイル処理
- `handleYAMLFileSelect(event)` - YAMLファイル処理
- `loadPGMImageFile(file)` - PGM画像読み込み

**依存**: mapState.js, imageProcessing.js, layerManager.js

### modules/apiClient.js (139行)
**責務**: バックエンドAPIとの通信

**主要関数**:
- `loadStats()` - 統計情報取得
- `loadOperations()` - 操作カタログ取得
- `executeQuery()` - MapQLクエリ実行

**依存**: config.js

### modules/profileManager.js (400行)
**責務**: プロファイルの保存・読み込み・管理

**主要関数**:
- `saveProfile(profileName)` - プロファイルの保存
- `loadProfile(profileName)` - プロファイルの読み込み
- `deleteProfile(profileName)` - プロファイルの削除
- `listProfiles()` - プロファイル一覧の取得
- `getLastProfile()` - 最後に使用したプロファイルの取得
- `setLastProfile(profileName)` - 最後に使用したプロファイルの設定
- `exportProfileToFile(profileName)` - プロファイルをファイルにエクスポート

**依存**: mapState.js, toast.js

**ストレージ**: LocalStorage (最大5MB)

### modules/rectangleManager.js (350行)
**責務**: 四角形オブジェクトのライフサイクル管理

**主要関数**:
- `createRectangle(x, y, width, height, rotation, color)` - 四角形の作成
- `deleteRectangle(rectangleId)` - 四角形の削除
- `selectRectangle(rectangleId)` - 四角形の選択
- `deselectAllRectangles()` - 全選択解除
- `getRectangleById(rectangleId)` - IDによる四角形取得
- `getSelectedRectangle()` - 選択中の四角形取得
- `getAllRectangles()` - 全四角形取得

**依存**: mapState.js, objectPropertyPanel.js

**データモデル**: オブジェクトプロパティ（objectType, heightMeters, frontDirection, objectProperties）を含む拡張四角形モデル

### modules/rectangleRenderer.js (520行)
**責務**: 四角形の2D描画とビジュアルフィードバック

**主要関数**:
- `drawRectangle(ctx, rectangle)` - 四角形を描画
- `drawResizeHandles(ctx, width, height)` - リサイズハンドル描画
- `drawRotationHandle(ctx, width, height)` - 回転ハンドル描画
- `drawHatchPattern(ctx, width, height, color)` - 斜線パターン描画
- `drawFrontDirectionArrow(ctx, rectangle, width, height, centerPos)` - 前面方向矢印描画
- `drawEdgeLength(ctx, rectangle, edge)` - エッジの長さ表示

**視覚的フィードバック**:
- 斜線パターン: カテゴリと高さが設定されたオブジェクトに表示
- 前面方向矢印: 前面方向が設定されたオブジェクトに表示（上/下/左/右）
- ハンドル: 選択時のリサイズ・回転操作

**依存**: mapState.js, RECTANGLE_DEFAULTS (config.js)

### modules/objectPropertyManager.js (350行)
**責務**: オブジェクトプロパティのビジネスロジック

**主要関数**:
- `setObjectType(rectangleId, objectType, preserveProperties)` - オブジェクトタイプ設定
- `setObjectProperty(rectangleId, key, value)` - 個別プロパティ設定
- `setObjectProperties(rectangleId, properties)` - 複数プロパティ一括設定
- `getObjectProperties(rectangleId)` - プロパティ取得
- `setHeightMeters(rectangleId, height)` - 高さ設定
- `setFrontDirection(rectangleId, direction)` - 前面方向設定
- `resetObjectProperties(rectangleId)` - プロパティリセット
- `get3DCoordinates(rectangleId)` - 3D座標への変換

**プロパティ検証**: スキーマベースのバリデーション（範囲チェック、型チェック）

**依存**: rectangleManager.js, objectTypes.js, historyManager.js

### modules/threeDRenderer.js (1250行)
**責務**: 3D描画エンジン（等角投影、カテゴリ別モデル）

**主要関数**:
- `initialize3DView()` - 3Dビューの初期化
- `toggle3DView(visible)` - 3Dビュー表示切替
- `render3DScene()` - 3Dシーン全体の描画
- `initializePropertyPreview()` - プロパティパネル内プレビュー初期化
- `renderPropertyPreview(rectangleId)` - プレビュー描画

**カテゴリ別3Dモデル描画**:
- `draw3DShelf()` / `drawPreviewShelf()` - 棚（前面開口、棚板表示）
- `draw3DBox_Hollowed()` / `drawPreviewBox_Hollowed()` - 箱（上部開放）
- `draw3DTable()` / `drawPreviewTable()` - テーブル（天板+脚）
- `draw3DDoor()` / `drawPreviewDoor()` - 扉（薄型）
- `draw3DWall()` / `drawPreviewWall()` - 壁（標準）

**等角投影**:
- `worldToIso(x, y, z)` - 3D座標→2D画面座標変換
- `worldToPreviewIso(x, y, z)` - プレビュー用投影変換

**インタラクション**（メインビューのみ）:
- マウスドラッグ: 視点回転
- マウスホイール: ズーム
- オフセット調整: パン操作

**状態管理**:
- `view3DState`: メイン3Dビュー状態
- `previewState`: プロパティパネルプレビュー状態

**依存**: mapState.js, objectPropertyManager.js, rectangleManager.js, objectTypes.js

## ユーティリティ層

### utils/coordinates.js (133行)
**責務**: 座標変換

**主要関数**:
- `worldToCanvas(worldX, worldY, drawX, drawY)` - 実世界→キャンバス
- `canvasToImagePixel(canvasX, canvasY, container)` - キャンバス→画像
- `imagePixelToCanvas(imagePixelX, imagePixelY, container)` - 画像→キャンバス

**依存**: mapState.js

### utils/formatting.js (94行)
**責務**: 数値・距離のフォーマット

**主要関数**:
- `formatDistance(meters)` - 距離フォーマット (km, m, cm, mm)
- `getNiceNumber(value)` - きれいな数値に丸める

**依存**: なし

### utils/canvas.js (54行)
**責務**: Canvas操作ヘルパー

**依存**: なし

### utils/imageProcessing.js (190行)
**責務**: 画像処理、PGMパーサー

**主要関数**:
- `parsePGM(uint8Array)` - PGMフォーマットパース
- `pgmToImage(pgmData)` - PGMデータ→Image変換
- `hexToRgb(hex)` - HEX→RGB変換

**依存**: なし

## UI層

### ui/tabs.js (48行)
**責務**: タブ切り替え

**主要関数**:
- `switchTab(tabId, event)` - タブ切り替え

**依存**: なし

### ui/controls.js (399行)
**責務**: UIコントロール操作、プロファイル管理

**主要関数**:
- `loadImageFile()` - 画像ファイル選択ダイアログ
- `loadYAMLFile()` - YAMLファイル選択ダイアログ
- `clearMap()` - マップクリア
- `drawMap()` - マップ描画
- `showProfileManager()` - プロファイル管理モーダルを表示
- `saveCurrentProfile()` - 現在の状態をプロファイルとして保存
- `loadSelectedProfile(profileName, options)` - プロファイルをロード
- `deleteSelectedProfile(profileName)` - プロファイルを削除

**依存**: mapState.js, layerManager.js, profileManager.js, toast.js

### ui/toast.js (143行)
**責務**: トーストメッセージ表示システム

**主要関数**:
- `showToast(message, options)` - トーストメッセージを表示
- `showSuccess(message, duration)` - 成功メッセージを表示
- `showError(message, duration)` - エラーメッセージを表示
- `showInfo(message, duration)` - 情報メッセージを表示
- `showWarning(message, duration)` - 警告メッセージを表示

**依存**: なし

**使用例**:
```javascript
import { showSuccess, showError } from './toast.js';
showSuccess('プロファイルを読み込みました');
showError('エラーが発生しました', 2000);
```

### ui/objectPropertyPanel.js (600行)
**責務**: オブジェクトプロパティパネルのUI制御

**主要関数**:
- `initializePropertyPanel()` - パネル初期化
- `showPropertyPanel(rectangleId)` - パネル表示
- `hidePropertyPanel()` - パネル非表示
- `updatePropertyPanel(rectangleId)` - パネル内容更新
- `refreshPropertyPanel()` - 現在選択中の四角形でパネルを更新

**動的フォーム生成**:
- `createNumberInput(field, currentValue, rectangle)` - 数値入力生成（スライダー付き）
- `createCheckboxInput(field, currentValue, rectangle)` - チェックボックス生成
- `createSelectInput(field, currentValue, rectangle)` - セレクトボックス生成
- `createRadioInput(field, currentValue, rectangle)` - ラジオボタン生成

**イベントハンドラ**:
- `handleCategoryChange()` - カテゴリ変更時の処理
- `handleHeightChange()` - 高さ変更時の処理
- `handleFrontDirectionChange()` - 前面方向変更時の処理
- ホイールイベント: パネル内スクロールのみ許可（マップズーム防止）

**特徴**:
- スキーマベースの動的フォーム生成
- リアルタイムプレビュー更新
- バリデーション付き入力
- カテゴリごとの専用プロパティフォーム

**依存**: mapState.js, objectPropertyManager.js, rectangleManager.js, objectTypes.js, toast.js, threeDRenderer.js

### ui/events.js (404行)
**責務**: イベントリスナーの統合管理

**主要関数**:
- `setupEventListeners()` - すべてのイベントリスナーをセットアップ
- `handleMouseDown()`, `handleMouseMove()`, `handleMouseUp()` - マウスイベント
- `handleWheel()` - ホイールイベント (ズーム)
- `handleKeydown()` - キーボードショートカット

**依存**: mapState.js, drawingTools.js, viewportControl.js, historyManager.js

### main.js (291行)
**責務**: アプリケーションのエントリーポイント

**機能**:
- すべてのモジュールをインポート
- 初期化処理 (レイヤーシステム、メニューバー、イベントリスナー)
- 起動時の自動プロファイル読み込み
- window オブジェクトへの関数公開 (HTML onclick 互換性のため)

**依存**: すべてのモジュール

**起動時の動作**:
1. DOMContentLoadedでinitializeApp()を実行
2. window.loadでonPageLoad()を実行
3. onPageLoad()で最後に使用したプロファイルを自動読み込み

## CSSモジュール

| ファイル | 行数 | 責務 |
|---------|------|------|
| base.css | 23行 | リセット、基本スタイル |
| layout.css | 40行 | レイアウト、グリッド |
| components.css | 123行 | 共通コンポーネント (ボタン、入力など) |
| map-viewer.css | 127行 | マップビューア専用スタイル |
| drawing-tools.css | 116行 | 描画ツール専用スタイル |
| layers-panel.css | 183行 | レイヤーパネル専用スタイル |
| metadata-overlay.css | 188行 | メタデータオーバーレイ専用スタイル |
| tabs.css | 128行 | タブUI専用スタイル |

## トークン消費の最適化

### シナリオ例: レイヤー機能の修正

Before (リファクタリング前):
```
必要なファイル: index.html (32,645トークン)
```

After (リファクタリング後):
```
必要なファイル:
- js/modules/layerManager.js (4,000トークン)
- js/state/mapState.js (1,500トークン)
- js/utils/canvas.js (450トークン)
合計: 5,950トークン (削減率: 82%)
```

### シナリオ例: 座標変換の修正

Before (リファクタリング前):
```
必要なファイル: index.html (32,645トークン)
```

After (リファクタリング後):
```
必要なファイル:
- js/utils/coordinates.js (1,100トークン)
- js/state/mapState.js (1,500トークン)
合計: 2,600トークン (削減率: 92%)
```

## 関連ドキュメント

- [AI_GUIDELINES.md](./AI_GUIDELINES.md) - 生成AI向け開発ガイドライン
- [README.md](../README.md) - プロジェクト概要
- [MAP-EDITOR.md](./MAP-EDITOR.md) - 2Dマップエディタ - オブジェクトプロパティ管理
- [OPERATION-CATALOG.md](./OPERATION-CATALOG.md) - 操作カタログ仕様
