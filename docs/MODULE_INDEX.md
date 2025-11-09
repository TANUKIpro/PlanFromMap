# モジュール索引

## ディレクトリ構造

```
apps/frontend/static/
├── index.html                      # メインHTML (184行)
├── index.html.backup               # 元のバックアップ (3,238行)
├── styles/                         # CSSモジュール (10ファイル)
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
├── js/                            # JavaScriptモジュール (31ファイル)
│   ├── main.js                    # 98行 - エントリーポイント
│   ├── config.js                  # 150行 - 設定・定数
│   ├── state/
│   │   └── mapState.js            # 250行 - 状態管理
│   ├── models/                    # データモデル (1ファイル)
│   │   └── objectTypes.js         # 500行 - オブジェクトタイプ定義
│   ├── modules/                   # コア機能 (19ファイル)
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
│   │   ├── rectangleEditCore.js   # 783行 - 四角形編集ロジック（当たり判定、マウスイベント）
│   │   ├── rectangleMeasure.js    # 238行 - 測量モード専用UI・ロジック
│   │   ├── rectangleInteraction.js # 44行 - 四角形インタラクション（再エクスポート）
│   │   ├── objectPropertyManager.js # 350行 - オブジェクトプロパティ管理
│   │   ├── threeDModels.js        # 1944行 - 3Dモデル描画ロジック
│   │   ├── threeDPreview.js       # 428行 - プロパティパネル用3Dプレビュー
│   │   ├── threeDViewCore.js      # 1177行 - メインビュー管理、イベント処理
│   │   └── threeDRenderer.js      # 66行 - 3D描画（再エクスポート）
│   ├── utils/                     # ユーティリティ (5ファイル)
│   │   ├── coordinates.js         # 133行 - 座標変換
│   │   ├── formatting.js          # 94行 - フォーマット
│   │   ├── canvas.js              # 54行 - Canvas操作
│   │   ├── imageProcessing.js     # 190行 - 画像処理
│   │   └── threeDUtils.js         # 188行 - 3D描画用ユーティリティ
│   └── ui/                        # UI制御 (9ファイル)
│       ├── tabs.js                # 48行 - タブ切り替え
│       ├── objectPropertyPanel.js # 600行 - オブジェクトプロパティパネル
│       ├── controls.js            # 163行 - UIコントロール
│       ├── toast.js               # 143行 - トーストメッセージ
│       ├── events.js              # 404行 - イベント管理
│       ├── viewCube.js            # 336行 - ビューキューブ（3D回転UI）
│       ├── objectCatalogList.js   # 225行 - カタログリスト表示・フィルタ
│       ├── objectCatalogEditor.js # 365行 - カタログ詳細編集パネル
│       ├── objectCatalogPreview.js # 600行 - カタログ3Dプレビュー
│       └── objectCatalog.js       # 65行 - オブジェクトカタログ（再エクスポート）
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

### modules/rectangleEditCore.js (783行)
**責務**: 四角形の当たり判定、マウスイベント、編集操作

**主要関数**:
- `hitTestRectangle(x, y, rectangle)` - 四角形の当たり判定
- `handleRectangleMouseDown(e)` - マウス押下時の処理
- `handleRectangleMouseMove(e)` - マウス移動時の処理
- `handleRectangleMouseUp(e)` - マウスリリース時の処理
- `handleResize(dx, dy, handle)` - リサイズ操作
- `handleMove(dx, dy)` - 移動操作
- `handleRotate(dx, dy)` - 回転操作

**編集モード**:
- 移動: 四角形本体をドラッグ
- リサイズ: 8方向のハンドルをドラッグ
- 回転: 回転ハンドルをドラッグ

**依存**: rectangleManager.js, mapState.js, coordinates.js

### modules/rectangleMeasure.js (238行)
**責務**: 測量モード専用のUI・ロジック

**主要関数**:
- `promptEdgeLength(rectangleId, edge)` - エッジ長の入力を促す
- `showEdgeLengthInputUI(rectangleId, edge, canvasX, canvasY)` - 長さ入力UIの表示
- `getEdgeMidpoint(rectangle, edge)` - エッジの中点を取得

**測量機能**:
- エッジクリックで長さ入力UIを表示
- 入力値に基づいて四角形のサイズを自動調整
- スケール計算による実世界サイズの設定

**依存**: rectangleManager.js, config.js, coordinates.js

### modules/rectangleInteraction.js (44行)
**責務**: 後方互換性のための再エクスポート

**再エクスポート**:
- rectangleEditCore.js からすべての編集関数
- rectangleMeasure.js からすべての測量関数

**依存**: rectangleEditCore.js, rectangleMeasure.js

### modules/threeDModels.js (1944行)
**責務**: 3Dモデル描画ロジック（メインとプレビュー両方）

**カテゴリ別3Dモデル描画（メインビュー）**:
- `draw3DShelf(ctx, obj, isoFunc, rotation, zoom)` - 棚（前面開口、棚板表示）
- `draw3DBox(ctx, obj, isoFunc, rotation, zoom)` - 箱（上部開放）
- `draw3DTable(ctx, obj, isoFunc, rotation, zoom)` - テーブル（天板+脚）
- `draw3DDoor(ctx, obj, isoFunc, rotation, zoom)` - 扉（薄型）
- `draw3DWall(ctx, obj, isoFunc, rotation, zoom)` - 壁（標準）
- `drawBox(ctx, x, y, z, w, d, h, color, isoFunc, rotation, zoom)` - 汎用ボックス

**プレビュー用3Dモデル描画**:
- `drawPreviewShelf()`, `drawPreviewBox()`, `drawPreviewTable()`, `drawPreviewDoor()`, `drawPreviewWall()`
- 各カテゴリのプレビュー版関数

**特徴**:
- パラメータ駆動型のモデル生成（段数、仕切り数、脚の数など）
- 奥行きソートによる正しい描画順序
- 色の明暗処理による立体感の表現

**依存**: threeDUtils.js

### modules/threeDPreview.js (428行)
**責務**: プロパティパネル用3Dプレビュー管理

**主要関数**:
- `initializePropertyPreview()` - プロパティパネル内プレビュー初期化
- `renderPropertyPreview(rectangleId)` - プレビュー描画
- `getPreviewState()` - プレビュー状態の取得
- `drawPreviewGrid(ctx, gridSize)` - グリッド描画
- `drawPreviewFrontDirection(ctx, rectangle)` - 前面方向インジケーター描画

**インタラクション**:
- ホイールでズーム（回転固定）
- リアルタイムプロパティ反映

**依存**: threeDModels.js, threeDUtils.js, mapState.js

### modules/threeDViewCore.js (1177行)
**責務**: メインビューの管理、イベント処理、床面描画

**主要関数**:
- `initialize3DView()` - 3Dビューの初期化
- `render3DScene()` - 3Dシーン全体の描画
- `update3DObject(rectangleId)` - 特定オブジェクトの更新
- `resize3DView()` - ビューのリサイズ
- `set3DViewRotation(angle)` - 回転角度の設定
- `reset3DView()` - ビューのリセット
- `select3DObject(rectangleId)` - オブジェクトの選択
- `deselect3DObject()` - 選択解除
- `goto2DMap()` - 2Dマップへ遷移
- `gotoObjectCatalog()` - オブジェクトカタログへ遷移

**インタラクション**:
- マウスドラッグで回転
- ホイールでズーム
- クリックでオブジェクト選択
- 右方向へドラッグでシーンが右回転（直感的操作）

**床面描画**:
- グリッド描画
- 原点マーカー
- 前面方向インジケーター

**状態管理**:
- `view3DState`: メイン3Dビュー状態（回転、ズーム、選択状態）

**依存**: threeDModels.js, threeDUtils.js, mapState.js, rectangleManager.js, objectPropertyManager.js, viewCube.js

### modules/threeDRenderer.js (66行)
**責務**: 後方互換性のための再エクスポート

**再エクスポート**:
- threeDViewCore.js からすべてのメインビュー関数
- threeDPreview.js からすべてのプレビュー関数
- threeDModels.js からすべてのモデル描画関数

**依存**: threeDViewCore.js, threeDPreview.js, threeDModels.js

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

### utils/threeDUtils.js (188行)
**責務**: 3D描画用ユーティリティ関数

**主要関数**:
- `worldToIso(x, y, z)` - ワールド座標→等角投影座標変換（メインビュー用）
- `worldToPreviewIso(x, y, z)` - ワールド座標→等角投影座標変換（プレビュー用）
- `lightenColor(color, factor)` - 色を明るくする
- `darkenColor(color, factor)` - 色を暗くする
- `sortFacesByDepth(faces)` - 面を奥行き順にソート（描画順序決定）
- `applyRotation(x, y, z, rotation)` - 回転変換を適用

**等角投影座標系**:
- X軸反転により2Dマップとの整合性を維持
- 右方向ドラッグで右回転する直感的な回転操作

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

### ui/viewCube.js (336行)
**責務**: ビューキューブ（3D回転UI）

**主要関数**:
- `initializeViewCube(container, onRotationChange)` - ビューキューブの初期化
- `updateViewCube(rotation)` - 回転状態の更新
- `drawViewCube(ctx, rotation)` - ビューキューブの描画
- `handleViewCubeClick(x, y)` - クリック処理（面選択）

**機能**:
- 3Dビューの現在の回転状態を視覚的に表示
- 面をクリックして視点を即座に変更
- 標準視点への素早いアクセス（正面、背面、左、右、上、下）

**依存**: なし

### ui/objectCatalogList.js (225行)
**責務**: カタログリストの表示・フィルタ・選択

**主要関数**:
- `initializeObjectCatalog()` - オブジェクトカタログの初期化
- `updateObjectCatalog(filterText)` - カタログリストの更新
- `createCatalogItem(rectangle)` - カタログアイテムの生成
- `selectCatalogItem(rectangleId)` - カタログアイテムの選択

**機能**:
- オブジェクトタイプ別のフィルタリング
- テキスト検索
- サムネイルプレビュー
- 選択状態の視覚的フィードバック

**依存**: rectangleManager.js, objectTypes.js

### ui/objectCatalogEditor.js (365行)
**責務**: 詳細編集パネル、フォーム、保存

**主要関数**:
- `showCatalogDetail(rectangleId)` - 詳細パネルの表示
- `createFormInput(field, currentValue, rectangle)` - フォーム入力の生成
- `saveCatalogEdit(rectangleId)` - 編集内容の保存
- `switchCatalogTab(tabName)` - タブ切り替え（プロパティ/3Dプレビュー）

**機能**:
- カテゴリ別の動的フォーム生成
- リアルタイムバリデーション
- 3Dプレビューとの連携
- 編集履歴の管理

**依存**: rectangleManager.js, objectTypes.js, objectPropertyManager.js

### ui/objectCatalogPreview.js (600行)
**責務**: 3Dプレビュー（リサイザー含む）

**主要関数**:
- `updatePreview(rectangleId)` - 3Dプレビューの更新
- `initializeCatalogResizer()` - リサイザーの初期化
- `handlePreviewMouseDown(e)` - マウス押下（回転開始）
- `handlePreviewMouseMove(e)` - マウス移動（回転中）
- `handlePreviewWheel(e)` - ホイール（ズーム）

**機能**:
- インタラクティブな3Dプレビュー（マウスドラッグで回転）
- リサイザーによるプレビューサイズ調整
- ズーム機能
- リアルタイムプロパティ反映

**依存**: threeDModels.js, threeDUtils.js, viewCube.js, rectangleManager.js

### ui/objectCatalog.js (65行)
**責務**: 後方互換性のための再エクスポート

**再エクスポート**:
- objectCatalogList.js からすべてのリスト関数
- objectCatalogEditor.js からすべての編集関数
- objectCatalogPreview.js からすべてのプレビュー関数

**依存**: objectCatalogList.js, objectCatalogEditor.js, objectCatalogPreview.js

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

## リファクタリングの成果

### 2024年11月 - 3Dレンダリングとインタラクションの分離

**目的**: 1250行の `threeDRenderer.js` を責務ごとに分離し、保守性を向上

**分離結果**:
- **utils/threeDUtils.js (188行)**: 座標変換、色処理などの純粋関数
- **modules/threeDModels.js (1944行)**: カテゴリ別3Dモデル描画ロジック
- **modules/threeDPreview.js (428行)**: プロパティパネル用プレビュー管理
- **modules/threeDViewCore.js (1177行)**: メインビュー管理とイベント処理
- **modules/threeDRenderer.js (66行)**: 後方互換性のための再エクスポート層

**効果**:
- 責務の明確化により、特定の機能（例: プレビュー機能のみ）の修正が容易に
- ユーティリティ関数の再利用性向上
- テストの容易性向上（純粋関数の分離）

### 2024年11月 - 四角形インタラクションの分離

**目的**: 四角形編集機能を責務ごとに分離

**分離結果**:
- **modules/rectangleEditCore.js (783行)**: 当たり判定、マウスイベント、編集操作
- **modules/rectangleMeasure.js (238行)**: 測量モード専用UI・ロジック
- **modules/rectangleInteraction.js (44行)**: 後方互換性のための再エクスポート層

**効果**:
- 編集機能と測量機能の分離により、各機能の独立した修正が可能
- 測量UIの改善が他の編集機能に影響しない

### 2024年11月 - オブジェクトカタログUIの分離

**目的**: オブジェクトカタログ機能を責務ごとに分離

**分離結果**:
- **ui/objectCatalogList.js (225行)**: リスト表示、フィルタ、選択
- **ui/objectCatalogEditor.js (365行)**: 詳細編集パネル、フォーム
- **ui/objectCatalogPreview.js (600行)**: 3Dプレビュー、リサイザー
- **ui/objectCatalog.js (65行)**: 後方互換性のための再エクスポート層
- **ui/viewCube.js (336行)**: ビューキューブUI（3D回転コントロール）

**効果**:
- リスト、編集、プレビューの独立した保守
- viewCube.jsの独立により、他の3Dビューでも再利用可能

### モジュール数の推移

| 時期 | ファイル数 | 平均行数 | 最大行数 |
|------|-----------|---------|---------|
| リファクタリング前 (index.html) | 1 | 3,238行 | 3,238行 |
| 初期分離後 | 27 | 約200行 | 1,250行 (threeDRenderer.js) |
| 2024年11月 | 31 | 約180行 | 1,944行 (threeDModels.js) |

### 今後の方針

- **再エクスポート層の活用**: 後方互換性を保ちつつ、段階的なリファクタリングを継続
- **pure function の分離**: 副作用のない関数を utils/ に集約し、テスト容易性を向上
- **UI層の更なる分離**: 必要に応じて大きなUIモジュールを分割

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

### シナリオ例: 3Dプレビュー機能の修正

Before (リファクタリング前):
```
必要なファイル: threeDRenderer.js (約10,000トークン)
```

After (2024年11月リファクタリング後):
```
必要なファイル:
- js/modules/threeDPreview.js (3,500トークン)
- js/utils/threeDUtils.js (1,500トークン)
- js/state/mapState.js (1,500トークン)
合計: 6,500トークン (削減率: 35%)
```

## 関連ドキュメント

- [AI_GUIDELINES.md](./AI_GUIDELINES.md) - 生成AI向け開発ガイドライン
- [README.md](../README.md) - プロジェクト概要
- [MAP-EDITOR.md](./MAP-EDITOR.md) - 2Dマップエディタ - オブジェクトプロパティ管理
- [OPERATION-CATALOG.md](./OPERATION-CATALOG.md) - 操作カタログ仕様
