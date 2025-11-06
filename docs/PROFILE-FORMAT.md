# プロファイル形式仕様書

## 概要

プロファイルは、マップデータ、オブジェクト、レイヤー、描画情報などの状態を保存・復元するためのJSON形式のデータ構造です。

## バージョン

- **現在のバージョン**: 1.0
- **互換性**: 古いプロファイル（version フィールドがない）も読み込み可能で、自動的にマイグレーションされます

## プロファイル構造

### トップレベルフィールド

```json
{
  "name": "プロファイル名",
  "timestamp": 1234567890123,
  "version": "1.0",
  "image": "data:image/png;base64,...",
  "imageFileName": "map.pgm",
  "yamlFileName": "map.yaml",
  "metadata": { ... },
  "mapBounds": { ... },
  "imageCropOffset": { ... },
  "originalImageSize": { ... },
  "scale": 1.0,
  "offsetX": 0,
  "offsetY": 0,
  "layers": { ... },
  "overlaySettings": { ... },
  "layerStack": [ ... ],
  "drawingState": { ... },
  "rectangleToolState": { ... }
}
```

### フィールド詳細

#### 基本情報

| フィールド | 型 | 説明 |
|----------|----|----|
| `name` | string | プロファイル名（一意識別子） |
| `timestamp` | number | 保存日時（Unixタイムスタンプ、ミリ秒） |
| `version` | string | プロファイル形式のバージョン |

#### マップデータ

| フィールド | 型 | 説明 |
|----------|----|----|
| `image` | string\|null | Base64エンコードされた画像データ（PNG形式） |
| `imageFileName` | string\|null | 画像ファイル名 |
| `yamlFileName` | string\|null | YAMLメタデータファイル名 |
| `metadata` | object\|null | YAMLメタデータの解析結果 |

#### マップ境界・クロップ情報

| フィールド | 型 | 説明 |
|----------|----|----|
| `mapBounds` | object\|null | マップの境界情報（3Dレンダラー用） |
| `imageCropOffset` | object\|null | クロップオフセット情報 |
| `originalImageSize` | object\|null | 元の画像サイズ |

#### ビューポート

| フィールド | 型 | 説明 |
|----------|----|----|
| `scale` | number | 拡大率（デフォルト: 1.0） |
| `offsetX` | number | X方向オフセット（ピクセル） |
| `offsetY` | number | Y方向オフセット（ピクセル） |

#### レイヤー設定

| フィールド | 型 | 説明 |
|----------|----|----|
| `layers` | object | レイヤーの表示状態 |
| `layers.image` | boolean | 画像レイヤーの表示 |
| `layers.metadataOverlay` | boolean | メタデータオーバーレイの表示 |

#### オーバーレイ設定

| フィールド | 型 | 説明 |
|----------|----|----|
| `overlaySettings` | object | オーバーレイの設定 |
| `overlaySettings.showGrid` | boolean | グリッド表示 |
| `overlaySettings.gridSpacingMeters` | number | グリッド間隔（メートル） |
| `overlaySettings.showOrigin` | boolean | 原点表示 |
| `overlaySettings.showScaleBar` | boolean | スケールバー表示 |

#### レイヤースタック

`layerStack` は、描画レイヤー、四角形レイヤーなどの情報を含む配列です。

```json
[
  {
    "id": "layer-1",
    "name": "レイヤー名",
    "type": "drawing" | "rectangle" | "image",
    "permanent": false,
    "visible": true,
    "opacity": 1.0,
    "canvasData": "data:image/png;base64,...",
    "strokes": [],
    "collapsed": false,
    "parentId": null,
    "rectangleId": null,
    "children": [...]
  }
]
```

#### 描画ツール設定

```json
{
  "color": "#FF0000",
  "brushSize": 5
}
```

#### 四角形ツール設定

```json
{
  "enabled": false,
  "rectangles": [
    {
      "id": "rect-1",
      "name": "四角形 1",
      "x": 100,
      "y": 100,
      "width": 50,
      "height": 50,
      "rotation": 0,
      "color": "#FF0000",
      "selected": false,
      "objectType": "shelf",
      "heightMeters": 2.0,
      "frontDirection": "top",
      "objectProperties": {
        "shelfLevels": 3,
        "shelfDividers": 0,
        "shelfType": "open"
      },
      "commonProperties": {
        "name": "キッチンの棚",
        "description": "食器を収納",
        "tags": ["キッチン", "収納"],
        "customColor": null,
        "isAccessible": true,
        "isMovable": false,
        "weightKg": 50,
        "material": "木製",
        "surfaceType": "smooth",
        "hasObstacle": false,
        "isNoGoZone": false,
        "createdAt": "2024-01-01T00:00:00.000Z",
        "updatedAt": "2024-01-01T00:00:00.000Z"
      }
    }
  ],
  "nextRectangleId": 2
}
```

### 四角形オブジェクトの詳細

四角形オブジェクトは、以下のプロパティを持ちます：

#### 基本プロパティ

| フィールド | 型 | 説明 |
|----------|----|----|
| `id` | string | 四角形の一意識別子（例: "rect-1"） |
| `name` | string | 四角形の名前（例: "四角形 1"） |
| `x` | number | 中心X座標（画像ピクセル） |
| `y` | number | 中心Y座標（画像ピクセル） |
| `width` | number | 幅（画像ピクセル） |
| `height` | number | 高さ（画像ピクセル） |
| `rotation` | number | 回転角度（度） |
| `color` | string | 四角形の色（HEX形式） |
| `selected` | boolean | 選択状態 |

#### オブジェクト情報

| フィールド | 型 | 説明 |
|----------|----|----|
| `objectType` | string | オブジェクトタイプ（"none", "shelf", "box", "table", "door", "wall"） |
| `heightMeters` | number | 高さ（メートル） |
| `frontDirection` | string | 前面方向（"top", "right", "bottom", "left"） |
| `objectProperties` | object | オブジェクトタイプ固有のプロパティ |

#### オブジェクトタイプ別プロパティ

##### 棚（shelf）

```json
{
  "shelfLevels": 3,
  "shelfDividers": 0,
  "shelfType": "open"
}
```

##### 箱（box）

```json
{
  "boxHasLid": true,
  "boxStackable": true,
  "boxMaterial": "cardboard"
}
```

##### テーブル（table）

```json
{
  "tableShape": "rectangular",
  "tableLegCount": 4,
  "tableHasDrawer": false
}
```

##### 扉（door）

```json
{
  "doorOpenDirection": "push",
  "doorSwingDirection": "inward",
  "doorType": "single"
}
```

##### 壁（wall）

```json
{
  "wallMaterial": "drywall",
  "wallThicknessCm": 10,
  "wallHasWindow": false
}
```

#### 共通拡張プロパティ

すべてのオブジェクトタイプに共通のプロパティです。

| フィールド | 型 | 説明 |
|----------|----|----|
| `name` | string | オブジェクト名（ユーザー定義） |
| `description` | string | 説明・メモ |
| `tags` | string[] | タグ（検索・フィルタ用） |
| `customColor` | string\|null | カスタム色（nullの場合はデフォルト色） |
| `isAccessible` | boolean | ロボットがアクセス可能か |
| `isMovable` | boolean | 移動可能か |
| `weightKg` | number | 重量（kg） |
| `material` | string | 材質の詳細説明 |
| `surfaceType` | string | 表面タイプ（"smooth", "rough", "fragile"） |
| `hasObstacle` | boolean | 障害物としてマークするか |
| `isNoGoZone` | boolean | 進入禁止エリアか |
| `createdAt` | string | 作成日時（ISO 8601形式） |
| `updatedAt` | string | 更新日時（ISO 8601形式） |

## エクスポート/インポート

### エクスポート

プロファイルは、JSON形式でファイルにエクスポートされます。

- **ファイル名**: `{プロファイル名}_profile.json`
- **エンコーディング**: UTF-8（BOM付き）
- **Content-Type**: `application/json;charset=utf-8`

### インポート

エクスポートされたJSONファイルをインポートすると、プロファイルがLocalStorageに保存され、自動的に読み込まれます（ユーザーが確認ダイアログでOKした場合）。

## 互換性

### 古いプロファイルのマイグレーション

`version` フィールドがない古いプロファイルを読み込むと、以下の処理が自動的に行われます：

1. **四角形オブジェクトのプロパティ補完**
   - `objectType` が存在しない → `"none"` に設定
   - `heightMeters` が存在しない → `0.5` に設定
   - `frontDirection` が存在しない → `"top"` に設定
   - `objectProperties` が存在しない → `{}` に設定
   - `commonProperties` が存在しない → デフォルト値で初期化

2. **タイムスタンプの補完**
   - `createdAt` と `updatedAt` が存在しない場合、現在時刻で初期化

## ストレージ

### LocalStorage

プロファイルは、ブラウザのLocalStorageに保存されます。

- **キー**: `map_profile_{プロファイル名}`
- **最大サイズ**: 5MB
- **制限**: ブラウザのLocalStorage容量制限に依存（通常5-10MB）

### プロファイル一覧

- **キー**: `map_profiles_list`
- **内容**: プロファイル名の配列

### 最後に使用したプロファイル

- **キー**: `map_last_profile`
- **内容**: プロファイル名

### 現在のプロファイル

- **キー**: `map_current_profile`
- **内容**: プロファイル名

## 関連ファイル

- `apps/frontend/static/js/modules/profileManager.js` - プロファイル管理機能
- `apps/frontend/static/js/ui/controls.js` - プロファイルUI制御
- `apps/frontend/static/js/models/objectTypes.js` - オブジェクトタイプ定義
- `apps/frontend/static/js/modules/rectangleManager.js` - 四角形管理
