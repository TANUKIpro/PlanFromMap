# 操作カタログ仕様書

## 概要

操作カタログは、HSRが環境内の可動要素（ドア、引き出し、家電等）を操作するための**完全な仕様**を事前定義するシステムです。

## 設計思想

### 問題意識

ロボットが扉を開ける際、以下の情報を毎回視覚的に推定するのは非効率：

- ノブの回転方向と角度
- 押すのか引くのか
- ヒンジの位置
- 最大開度
- 必要な力

### 解決策

**人間が事前に完全な操作仕様を登録** → **ロボットは識別のみで即座に正しい操作を実行**

## データ構造

### 1. 基本構造

```typescript
OperationCatalogEntry {
  id: "kitchen_upper_left_door"
  type: "cabinet_door"
  location: "kitchen"
  operation_spec: { ... }
  state_detection: { ... }
  meta: { ... }
}
```

### 2. 操作仕様（operation_spec）

#### 2.1 事前動作（pre_actions）

扉を開ける前に必要な動作：

```json
"pre_actions": [
  {
    "type": "rotate_knob",
    "target": "main_knob",
    "params": {
      "axis": [0, 0, 1],
      "angle_deg": -15,
      "force_N": 5
    },
    "required": true
  }
]
```

**タイプ一覧**：
- `rotate_knob`: ノブを回転
- `push_button`: ボタンを押す
- `lift_latch`: ラッチを持ち上げる
- `unlock`: ロック解除
- `release_catch`: キャッチを外す

#### 2.2 主動作（main_action）

メインとなる開閉動作：

```json
"main_action": {
  "type": "pull",
  "params": {
    "direction": [1, 0, 0],
    "max_distance_mm": 800,
    "nominal_speed_mm_s": 100,
    "force_threshold_N": 30,
    "use_compliance": true
  }
}
```

**タイプ一覧**：
- `pull` / `push`: 引く/押す
- `slide_left` / `slide_right`: 横スライド
- `lift` / `lower`: 上げる/下げる
- `rotate`: 回転

#### 2.3 機構情報（mechanism）

物理的な構造：

```json
"mechanism": {
  "joint_type": "revolute",
  "axis_origin": [2.1, 0.5, 1.0],
  "axis_direction": [0, 0, 1],
  "range": {
    "min": 0,
    "max": 90,
    "unit": "deg"
  },
  "rest_position": 0,
  "self_closing": false
}
```

#### 2.4 把持仕様（grasp_specs）

どこをどう掴むか：

```json
"grasp_specs": [
  {
    "id": "primary_handle",
    "type": "handle",
    "pose": {
      "x": 2.3, "y": 0.55, "z": 1.1,
      "yaw": 3.14
    },
    "approach": [-1, 0, 0],
    "grasp_type": "cylindrical",
    "opening_mm": 40,
    "priority": 1
  }
]
```

### 3. 状態検出（state_detection）

開閉状態の判定方法：

```json
"state_detection": {
  "closed_indicators": [
    {
      "type": "visual",
      "feature": "no_gap_visible",
      "confidence_weight": 0.8
    }
  ],
  "open_indicators": [
    {
      "type": "visual", 
      "feature": "interior_visible",
      "confidence_weight": 0.9
    }
  ]
}
```

## 登録フロー

### 1. UIでの登録

```
1. 基本情報入力
   - 名前: "キッチン上段左扉"
   - タイプ: cabinet_door
   - 場所: kitchen

2. 動作パターン選択
   [🚪回転扉] ← 選択

3. 詳細設定
   - ノブ: [不要] ← 選択
   - 開き方: [手前引き] ← 選択  
   - 最大角度: [90°] ← 選択

4. 3Dマーキング
   - ハンドル位置をクリック
   - ヒンジ軸を指定

5. プレビュー確認
   - アニメーション再生
   - パラメータ微調整

6. 保存
```

### 2. テンプレートからの登録

```typescript
// IKEAメトッドシリーズのテンプレート適用
const operation = applyTemplate('IKEA_METOD_40x80', {
  id: 'kitchen_cab_1',
  location: 'kitchen',
  pose_offset: { x: 0.1, y: 0, z: 0 }
});
```

### 3. 実測値での更新

```bash
# キャリブレーションモード
npm run catalog:calibrate kitchen_upper_left_door

# 実測値入力
> Actual max opening angle (deg): 85
> Actual knob rotation (deg): 12
> Handle position correction (x,y,z mm): 10,-5,0
```

## クエリインターフェース

### 基本取得

```sql
GET Operation FOR 'kitchen_door'
```

**レスポンス**:
```json
{
  "success": true,
  "operation": { /* 完全な操作仕様 */ },
  "confidence": "verified",
  "estimated_time_s": 3.5
}
```

### 条件検索

```sql
FIND Operations WHERE 
  type = 'drawer' AND 
  location = 'bedroom' AND
  verified = true
```

### 代替取得

```sql
SUGGEST Alternative FOR 'broken_door_handle'
WHERE condition = 'handle_missing'
```

## 実行フロー

```python
# HSR側のコード例
from semantic_map import OperationExecutor

# 1. 操作仕様を取得
spec = catalog.get_operation('refrigerator_door')

# 2. 実行計画生成
plan = executor.create_plan(spec)
# → [rotate_knob, pull_door, hold_open]

# 3. 実行
for action in plan:
    result = executor.execute_action(action)
    if not result.success:
        # フォールバック戦略を実行
        executor.try_fallback(action, result.error)

# 4. 状態確認
state = executor.verify_state('open')
```

## プリセット一覧

### 家具

| カテゴリ | テンプレートID | 説明 |
|---------|--------------|------|
| キャビネット | CABINET_REVOLUTE_90 | 標準的な90度開きキャビネット |
| 引き出し | DRAWER_STANDARD | 標準引き出し（400-600mm） |
| 冷蔵庫 | FRIDGE_MAGNETIC | マグネット式冷蔵庫扉 |
| 食洗器 | DISHWASHER_PULLDOWN | プルダウン式食洗器 |

### パラメータ調整

```javascript
// テンプレートのカスタマイズ
const customized = {
  ...CABINET_REVOLUTE_90,
  mechanism: {
    ...CABINET_REVOLUTE_90.mechanism,
    range: { min: 0, max: 120, unit: 'deg' } // 拡張
  }
};
```

## ベストプラクティス

### 1. 命名規則

```
{room}_{position}_{type}_{index}
例: kitchen_upper_left_door_1
```

### 2. 優先度設定

```typescript
grasp_specs: [
  { priority: 1, ... }, // 主ハンドル
  { priority: 2, ... }, // 副ハンドル  
  { priority: 3, ... }  // 緊急用エッジ把持
]
```

### 3. 安全マージン

```json
"constraints": {
  "requires_clearance_mm": 100,
  "max_opening_speed_mm_s": 150,
  "collision_zones": [...]
}
```

## トラブルシューティング

### よくある問題

#### Q: 登録した仕様通りに動かない

**確認項目**:
1. 座標系の一致（map frame基準か確認）
2. 単位の確認（mm/m、deg/rad）
3. approach方向の符号

#### Q: 把持に失敗する

**対策**:
1. 複数の把持点を登録
2. pre_grasp_offsetを調整
3. approach方向を見直し

#### Q: 途中で止まる

**原因と対策**:
- force_threshold_Nが低すぎる → 適切な値に調整
- 干渉している → collision_zonesを設定
- 機構が固い → nominal_speedを下げる

## 付録

### A. 座標系定義

- 原点: マップ原点
- X軸: 前方向（ロボット基準）
- Y軸: 左方向
- Z軸: 上方向
- 角度: 反時計回りが正

### B. 推奨パラメータ

| パラメータ | 軽い扉 | 重い扉 | 引き出し |
|----------|--------|--------|---------|
| speed_mm_s | 150 | 80 | 100 |
| force_N | 20 | 50 | 30 |
| compliance | true | true | false |

### C. 拡張予定機能

- 音声フィードバック統合
- 学習による自動調整
- マルチロボット共有
