# リファクタリング戦略書

## 現状分析

### 問題点
- **単一ファイルの巨大化**: index.html が 3,238行、32,645トークンに及ぶ
  - CSS: 788行
  - HTML: 160行
  - JavaScript: 2,278行
  - 約70個の関数が1つのスクリプトタグに集約

### 影響
- **生成AIのトークン消費**: 単一ファイル読み込みで大量のトークンを消費
- **可読性の低下**: 人間・AIともにコード理解が困難
- **保守性の低下**: 変更箇所の特定や影響範囲の把握が困難
- **モジュール化不可**: 機能単位での再利用やテストが困難

## リファクタリング戦略

### 設計原則（生成AI最適化）

1. **責務単位での分割**: 1ファイル = 1責務
2. **適切なファイルサイズ**: 各ファイル500-1000行以下、理想は300-500行
3. **明確な命名規則**: ファイル名・関数名から機能を推測可能に
4. **依存関係の明示化**: import/export で依存関係を明確に
5. **ドキュメンテーション**: 各モジュールに明確なコメントとJSDoc

### ディレクトリ構造

```
apps/frontend/
├── static/
│   ├── index.html                  # メインHTML（軽量化）
│   ├── styles/                     # CSS モジュール
│   │   ├── base.css               # 基本スタイル、リセット
│   │   ├── layout.css             # レイアウト、グリッド
│   │   ├── components.css         # 共通コンポーネント
│   │   ├── map-viewer.css         # マップビューア専用
│   │   ├── drawing-tools.css      # 描画ツール専用
│   │   ├── layers-panel.css       # レイヤーパネル専用
│   │   ├── metadata-overlay.css   # メタデータオーバーレイ
│   │   └── tabs.css               # タブUI
│   ├── js/                        # JavaScript モジュール
│   │   ├── main.js                # エントリーポイント、初期化
│   │   ├── config.js              # 設定・定数
│   │   ├── state/
│   │   │   └── mapState.js        # 状態管理
│   │   ├── modules/
│   │   │   ├── layerManager.js    # レイヤー管理
│   │   │   ├── drawingTools.js    # 描画ツール
│   │   │   ├── fileLoader.js      # ファイル読み込み
│   │   │   ├── metadataDisplay.js # メタデータ表示
│   │   │   ├── overlayRenderer.js # オーバーレイ描画
│   │   │   ├── viewportControl.js # ズーム/パン
│   │   │   ├── historyManager.js  # アンドゥ/リドゥ
│   │   │   └── apiClient.js       # API通信
│   │   ├── utils/
│   │   │   ├── coordinates.js     # 座標変換
│   │   │   ├── canvas.js          # Canvas操作ユーティリティ
│   │   │   ├── imageProcessing.js # 画像処理（PGMパーサーなど）
│   │   │   └── formatting.js      # フォーマット関数
│   │   └── ui/
│   │       ├── tabs.js            # タブ切り替え
│   │       ├── controls.js        # UI コントロール
│   │       └── events.js          # イベントハンドラー
│   └── assets/                    # 静的アセット
│       └── icons/
└── server.py                      # フロントエンドサーバー
```

### モジュール分割計画

#### 1. CSS分割（8ファイル）
| ファイル | 行数目安 | 内容 |
|---------|---------|------|
| base.css | ~80 | リセット、基本スタイル |
| layout.css | ~100 | コンテナ、グリッド |
| components.css | ~150 | ボタン、入力フィールド |
| map-viewer.css | ~100 | キャンバス、マップコンテナ |
| drawing-tools.css | ~120 | ツールパレット、カラーピッカー |
| layers-panel.css | ~120 | レイヤーリスト、コントロール |
| metadata-overlay.css | ~100 | メタデータパネル |
| tabs.css | ~80 | タブナビゲーション |

#### 2. JavaScript分割（17ファイル）

##### コア機能
| ファイル | 行数目安 | 関数数 | 主要機能 |
|---------|---------|-------|---------|
| **main.js** | ~100 | 2-3 | アプリ初期化、モジュール統合 |
| **config.js** | ~50 | 0 | 定数、設定値 |
| **mapState.js** | ~150 | 3-5 | 状態管理、ゲッター/セッター |

##### 機能モジュール
| ファイル | 行数目安 | 関数数 | 主要機能 |
|---------|---------|-------|---------|
| **layerManager.js** | ~400 | 10-12 | レイヤーCRUD、描画、更新 |
| **drawingTools.js** | ~350 | 8-10 | ツール選択、描画処理、測量 |
| **fileLoader.js** | ~400 | 7-9 | ファイル読み込み、PGMパーサー |
| **metadataDisplay.js** | ~200 | 5-6 | メタデータ表示、トグル |
| **overlayRenderer.js** | ~350 | 8-10 | グリッド、原点、スケールバー |
| **viewportControl.js** | ~250 | 6-8 | ズーム、パン、ビューリセット |
| **historyManager.js** | ~200 | 5-6 | アンドゥ/リドゥ、履歴管理 |
| **apiClient.js** | ~150 | 3-4 | API通信、データ取得 |

##### ユーティリティ
| ファイル | 行数目安 | 関数数 | 主要機能 |
|---------|---------|-------|---------|
| **coordinates.js** | ~150 | 3-4 | 座標変換関数群 |
| **canvas.js** | ~100 | 2-3 | Canvas操作ヘルパー |
| **imageProcessing.js** | ~200 | 3-4 | PGMパーサー、画像変換 |
| **formatting.js** | ~80 | 3-4 | 数値フォーマット |

##### UI層
| ファイル | 行数目安 | 関数数 | 主要機能 |
|---------|---------|-------|---------|
| **tabs.js** | ~100 | 2-3 | タブ切り替え |
| **controls.js** | ~150 | 5-6 | UI コントロール操作 |
| **events.js** | ~200 | 1 + listeners | イベントリスナー統合 |

### モジュール間の依存関係

```
main.js
├── config.js
├── mapState.js
├── modules/
│   ├── layerManager.js → mapState, canvas, coordinates
│   ├── drawingTools.js → mapState, layerManager, historyManager, canvas
│   ├── fileLoader.js → mapState, layerManager, metadataDisplay, imageProcessing
│   ├── metadataDisplay.js → mapState, overlayRenderer
│   ├── overlayRenderer.js → mapState, coordinates, formatting
│   ├── viewportControl.js → mapState, layerManager
│   ├── historyManager.js → mapState
│   └── apiClient.js → config
├── utils/
│   ├── coordinates.js
│   ├── canvas.js
│   ├── imageProcessing.js
│   └── formatting.js
└── ui/
    ├── tabs.js
    ├── controls.js → layerManager, drawingTools, fileLoader, viewportControl
    └── events.js → drawingTools, viewportControl, controls
```

## 生成AI向けガイドライン

### ファイル構造の最適化
1. **ファイルヘッダー**: 各ファイルに目的、依存関係、エクスポート内容を記載
2. **セクション分け**: 機能グループごとにコメント区切り
3. **JSDoc**: すべての public 関数に JSDoc を記載
4. **型ヒント**: 可能な限り型情報をコメントで提供

### コーディング規約
- **命名**: 明確で説明的な名前（略語を避ける）
- **関数サイズ**: 1関数20-50行以内を目標
- **コメント**: 「なぜ」を説明（「何を」ではなく）
- **エラーハンドリング**: 明示的なエラー処理

### ドキュメンテーション
- **AI_GUIDELINES.md**: 生成AI向けの開発ガイド
- **ARCHITECTURE.md**: システムアーキテクチャ
- **MODULE_INDEX.md**: モジュール索引と責務一覧

## 実装順序

### Phase 1: 基盤整備
1. ディレクトリ構造の作成
2. config.js の抽出
3. mapState.js の抽出

### Phase 2: ユーティリティ分割
4. coordinates.js の抽出
5. formatting.js の抽出
6. canvas.js の抽出
7. imageProcessing.js の抽出（PGMパーサー）

### Phase 3: コア機能分割
8. layerManager.js の抽出
9. historyManager.js の抽出
10. viewportControl.js の抽出

### Phase 4: 描画機能分割
11. drawingTools.js の抽出
12. overlayRenderer.js の抽出
13. metadataDisplay.js の抽出

### Phase 5: I/O機能分割
14. fileLoader.js の抽出
15. apiClient.js の抽出

### Phase 6: UI層分割
16. tabs.js の抽出
17. controls.js の抽出
18. events.js の抽出
19. main.js の作成

### Phase 7: CSS分割
20. base.css の抽出
21. layout.css の抽出
22. components.css の抽出
23. map-viewer.css の抽出
24. drawing-tools.css の抽出
25. layers-panel.css の抽出
26. metadata-overlay.css の抽出
27. tabs.css の抽出

### Phase 8: HTML整理
28. index.html の軽量化
29. CSS/JSファイルのインポート設定

### Phase 9: ドキュメント整備
30. AI_GUIDELINES.md の作成
31. MODULE_INDEX.md の作成
32. README.md の更新

### Phase 10: 検証
33. 動作確認
34. リファクタリング前後の比較

## 成功指標

- index.html のトークン数: 32,645 → 3,000以下（90%削減）
- 最大ファイルサイズ: 3,238行 → 500行以下
- ファイル数: 1 → 約30ファイル
- 生成AIによる単一機能理解: 全ファイル取得 → 必要ファイルのみ取得（5-10ファイル程度）

## リスク管理

### リスク
1. 動作不良の発生
2. 開発時間の超過
3. 依存関係の複雑化

### 対策
1. 段階的リファクタリング + 都度動作確認
2. 自動化可能な部分はスクリプトで処理
3. 依存関係図の作成と定期的レビュー
