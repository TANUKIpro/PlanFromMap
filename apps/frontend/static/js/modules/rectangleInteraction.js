/**
 * @file rectangleInteraction.js
 * @description 四角形とのインタラクション処理（後方互換性のための再エクスポート）
 *
 * 注意: このファイルは後方互換性のために維持されています。
 * 新しいコードでは、以下のモジュールを直接インポートしてください：
 * - rectangleEditCore.js: 編集操作、マウスイベント、当たり判定
 * - rectangleMeasure.js: 測量モード専用のUI・ロジック
 *
 * @requires ./rectangleEditCore.js - 編集操作コア機能
 * @requires ./rectangleMeasure.js - 測量機能
 *
 * @exports canvasToImagePixel - キャンバス座標を画像ピクセル座標に変換
 * @exports hitTestRectangle - 四角形との当たり判定
 * @exports hitTestHandle - ハンドルとの当たり判定
 * @exports handleRectangleMouseDown - マウスダウンイベント処理
 * @exports handleRectangleMouseMove - マウスムーブイベント処理
 * @exports handleRectangleMouseUp - マウスアップイベント処理
 * @exports calculateRotationAngle - 回転角度を計算
 * @exports snapRotationAngle - 回転角度をスナップ
 */

// ================
// 再エクスポート
// ================

// rectangleEditCore.js から再エクスポート
export {
    canvasToImagePixel,
    hitTestRectangle,
    hitTestHandle,
    handleRectangleMouseDown,
    handleRectangleMouseMove,
    handleRectangleMouseUp,
    calculateRotationAngle,
    snapRotationAngle
} from './rectangleEditCore.js';

// rectangleMeasure.js から再エクスポート（必要に応じて）
export {
    getEdgeMidpoint,
    promptEdgeLength,
    showEdgeLengthInputUI
} from './rectangleMeasure.js';
