/**
 * @file overlayRenderer.js
 * @description メタデータオーバーレイの描画機能を提供するモジュール
 * @requires ../state/mapState.js
 * @exports drawMetadataOverlay
 * @exports drawMetadataOverlayOnContext
 * @exports drawGridOverlay
 * @exports drawGridOverlayOnContext
 * @exports drawOriginOverlay
 * @exports drawOriginOverlayOnContext
 * @exports drawScaleBar
 * @exports drawScaleBarOnContext
 */

import { mapState } from '../state/mapState.js';

/**
 * 実世界座標（メートル）をキャンバス座標に変換
 * @param {number} worldX - 実世界X座標
 * @param {number} worldY - 実世界Y座標
 * @param {number} drawX - 描画開始X座標
 * @param {number} drawY - 描画開始Y座標
 * @returns {{x: number, y: number}|null} キャンバス座標
 */
function worldToCanvas(worldX, worldY, drawX, drawY) {
    if (!mapState.metadata || !Array.isArray(mapState.metadata.origin)) {
        return null;
    }
    const resolution = mapState.metadata.resolution;
    if (!resolution) return null;

    const [originX, originY] = mapState.metadata.origin;
    const pixelX = (worldX - originX) / resolution;
    const pixelY = mapState.image.height - (worldY - originY) / resolution;

    if (!isFinite(pixelX) || !isFinite(pixelY)) {
        return null;
    }

    return {
        x: drawX + pixelX * mapState.scale,
        y: drawY + pixelY * mapState.scale
    };
}

/**
 * 適切な数値に丸める
 * @param {number} value - 数値
 * @returns {number} 丸められた数値
 */
function getNiceNumber(value) {
    if (!isFinite(value) || value === 0) return 1;
    const exponent = Math.floor(Math.log10(Math.abs(value)));
    const fraction = Math.abs(value) / Math.pow(10, exponent);
    let niceFraction;
    if (fraction < 1.5) {
        niceFraction = 1;
    } else if (fraction < 3) {
        niceFraction = 2;
    } else if (fraction < 7) {
        niceFraction = 5;
    } else {
        niceFraction = 10;
    }
    return Math.sign(value) * niceFraction * Math.pow(10, exponent);
}

/**
 * 距離をフォーマット
 * @param {number} meters - メートル単位の距離
 * @returns {string} フォーマットされた距離文字列
 */
function formatDistance(meters) {
    if (!isFinite(meters)) return '';
    const absMeters = Math.abs(meters);
    if (absMeters >= 1000) {
        return `${(meters / 1000).toFixed(2)} km`;
    }
    if (absMeters >= 1) {
        const precision = absMeters >= 10 ? 0 : 2;
        return `${meters.toFixed(precision)} m`;
    }
    const centimeters = meters * 100;
    if (Math.abs(centimeters) >= 1) {
        const precision = Math.abs(centimeters) >= 10 ? 0 : 1;
        return `${centimeters.toFixed(precision)} cm`;
    }
    const millimeters = meters * 1000;
    return `${millimeters.toFixed(0)} mm`;
}

/**
 * メタデータオーバーレイを描画（デフォルトのctxを使用）
 * @export
 * @param {number} drawX - 描画開始X座標
 * @param {number} drawY - 描画開始Y座標
 * @param {number} scaledWidth - スケール後の幅
 * @param {number} scaledHeight - スケール後の高さ
 */
export function drawMetadataOverlay(drawX, drawY, scaledWidth, scaledHeight) {
    const metadata = mapState.metadata;
    if (!metadata) return;

    const resolution = metadata.resolution;

    if (mapState.overlaySettings.showGrid && resolution) {
        drawGridOverlay(drawX, drawY, scaledWidth, scaledHeight, resolution);
    }

    if (mapState.overlaySettings.showOrigin && Array.isArray(metadata.origin)) {
        drawOriginOverlay(drawX, drawY, scaledWidth, scaledHeight, metadata.origin);
    }

    if (mapState.overlaySettings.showScaleBar && resolution) {
        drawScaleBar(resolution);
    }
}

/**
 * メタデータオーバーレイを指定されたコンテキストに描画
 * @export
 * @param {CanvasRenderingContext2D} targetCtx - 描画先のコンテキスト
 * @param {number} drawX - 描画開始X座標
 * @param {number} drawY - 描画開始Y座標
 * @param {number} scaledWidth - スケール後の幅
 * @param {number} scaledHeight - スケール後の高さ
 */
export function drawMetadataOverlayOnContext(targetCtx, drawX, drawY, scaledWidth, scaledHeight) {
    const metadata = mapState.metadata;
    if (!metadata) return;

    const resolution = metadata.resolution;

    if (mapState.overlaySettings.showGrid && resolution) {
        drawGridOverlayOnContext(targetCtx, drawX, drawY, scaledWidth, scaledHeight, resolution);
    }

    if (mapState.overlaySettings.showOrigin && Array.isArray(metadata.origin)) {
        drawOriginOverlayOnContext(targetCtx, drawX, drawY, scaledWidth, scaledHeight, metadata.origin);
    }

    if (mapState.overlaySettings.showScaleBar && resolution) {
        drawScaleBarOnContext(targetCtx, resolution);
    }
}

/**
 * グリッドオーバーレイを描画（デフォルトのctxを使用）
 * @export
 * @param {number} drawX - 描画開始X座標
 * @param {number} drawY - 描画開始Y座標
 * @param {number} scaledWidth - スケール後の幅
 * @param {number} scaledHeight - スケール後の高さ
 * @param {number} resolution - 解像度（m/pixel）
 */
export function drawGridOverlay(drawX, drawY, scaledWidth, scaledHeight, resolution) {
    if (!resolution) return;

    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');

    const spacingMeters = mapState.gridWidthInMeters || 1;
    if (spacingMeters <= 0) return;

    const spacingPixels = spacingMeters / resolution;
    const scaledSpacing = spacingPixels * mapState.scale;
    if (!isFinite(scaledSpacing) || scaledSpacing < 4) {
        return;
    }

    ctx.save();
    ctx.strokeStyle = 'rgba(102, 126, 234, 0.35)';
    ctx.lineWidth = 1;

    const imageWidth = mapState.image.width;
    const imageHeight = mapState.image.height;

    for (let px = 0; px <= imageWidth; px += spacingPixels) {
        const canvasX = drawX + px * mapState.scale;
        if (canvasX < drawX - 1 || canvasX > drawX + scaledWidth + 1) continue;
        ctx.beginPath();
        ctx.moveTo(canvasX, drawY);
        ctx.lineTo(canvasX, drawY + scaledHeight);
        ctx.stroke();
    }

    for (let py = 0; py <= imageHeight; py += spacingPixels) {
        const canvasY = drawY + py * mapState.scale;
        if (canvasY < drawY - 1 || canvasY > drawY + scaledHeight + 1) continue;
        ctx.beginPath();
        ctx.moveTo(drawX, canvasY);
        ctx.lineTo(drawX + scaledWidth, canvasY);
        ctx.stroke();
    }

    ctx.restore();
}

/**
 * グリッドオーバーレイを指定されたコンテキストに描画
 * @export
 * @param {CanvasRenderingContext2D} targetCtx - 描画先のコンテキスト
 * @param {number} drawX - 描画開始X座標
 * @param {number} drawY - 描画開始Y座標
 * @param {number} scaledWidth - スケール後の幅
 * @param {number} scaledHeight - スケール後の高さ
 * @param {number} resolution - 解像度（m/pixel）
 */
export function drawGridOverlayOnContext(targetCtx, drawX, drawY, scaledWidth, scaledHeight, resolution) {
    if (!resolution) return;

    const spacingMeters = mapState.gridWidthInMeters || 1;
    if (spacingMeters <= 0) return;

    const spacingPixels = spacingMeters / resolution;
    const scaledSpacing = spacingPixels * mapState.scale;
    if (!isFinite(scaledSpacing) || scaledSpacing < 4) {
        return;
    }

    targetCtx.save();
    targetCtx.strokeStyle = 'rgba(102, 126, 234, 0.35)';
    targetCtx.lineWidth = 1;

    const imageWidth = mapState.image.width;
    const imageHeight = mapState.image.height;

    for (let px = 0; px <= imageWidth; px += spacingPixels) {
        const canvasX = drawX + px * mapState.scale;
        if (canvasX < drawX - 1 || canvasX > drawX + scaledWidth + 1) continue;
        targetCtx.beginPath();
        targetCtx.moveTo(canvasX, drawY);
        targetCtx.lineTo(canvasX, drawY + scaledHeight);
        targetCtx.stroke();
    }

    for (let py = 0; py <= imageHeight; py += spacingPixels) {
        const canvasY = drawY + py * mapState.scale;
        if (canvasY < drawY - 1 || canvasY > drawY + scaledHeight + 1) continue;
        targetCtx.beginPath();
        targetCtx.moveTo(drawX, canvasY);
        targetCtx.lineTo(drawX + scaledWidth, canvasY);
        targetCtx.stroke();
    }

    targetCtx.restore();
}

/**
 * 原点オーバーレイを描画（デフォルトのctxを使用）
 * @export
 * @param {number} drawX - 描画開始X座標
 * @param {number} drawY - 描画開始Y座標
 * @param {number} scaledWidth - スケール後の幅
 * @param {number} scaledHeight - スケール後の高さ
 * @param {Array<number>} origin - 原点座標 [x, y, theta]
 */
export function drawOriginOverlay(drawX, drawY, scaledWidth, scaledHeight, origin) {
    if (!Array.isArray(origin) || origin.length < 2) return;
    if (!mapState.metadata || mapState.metadata.resolution === undefined) return;

    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');

    const originCanvas = worldToCanvas(0, 0, drawX, drawY);
    if (!originCanvas) return;

    // 原点マーカーの描画要素（十字、矢印、ラベル）のサイズを考慮したマージン
    const margin = 100;

    // キャンバスの表示領域内にあるかチェック（マージン付き）
    if (
        originCanvas.x < -margin ||
        originCanvas.x > canvas.width + margin ||
        originCanvas.y < -margin ||
        originCanvas.y > canvas.height + margin
    ) {
        // 表示領域外の場合は描画しない
        return;
    }

    ctx.save();
    ctx.strokeStyle = 'rgba(231, 76, 60, 0.9)';
    ctx.lineWidth = 2;

    const crossSize = 10;
    ctx.beginPath();
    ctx.moveTo(originCanvas.x - crossSize, originCanvas.y);
    ctx.lineTo(originCanvas.x + crossSize, originCanvas.y);
    ctx.moveTo(originCanvas.x, originCanvas.y - crossSize);
    ctx.lineTo(originCanvas.x, originCanvas.y + crossSize);
    ctx.stroke();

    const theta = origin.length >= 3 ? origin[2] : 0;
    const arrowLength = 60;
    const dirX = Math.cos(theta);
    const dirY = -Math.sin(theta);
    const endX = originCanvas.x + dirX * arrowLength;
    const endY = originCanvas.y + dirY * arrowLength;

    ctx.beginPath();
    ctx.moveTo(originCanvas.x, originCanvas.y);
    ctx.lineTo(endX, endY);
    ctx.stroke();

    const headLength = 12;
    const leftDirX = Math.cos(theta + Math.PI / 6);
    const leftDirY = -Math.sin(theta + Math.PI / 6);
    const rightDirX = Math.cos(theta - Math.PI / 6);
    const rightDirY = -Math.sin(theta - Math.PI / 6);

    ctx.beginPath();
    ctx.moveTo(endX, endY);
    ctx.lineTo(endX - leftDirX * headLength, endY - leftDirY * headLength);
    ctx.moveTo(endX, endY);
    ctx.lineTo(endX - rightDirX * headLength, endY - rightDirY * headLength);
    ctx.stroke();

    const label = '原点 (0,0)';
    ctx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", "Roboto", sans-serif';
    const labelPadding = 4;
    const textWidth = ctx.measureText(label).width;
    const labelX = originCanvas.x + 12;
    const labelY = originCanvas.y + 12;

    ctx.fillStyle = 'rgba(231, 76, 60, 0.85)';
    ctx.fillRect(
        labelX - labelPadding,
        labelY - labelPadding,
        textWidth + labelPadding * 2,
        18
    );

    ctx.fillStyle = '#ffffff';
    ctx.textBaseline = 'top';
    ctx.fillText(label, labelX, labelY - 1);
    ctx.restore();
}

/**
 * 原点オーバーレイを指定されたコンテキストに描画
 * @export
 * @param {CanvasRenderingContext2D} targetCtx - 描画先のコンテキスト
 * @param {number} drawX - 描画開始X座標
 * @param {number} drawY - 描画開始Y座標
 * @param {number} scaledWidth - スケール後の幅
 * @param {number} scaledHeight - スケール後の高さ
 * @param {Array<number>} origin - 原点座標 [x, y, theta]
 */
export function drawOriginOverlayOnContext(targetCtx, drawX, drawY, scaledWidth, scaledHeight, origin) {
    if (!Array.isArray(origin) || origin.length < 2) return;
    if (!mapState.metadata || mapState.metadata.resolution === undefined) return;

    const originCanvas = worldToCanvas(0, 0, drawX, drawY);
    if (!originCanvas) return;

    // 原点マーカーの描画要素（十字、矢印、ラベル）のサイズを考慮したマージン
    const margin = 100;

    // レイヤーシステムのキャンバスサイズを取得
    const canvasWidth = mapState.layerStack.length > 0 ? mapState.layerStack[0].canvas.width :
                        document.getElementById('mapCanvas').width;
    const canvasHeight = mapState.layerStack.length > 0 ? mapState.layerStack[0].canvas.height :
                         document.getElementById('mapCanvas').height;

    // キャンバスの表示領域内にあるかチェック（マージン付き）
    if (
        originCanvas.x < -margin ||
        originCanvas.x > canvasWidth + margin ||
        originCanvas.y < -margin ||
        originCanvas.y > canvasHeight + margin
    ) {
        // 表示領域外の場合は描画しない
        return;
    }

    targetCtx.save();
    targetCtx.strokeStyle = 'rgba(231, 76, 60, 0.9)';
    targetCtx.lineWidth = 2;

    const crossSize = 10;
    targetCtx.beginPath();
    targetCtx.moveTo(originCanvas.x - crossSize, originCanvas.y);
    targetCtx.lineTo(originCanvas.x + crossSize, originCanvas.y);
    targetCtx.moveTo(originCanvas.x, originCanvas.y - crossSize);
    targetCtx.lineTo(originCanvas.x, originCanvas.y + crossSize);
    targetCtx.stroke();

    const theta = origin.length >= 3 ? origin[2] : 0;
    const arrowLength = 60;
    const dirX = Math.cos(theta);
    const dirY = -Math.sin(theta);
    const endX = originCanvas.x + dirX * arrowLength;
    const endY = originCanvas.y + dirY * arrowLength;

    targetCtx.beginPath();
    targetCtx.moveTo(originCanvas.x, originCanvas.y);
    targetCtx.lineTo(endX, endY);
    targetCtx.stroke();

    const headLength = 12;
    const leftDirX = Math.cos(theta + Math.PI / 6);
    const leftDirY = -Math.sin(theta + Math.PI / 6);
    const rightDirX = Math.cos(theta - Math.PI / 6);
    const rightDirY = -Math.sin(theta - Math.PI / 6);

    targetCtx.beginPath();
    targetCtx.moveTo(endX, endY);
    targetCtx.lineTo(endX - leftDirX * headLength, endY - leftDirY * headLength);
    targetCtx.moveTo(endX, endY);
    targetCtx.lineTo(endX - rightDirX * headLength, endY - rightDirY * headLength);
    targetCtx.stroke();

    const label = '原点 (0,0)';
    targetCtx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", "Roboto", sans-serif';
    const labelPadding = 4;
    const textWidth = targetCtx.measureText(label).width;
    const labelX = originCanvas.x + 12;
    const labelY = originCanvas.y + 12;

    targetCtx.fillStyle = 'rgba(231, 76, 60, 0.85)';
    targetCtx.fillRect(
        labelX - labelPadding,
        labelY - labelPadding,
        textWidth + labelPadding * 2,
        18
    );

    targetCtx.fillStyle = '#ffffff';
    targetCtx.textBaseline = 'top';
    targetCtx.fillText(label, labelX, labelY - 1);
    targetCtx.restore();
}

/**
 * スケールバーを描画（デフォルトのctxを使用）
 * @export
 * @param {number} resolution - 解像度（m/pixel）
 */
export function drawScaleBar(resolution) {
    if (!resolution) return;

    const canvas = document.getElementById('mapCanvas');
    const ctx = canvas.getContext('2d');

    const pixelsPerMeter = (1 / resolution) * mapState.scale;
    if (!isFinite(pixelsPerMeter) || pixelsPerMeter <= 0) return;

    const desiredPx = 160;
    let meters = getNiceNumber(desiredPx / pixelsPerMeter);
    let barPx = meters * pixelsPerMeter;

    if (barPx < 60) {
        while (barPx < 60) {
            meters *= 2;
            barPx = meters * pixelsPerMeter;
            if (meters > 1e6) break;
        }
    } else if (barPx > 220) {
        while (barPx > 220) {
            meters /= 2;
            barPx = meters * pixelsPerMeter;
            if (meters < 1e-6) break;
        }
    }

    if (!isFinite(barPx) || barPx <= 0) return;

    const margin = 20;
    const barHeight = 12;
    const x = margin;
    const y = canvas.height - margin;

    ctx.save();
    ctx.fillStyle = 'rgba(0, 0, 0, 0.65)';
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.rect(x, y - barHeight, barPx, barHeight);
    ctx.fill();
    ctx.stroke();

    ctx.fillStyle = '#ffffff';
    ctx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", "Roboto", sans-serif';
    ctx.textAlign = 'center';
    ctx.textBaseline = 'bottom';
    ctx.fillText(formatDistance(meters), x + barPx / 2, y - barHeight - 6);
    ctx.restore();
}

/**
 * スケールバーを指定されたコンテキストに描画
 * @export
 * @param {CanvasRenderingContext2D} targetCtx - 描画先のコンテキスト
 * @param {number} resolution - 解像度（m/pixel）
 */
export function drawScaleBarOnContext(targetCtx, resolution) {
    if (!resolution) return;

    const pixelsPerMeter = (1 / resolution) * mapState.scale;
    if (!isFinite(pixelsPerMeter) || pixelsPerMeter <= 0) return;

    const desiredPx = 160;
    let meters = getNiceNumber(desiredPx / pixelsPerMeter);
    let barPx = meters * pixelsPerMeter;

    if (barPx < 60) {
        while (barPx < 60) {
            meters *= 2;
            barPx = meters * pixelsPerMeter;
            if (meters > 1e6) break;
        }
    } else if (barPx > 220) {
        while (barPx > 220) {
            meters /= 2;
            barPx = meters * pixelsPerMeter;
            if (meters < 1e-6) break;
        }
    }

    if (!isFinite(barPx) || barPx <= 0) return;

    const margin = 20;
    const barHeight = 12;
    const x = margin;
    // レイヤーシステムのキャンバス高さを取得
    const canvasHeight = mapState.layerStack.length > 0 ? mapState.layerStack[0].canvas.height :
                         document.getElementById('mapCanvas').height;
    const y = canvasHeight - margin;

    targetCtx.save();
    targetCtx.fillStyle = 'rgba(0, 0, 0, 0.65)';
    targetCtx.strokeStyle = 'rgba(255, 255, 255, 0.8)';
    targetCtx.lineWidth = 2;
    targetCtx.beginPath();
    targetCtx.rect(x, y - barHeight, barPx, barHeight);
    targetCtx.fill();
    targetCtx.stroke();

    targetCtx.fillStyle = '#ffffff';
    targetCtx.font = '12px -apple-system, BlinkMacSystemFont, "Segoe UI", "Roboto", sans-serif';
    targetCtx.textAlign = 'center';
    targetCtx.textBaseline = 'bottom';
    targetCtx.fillText(formatDistance(meters), x + barPx / 2, y - barHeight - 6);
    targetCtx.restore();
}
