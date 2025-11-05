/**
 * @file coordinates.test.js
 * @description coordinates.jsのユニットテスト
 */

import { describe, it, expect, beforeEach, vi } from 'vitest';
import { worldToCanvas, canvasToImagePixel, imagePixelToCanvas } from '@utils/coordinates.js';
import { mapState } from '@state/mapState.js';

describe('worldToCanvas', () => {
  beforeEach(() => {
    // mapStateをリセット
    mapState.metadata = {
      resolution: 0.05,
      origin: [-10.0, -10.0, 0.0]
    };
    mapState.mapImage = {
      height: 400
    };
    mapState.scale = 1.0;
  });

  it('メタデータがある場合、ワールド座標をキャンバス座標に変換する', () => {
    const result = worldToCanvas(0, 0, 100, 50);

    expect(result).not.toBeNull();
    expect(result).toHaveProperty('x');
    expect(result).toHaveProperty('y');
  });

  it('メタデータがnullの場合、nullを返す', () => {
    mapState.metadata = null;

    const result = worldToCanvas(0, 0, 100, 50);

    expect(result).toBeNull();
  });

  it('originが配列でない場合、nullを返す', () => {
    mapState.metadata.origin = 'invalid';

    const result = worldToCanvas(0, 0, 100, 50);

    expect(result).toBeNull();
  });

  it('resolutionがない場合、nullを返す', () => {
    mapState.metadata.resolution = null;

    const result = worldToCanvas(0, 0, 100, 50);

    expect(result).toBeNull();
  });

  it('原点(0,0)のワールド座標を正しく変換する', () => {
    // origin: [-10, -10], resolution: 0.05
    // (0 - (-10)) / 0.05 = 200 pixel
    const result = worldToCanvas(0, 0, 0, 0);

    expect(result).not.toBeNull();
    // pixelX = (0 - (-10)) / 0.05 = 200
    // pixelY = (0 - (-10)) / 0.05 = 200
    // Y軸は反転されるので、canvasY = 0 + (400 - 200) * 1.0 = 200
    expect(result.x).toBe(200);
  });
});

describe('canvasToImagePixel', () => {
  beforeEach(() => {
    mapState.scale = 1.0;
    mapState.offsetX = 0;
    mapState.offsetY = 0;
  });

  it('キャンバス座標を画像ピクセル座標に変換する', () => {
    const container = { offsetLeft: 0, offsetTop: 0 };
    const event = { clientX: 100, clientY: 150 };

    const result = canvasToImagePixel(event.clientX, event.clientY, container);

    expect(result).not.toBeNull();
    expect(result).toHaveProperty('x');
    expect(result).toHaveProperty('y');
  });

  it('スケール2.0の場合、正しく変換する', () => {
    mapState.scale = 2.0;
    const container = { offsetLeft: 0, offsetTop: 0 };

    const result = canvasToImagePixel(100, 100, container);

    expect(result).not.toBeNull();
    // (100 - 0) / 2.0 = 50
    expect(result.x).toBe(50);
    expect(result.y).toBe(50);
  });

  it('オフセットがある場合、正しく変換する', () => {
    mapState.offsetX = 50;
    mapState.offsetY = 30;
    const container = { offsetLeft: 0, offsetTop: 0 };

    const result = canvasToImagePixel(100, 100, container);

    expect(result).not.toBeNull();
    // (100 - 0 - 50) / 1.0 = 50
    expect(result.x).toBe(50);
    // (100 - 0 - 30) / 1.0 = 70
    expect(result.y).toBe(70);
  });
});

describe('imagePixelToCanvas', () => {
  beforeEach(() => {
    mapState.scale = 1.0;
    mapState.offsetX = 0;
    mapState.offsetY = 0;
  });

  it('画像ピクセル座標をキャンバス座標に変換する', () => {
    const container = { offsetLeft: 0, offsetTop: 0 };

    const result = imagePixelToCanvas(100, 150, container);

    expect(result).not.toBeNull();
    expect(result).toHaveProperty('x');
    expect(result).toHaveProperty('y');
  });

  it('スケール2.0の場合、正しく変換する', () => {
    mapState.scale = 2.0;
    const container = { offsetLeft: 0, offsetTop: 0 };

    const result = imagePixelToCanvas(100, 100, container);

    expect(result).not.toBeNull();
    // 100 * 2.0 + 0 + 0 = 200
    expect(result.x).toBe(200);
    expect(result.y).toBe(200);
  });

  it('オフセットがある場合、正しく変換する', () => {
    mapState.offsetX = 50;
    mapState.offsetY = 30;
    const container = { offsetLeft: 10, offsetTop: 20 };

    const result = imagePixelToCanvas(100, 100, container);

    expect(result).not.toBeNull();
    // 100 * 1.0 + 50 + 10 = 160
    expect(result.x).toBe(160);
    // 100 * 1.0 + 30 + 20 = 150
    expect(result.y).toBe(150);
  });

  it('canvasToImagePixelの逆変換として機能する', () => {
    const container = { offsetLeft: 0, offsetTop: 0 };
    mapState.scale = 1.5;
    mapState.offsetX = 20;
    mapState.offsetY = 30;

    // キャンバス座標
    const canvasX = 200;
    const canvasY = 250;

    // キャンバス→画像ピクセル
    const imagePx = canvasToImagePixel(canvasX, canvasY, container);

    // 画像ピクセル→キャンバス（元に戻るはず）
    const backToCanvas = imagePixelToCanvas(imagePx.x, imagePx.y, container);

    expect(backToCanvas.x).toBeCloseTo(canvasX, 5);
    expect(backToCanvas.y).toBeCloseTo(canvasY, 5);
  });
});
