/**
 * @file controls.js
 * @description マップコントロール（読み込み、クリア、描画）機能を提供するモジュール
 * @requires ../state/mapState.js - マップの状態管理
 * @exports loadImageFile - 画像ファイル読み込みダイアログを開く
 * @exports loadYAMLFile - YAMLファイル読み込みダイアログを開く
 * @exports clearMap - マップをクリアする
 * @exports drawMap - マップを描画する
 */

import { mapState } from '../state/mapState.js';

/**
 * 画像ファイル読み込みダイアログを開く
 */
export function loadImageFile() {
    const imageInput = document.getElementById('imageFileInput');
    if (imageInput) {
        imageInput.click();
    }
}

/**
 * YAMLファイル読み込みダイアログを開く
 */
export function loadYAMLFile() {
    const yamlInput = document.getElementById('yamlFileInput');
    if (yamlInput) {
        yamlInput.click();
    }
}

/**
 * マップをクリアする
 */
export function clearMap() {
    const container = document.getElementById('mapContainer');
    const canvas = document.getElementById('mapCanvas');
    const canvasStack = document.getElementById('canvasStack');

    // マップ状態をリセット
    mapState.image = null;
    mapState.scale = 1.0;
    mapState.offsetX = 0;
    mapState.offsetY = 0;
    mapState.metadata = null;
    mapState.layers.image = true;
    mapState.layers.metadataOverlay = true;

    // レイヤースタックをクリア
    mapState.layerStack = [];
    if (canvasStack) {
        canvasStack.innerHTML = '';
        canvasStack.style.display = 'none';
    }

    // 履歴をクリア
    mapState.history.past = [];
    mapState.history.future = [];

    // アンドゥ/リドゥボタンの状態を更新
    if (window.updateUndoRedoButtons && typeof window.updateUndoRedoButtons === 'function') {
        window.updateUndoRedoButtons();
    }

    if (canvas) {
        canvas.style.display = 'none';
    }

    const mapPlaceholder = document.getElementById('mapPlaceholder');
    if (mapPlaceholder) {
        mapPlaceholder.style.display = 'block';
    }

    // メタデータ表示をリセット
    const metadataOverlay = document.getElementById('metadataOverlay');
    const metadataContent = document.getElementById('metadataOverlayContent');
    if (metadataContent) {
        metadataContent.innerHTML = '';
    }
    if (metadataOverlay) {
        metadataOverlay.classList.remove('active', 'minimized');
        metadataOverlay.setAttribute('aria-hidden', 'true');
    }

    // ファイル入力をリセット
    const imageInput = document.getElementById('imageFileInput');
    const yamlInput = document.getElementById('yamlFileInput');
    if (imageInput) {
        imageInput.value = '';
    }
    if (yamlInput) {
        yamlInput.value = '';
    }

    // 各種コントロールの状態を更新
    if (window.updateOverlayControls && typeof window.updateOverlayControls === 'function') {
        window.updateOverlayControls();
    }
    if (window.updateLayersPanel && typeof window.updateLayersPanel === 'function') {
        window.updateLayersPanel();
    }
    if (window.updateZoomInfo && typeof window.updateZoomInfo === 'function') {
        window.updateZoomInfo();
    }
    if (window.updateMetadataOverlayVisibility && typeof window.updateMetadataOverlayVisibility === 'function') {
        window.updateMetadataOverlayVisibility();
    }
}

/**
 * マップを描画する
 */
export function drawMap() {
    const canvas = document.getElementById('mapCanvas');
    const container = document.getElementById('mapContainer');

    if (!mapState.image || !canvas || !container) {
        return;
    }

    const ctx = canvas.getContext('2d');
    if (!ctx) {
        return;
    }

    // キャンバスをクリア
    ctx.clearRect(0, 0, canvas.width, canvas.height);

    // 背景色
    ctx.fillStyle = '#f5f5f5';
    ctx.fillRect(0, 0, canvas.width, canvas.height);

    // 画像のスケール後のサイズを計算
    const scaledWidth = mapState.image.width * mapState.scale;
    const scaledHeight = mapState.image.height * mapState.scale;

    // 中央配置の基準位置を計算
    const baseX = (canvas.width - scaledWidth) / 2;
    const baseY = (canvas.height - scaledHeight) / 2;

    // オフセットを適用して描画
    const drawX = baseX + mapState.offsetX;
    const drawY = baseY + mapState.offsetY;

    // 画像レイヤーを描画
    if (mapState.layers.image) {
        ctx.imageSmoothingEnabled = false;
        ctx.drawImage(mapState.image, drawX, drawY, scaledWidth, scaledHeight);
    }

    // メタデータオーバーレイを描画
    if (mapState.layers.metadataOverlay && mapState.metadata) {
        if (window.drawMetadataOverlay && typeof window.drawMetadataOverlay === 'function') {
            window.drawMetadataOverlay(drawX, drawY, scaledWidth, scaledHeight);
        }
    }

    // ズーム情報を更新
    if (window.updateZoomInfo && typeof window.updateZoomInfo === 'function') {
        window.updateZoomInfo();
    }
}
