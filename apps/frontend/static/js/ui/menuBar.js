/**
 * @file menuBar.js
 * @description メニューバーの制御
 *
 * @requires ../state/mapState.js
 *
 * @exports toggleMenu
 * @exports closeAllMenus
 * @exports updateGridWidth
 * @exports toggleSnapToGrid
 * @exports initMenuBar
 */

import { mapState } from '../state/mapState.js';
import { redrawAllLayers } from '../modules/layerManager.js';

/**
 * メニューを開閉する
 * @export
 * @param {string} menuId - メニューのID
 */
export function toggleMenu(menuId) {
    const menu = document.getElementById(menuId);
    if (!menu) return;

    // 他のメニューを閉じる
    const allMenus = document.querySelectorAll('.menu-bar__dropdown');
    allMenus.forEach(m => {
        if (m.id !== menuId) {
            m.classList.remove('active');
            // 対応するボタンのactiveクラスも削除
            const button = m.previousElementSibling;
            if (button) button.classList.remove('active');
        }
    });

    // メニューの表示切り替え
    menu.classList.toggle('active');

    // ボタンのactiveクラスも切り替え
    const button = menu.previousElementSibling;
    if (button) {
        button.classList.toggle('active');
    }
}

/**
 * すべてのメニューを閉じる
 * @export
 */
export function closeAllMenus() {
    const allMenus = document.querySelectorAll('.menu-bar__dropdown');
    allMenus.forEach(menu => {
        menu.classList.remove('active');
    });

    const allButtons = document.querySelectorAll('.menu-bar__button');
    allButtons.forEach(button => {
        button.classList.remove('active');
    });
}

/**
 * グリッド幅を更新
 * @export
 * @param {number} widthInCm - グリッド幅（cm単位）
 */
export function updateGridWidth(widthInCm) {
    const width = parseFloat(widthInCm);
    if (isNaN(width) || width <= 0) {
        console.warn('無効なグリッド幅:', widthInCm);
        return;
    }

    // cmをメートルに変換
    mapState.gridWidthInMeters = width / 100;

    // レイヤーを再描画
    redrawAllLayers();
}

/**
 * スナップ機能の有効/無効を切り替え
 * @export
 * @param {boolean} enabled - スナップを有効にするか
 */
export function toggleSnapToGrid(enabled) {
    mapState.snapToGrid = enabled;

    // ラベルを更新
    const label = document.getElementById('snapToGridLabel');
    if (label) {
        label.textContent = enabled ? 'スナップ ✓' : 'スナップ';
    }
}

/**
 * 座標をグリッドにスナップ
 * @export
 * @param {number} x - X座標（画像ピクセル）
 * @param {number} y - Y座標（画像ピクセル）
 * @returns {{x: number, y: number}} スナップされた座標
 */
export function snapToGrid(x, y) {
    if (!mapState.snapToGrid || !mapState.metadata || !mapState.metadata.resolution) {
        return { x, y };
    }

    const resolution = mapState.metadata.resolution; // m/pixel
    const gridWidth = mapState.gridWidthInMeters || 1.0; // デフォルト1m

    // 画像ピクセル座標を実世界座標に変換
    const worldX = x * resolution;
    const worldY = y * resolution;

    // グリッドにスナップ
    const snappedWorldX = Math.round(worldX / gridWidth) * gridWidth;
    const snappedWorldY = Math.round(worldY / gridWidth) * gridWidth;

    // 実世界座標を画像ピクセル座標に戻す
    const snappedX = snappedWorldX / resolution;
    const snappedY = snappedWorldY / resolution;

    return { x: snappedX, y: snappedY };
}

/**
 * メニューバーの初期化
 * @export
 */
export function initMenuBar() {
    // グリッド幅のデフォルト値を設定
    mapState.gridWidthInMeters = 0.1; // 0.1m = 10cm
    mapState.snapToGrid = false;

    // メニュー外をクリックしたら閉じる
    document.addEventListener('click', (event) => {
        const isMenuButton = event.target.closest('.menu-bar__button');
        const isMenuDropdown = event.target.closest('.menu-bar__dropdown');

        if (!isMenuButton && !isMenuDropdown) {
            closeAllMenus();
        }
    });

    // ドロップダウン内の入力要素のクリックでメニューが閉じないようにする
    const dropdownInputs = document.querySelectorAll('.menu-bar__dropdown input');
    dropdownInputs.forEach(input => {
        input.addEventListener('click', (event) => {
            event.stopPropagation();
        });
    });
}
