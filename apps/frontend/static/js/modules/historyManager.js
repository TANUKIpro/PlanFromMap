/**
 * @file historyManager.js
 * @description アンドゥ/リドゥ履歴管理システム
 *
 * レイヤーの状態変更を記録し、アンドゥ/リドゥ機能を提供します。
 * 状態のキャプチャ、復元、履歴スタック管理を行います。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 *
 * @exports saveToHistory - 現在の状態を履歴に保存
 * @exports captureState - 現在の状態をキャプチャ
 * @exports restoreState - 状態を復元
 * @exports undo - アンドゥ（元に戻す）
 * @exports redo - リドゥ（やり直す）
 * @exports updateUndoRedoButtons - アンドゥ/リドゥボタンのUI更新
 */

import { mapState } from '../state/mapState.js';

/**
 * 現在の状態を履歴に保存する
 * 最大履歴数を超えた場合は古い履歴を削除します
 *
 * @returns {void}
 *
 * @example
 * saveToHistory();
 */
export function saveToHistory() {
    // 現在の状態をキャプチャ
    const state = captureState();

    mapState.history.past.push(state);
    mapState.history.future = [];  // 新しいアクションで未来の履歴をクリア

    // 最大履歴数を超えたら古いものを削除
    if (mapState.history.past.length > mapState.history.maxHistory) {
        mapState.history.past.shift();
    }

    updateUndoRedoButtons();
}

/**
 * 現在の状態をキャプチャする
 * レイヤーの画像データ、ストローク、プロパティを保存します
 *
 * @returns {Object} キャプチャされた状態オブジェクト
 *
 * @example
 * const state = captureState();
 */
export function captureState() {
    const state = {
        layers: mapState.layerStack.map(layer => ({
            id: layer.id,
            name: layer.name,
            type: layer.type,
            permanent: layer.permanent,
            visible: layer.visible,
            opacity: layer.opacity,
            imageData: layer.type === 'drawing' ? layer.ctx.getImageData(0, 0, layer.canvas.width, layer.canvas.height) : null,
            // ストロークデータも保存（ディープコピー）
            strokes: layer.type === 'drawing' && layer.strokes ? JSON.parse(JSON.stringify(layer.strokes)) : null
        }))
    };
    return state;
}

/**
 * 状態を復元する
 * 保存された状態からレイヤーの内容を復元します
 *
 * @param {Object} state - 復元する状態オブジェクト
 * @returns {void}
 *
 * @example
 * restoreState(previousState);
 */
export function restoreState(state) {
    // レイヤーを復元
    state.layers.forEach(savedLayer => {
        const layer = mapState.layerStack.find(l => l.id === savedLayer.id);
        if (layer) {
            layer.visible = savedLayer.visible;
            layer.opacity = savedLayer.opacity;
            layer.canvas.style.display = savedLayer.visible ? 'block' : 'none';
            layer.canvas.style.opacity = savedLayer.opacity;

            if (layer.type === 'drawing') {
                // ストロークデータを復元
                if (savedLayer.strokes) {
                    layer.strokes = JSON.parse(JSON.stringify(savedLayer.strokes));
                }
                // イメージデータも復元（バケツ塗りつぶしなどのピクセル操作のため）
                if (savedLayer.imageData) {
                    layer.ctx.putImageData(savedLayer.imageData, 0, 0);
                }
            }
        }
    });

    // レイヤーパネルを更新（外部関数を期待）
    if (typeof window.updateLayersPanel === 'function') {
        window.updateLayersPanel();
    }

    // すべてのレイヤーを再描画（外部関数を期待）
    if (typeof window.redrawAllLayers === 'function') {
        window.redrawAllLayers();
    }
}

/**
 * アンドゥ（元に戻す）
 * 履歴から1つ前の状態を復元します
 *
 * @returns {void}
 *
 * @example
 * undo();
 */
export function undo() {
    if (mapState.history.past.length === 0) return;

    const currentState = captureState();
    const previousState = mapState.history.past.pop();

    mapState.history.future.push(currentState);
    restoreState(previousState);

    updateUndoRedoButtons();
}

/**
 * リドゥ（やり直す）
 * アンドゥした操作をもう一度実行します
 *
 * @returns {void}
 *
 * @example
 * redo();
 */
export function redo() {
    if (mapState.history.future.length === 0) return;

    const currentState = captureState();
    const nextState = mapState.history.future.pop();

    mapState.history.past.push(currentState);
    restoreState(nextState);

    updateUndoRedoButtons();
}

/**
 * アンドゥ/リドゥボタンのUI状態を更新する
 * 履歴の有無に応じてボタンの有効/無効を切り替えます
 *
 * @returns {void}
 *
 * @example
 * updateUndoRedoButtons();
 */
export function updateUndoRedoButtons() {
    const undoButton = document.getElementById('undoButton');
    const redoButton = document.getElementById('redoButton');

    if (undoButton) {
        undoButton.disabled = mapState.history.past.length === 0;
    }
    if (redoButton) {
        redoButton.disabled = mapState.history.future.length === 0;
    }
}
