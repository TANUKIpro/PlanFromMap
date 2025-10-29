/**
 * @file controls.js
 * @description マップコントロール（読み込み、クリア、描画）機能を提供するモジュール
 * @requires ../state/mapState.js - マップの状態管理
 * @requires ../modules/profileManager.js - プロファイル管理
 * @exports loadImageFile - 画像ファイル読み込みダイアログを開く
 * @exports loadYAMLFile - YAMLファイル読み込みダイアログを開く
 * @exports clearMap - マップをクリアする
 * @exports drawMap - マップを描画する
 * @exports showProfileManager - プロファイル管理モーダルを表示
 * @exports closeProfileManager - プロファイル管理モーダルを閉じる
 * @exports saveCurrentProfile - 現在の状態をプロファイルとして保存
 * @exports loadSelectedProfile - プロファイルをロード
 * @exports deleteSelectedProfile - プロファイルを削除
 * @exports toggleDrawingToolsExpand - 描画ツールの展開/折りたたみ（廃止）
 * @exports toggleDrawingToolsVisibility - 描画ツールの表示/非表示切り替え
 * @exports toggleLayersPanelExpand - レイヤーパネルの展開/折りたたみ（廃止）
 * @exports toggleLayersPanelVisibility - レイヤーパネルの表示/非表示切り替え
 */

import { mapState } from '../state/mapState.js';
import {
    saveProfile,
    loadProfile,
    deleteProfile,
    listProfiles,
    getProfileInfo,
    exportProfileToFile
} from '../modules/profileManager.js';

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

/**
 * プロファイル管理モーダルを表示
 */
export function showProfileManager() {
    const modal = document.getElementById('profileManagerModal');
    if (modal) {
        modal.style.display = 'flex';
        updateProfileList();
    }
}

/**
 * プロファイル管理モーダルを閉じる
 */
export function closeProfileManager() {
    const modal = document.getElementById('profileManagerModal');
    if (modal) {
        modal.style.display = 'none';
    }
}

/**
 * 現在の状態をプロファイルとして保存
 */
export function saveCurrentProfile() {
    const input = document.getElementById('profileNameInput');
    if (!input) return;

    const profileName = input.value.trim();
    if (!profileName) {
        alert('プロファイル名を入力してください');
        return;
    }

    const success = saveProfile(profileName);
    if (success) {
        alert('プロファイルを保存しました');
        input.value = '';
        updateProfileList();
    } else {
        alert('プロファイルの保存に失敗しました');
    }
}

/**
 * プロファイル一覧を更新
 */
function updateProfileList() {
    const profileList = document.getElementById('profileList');
    if (!profileList) return;

    const profiles = listProfiles();

    if (profiles.length === 0) {
        profileList.innerHTML = '<p style="color: #999; text-align: center;">保存されたプロファイルがありません</p>';
        return;
    }

    profileList.innerHTML = profiles.map(profileName => {
        const info = getProfileInfo(profileName);
        const date = info && info.timestamp ? new Date(info.timestamp).toLocaleString('ja-JP') : '不明';

        return `
            <div class="profile-item">
                <div class="profile-item-info">
                    <div class="profile-item-name">${profileName}</div>
                    <div class="profile-item-meta">保存日時: ${date}</div>
                </div>
                <div class="profile-item-actions">
                    <button class="profile-button load" onclick="loadSelectedProfile('${profileName}')">読み込み</button>
                    <button class="profile-button export" onclick="exportSelectedProfile('${profileName}')">エクスポート</button>
                    <button class="profile-button delete" onclick="deleteSelectedProfile('${profileName}')">削除</button>
                </div>
            </div>
        `;
    }).join('');
}

/**
 * プロファイルをロード
 */
export async function loadSelectedProfile(profileName) {
    const success = await loadProfile(profileName);
    if (success) {
        // レイヤーを再構築
        if (window.initializeLayers && typeof window.initializeLayers === 'function') {
            window.initializeLayers();
        }

        // すべてのレイヤーを再描画
        if (window.redrawAllLayers && typeof window.redrawAllLayers === 'function') {
            window.redrawAllLayers();
        }

        // メタデータ表示を更新
        if (window.displayMetadata && typeof window.displayMetadata === 'function') {
            window.displayMetadata(mapState.metadata);
        }

        // UIを更新
        if (window.updateOverlayControls && typeof window.updateOverlayControls === 'function') {
            window.updateOverlayControls();
        }
        if (window.updateZoomInfo && typeof window.updateZoomInfo === 'function') {
            window.updateZoomInfo();
        }

        // マップコンテナを表示
        const mapPlaceholder = document.getElementById('mapPlaceholder');
        const canvasStack = document.getElementById('canvasStack');
        if (mapPlaceholder) {
            mapPlaceholder.style.display = 'none';
        }
        if (canvasStack) {
            canvasStack.style.display = 'block';
        }

        alert('プロファイルを読み込みました');
        closeProfileManager();
    } else {
        alert('プロファイルの読み込みに失敗しました');
    }
}

/**
 * プロファイルを削除
 */
export function deleteSelectedProfile(profileName) {
    if (!confirm(`プロファイル「${profileName}」を削除しますか?`)) {
        return;
    }

    const success = deleteProfile(profileName);
    if (success) {
        alert('プロファイルを削除しました');
        updateProfileList();
    } else {
        alert('プロファイルの削除に失敗しました');
    }
}

/**
 * プロファイルをエクスポート
 */
export function exportSelectedProfile(profileName) {
    exportProfileToFile(profileName);
}

/**
 * 描画ツールの展開/折りたたみ（廃止）
 * @deprecated 新しいtoggleDrawingToolsVisibilityを使用してください
 */
export function toggleDrawingToolsExpand() {
    const palette = document.getElementById('drawingToolsPalette');
    if (palette) {
        palette.classList.toggle('collapsed');
    }
}

/**
 * 描画ツールの表示/非表示切り替え
 */
export function toggleDrawingToolsVisibility() {
    const palette = document.getElementById('drawingToolsPalette');
    const toggleButton = document.getElementById('drawingToolsToggle');

    if (palette && toggleButton) {
        palette.classList.toggle('hidden');

        // ボタンのアイコンを切り替え
        if (palette.classList.contains('hidden')) {
            toggleButton.textContent = '▼';
            toggleButton.setAttribute('title', '描画ツールを表示');
        } else {
            toggleButton.textContent = '▲';
            toggleButton.setAttribute('title', '描画ツールを非表示');
        }
    }
}

/**
 * レイヤーパネルの展開/折りたたみ（廃止）
 * @deprecated 新しいtoggleLayersPanelVisibilityを使用してください
 */
export function toggleLayersPanelExpand(event) {
    // イベントがボタンから来た場合は何もしない
    if (event && event.target.classList.contains('layer-add-button')) {
        return;
    }

    const panel = document.getElementById('layersPanel');
    if (panel) {
        panel.classList.toggle('collapsed');
    }
}

/**
 * レイヤーパネルの表示/非表示切り替え
 */
export function toggleLayersPanelVisibility() {
    const panel = document.getElementById('layersPanel');
    const toggleButton = document.getElementById('layersPanelToggle');

    if (panel && toggleButton) {
        panel.classList.toggle('hidden');

        // ボタンのアイコンを切り替え
        if (panel.classList.contains('hidden')) {
            toggleButton.textContent = '→';
            toggleButton.setAttribute('title', 'レイヤーを表示');
        } else {
            toggleButton.textContent = '←';
            toggleButton.setAttribute('title', 'レイヤーを非表示');
        }
    }
}
