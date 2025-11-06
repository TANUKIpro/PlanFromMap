/**
 * @file profileManager.js
 * @description プロファイル管理機能
 *
 * マップデータ、メタデータ、レイヤー情報、描画情報などを
 * プロファイルとして保存・読み込む機能を提供します。
 * プロファイルはLocalStorageに保存されます。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../ui/toast.js - トーストメッセージ表示
 *
 * @exports saveProfile - プロファイルの保存
 * @exports loadProfile - プロファイルの読み込み
 * @exports deleteProfile - プロファイルの削除
 * @exports listProfiles - プロファイル一覧の取得
 * @exports getLastProfile - 最後に使用したプロファイルの取得
 * @exports setLastProfile - 最後に使用したプロファイルの設定
 * @exports exportProfileToFile - プロファイルをファイルにエクスポート
 * @exports importProfileFromFile - プロファイルをファイルからインポート
 */

import { mapState } from '../state/mapState.js';
import { showError } from '../ui/toast.js';

// ================
// 定数
// ================

/** プロファイルのLocalStorageキー接頭辞 */
const PROFILE_PREFIX = 'map_profile_';

/** プロファイル一覧のLocalStorageキー */
const PROFILES_LIST_KEY = 'map_profiles_list';

/** 最後に使用したプロファイルのキー */
const LAST_PROFILE_KEY = 'map_last_profile';

/** プロファイルの最大サイズ（バイト） */
const MAX_PROFILE_SIZE = 5 * 1024 * 1024; // 5MB

/** 現在のプロファイルのキー */
const CURRENT_PROFILE_KEY = 'map_current_profile';

// ================
// 状態管理
// ================

/** 現在参照しているプロファイル名 */
let currentProfileName = null;

// ================
// ユーティリティ関数
// ================

/**
 * 画像をBase64エンコードする
 *
 * @private
 * @param {HTMLImageElement} image - エンコードする画像
 * @returns {string|null} Base64エンコードされた画像データ
 */
function imageToBase64(image) {
    if (!image) return null;

    try {
        const canvas = document.createElement('canvas');
        canvas.width = image.width;
        canvas.height = image.height;
        const ctx = canvas.getContext('2d');
        ctx.drawImage(image, 0, 0);
        return canvas.toDataURL('image/png');
    } catch (error) {
        console.error('imageToBase64: エンコードエラー', error);
        return null;
    }
}

/**
 * Base64データから画像を復元する
 *
 * @private
 * @param {string} base64Data - Base64エンコードされた画像データ
 * @returns {Promise<HTMLImageElement>} 復元された画像
 */
function base64ToImage(base64Data) {
    return new Promise((resolve, reject) => {
        if (!base64Data) {
            reject(new Error('base64Data is null or empty'));
            return;
        }

        const img = new Image();
        img.onload = () => resolve(img);
        img.onerror = () => reject(new Error('Failed to load image from base64'));
        img.src = base64Data;
    });
}

/**
 * キャンバスをBase64エンコードする
 *
 * @private
 * @param {HTMLCanvasElement} canvas - エンコードするキャンバス
 * @returns {string|null} Base64エンコードされたキャンバスデータ
 */
function canvasToBase64(canvas) {
    if (!canvas) return null;

    try {
        return canvas.toDataURL('image/png');
    } catch (error) {
        console.error('canvasToBase64: エンコードエラー', error);
        return null;
    }
}

/**
 * レイヤースタックをシリアライズする
 *
 * @private
 * @param {Array} layerStack - レイヤースタック
 * @returns {Array} シリアライズされたレイヤースタック
 */
function serializeLayerStack(layerStack) {
    return layerStack.map(layer => {
        const serialized = {
            id: layer.id,
            name: layer.name,
            type: layer.type,
            permanent: layer.permanent,
            visible: layer.visible,
            opacity: layer.opacity,
            canvasData: layer.type === 'drawing' ? canvasToBase64(layer.canvas) : null,
            strokes: layer.strokes || [],
            collapsed: layer.collapsed || false,
            parentId: layer.parentId || null,
            rectangleId: layer.rectangleId || null
        };

        // 子レイヤーもシリアライズ
        if (layer.children && layer.children.length > 0) {
            serialized.children = layer.children.map(childLayer => ({
                id: childLayer.id,
                name: childLayer.name,
                type: childLayer.type,
                permanent: childLayer.permanent,
                visible: childLayer.visible,
                opacity: childLayer.opacity,
                parentId: childLayer.parentId,
                rectangleId: childLayer.rectangleId
            }));
        }

        return serialized;
    });
}

/**
 * プロファイルのサイズを計算する
 *
 * @private
 * @param {Object} profile - プロファイルオブジェクト
 * @returns {number} プロファイルのサイズ（バイト）
 */
function calculateProfileSize(profile) {
    try {
        const jsonStr = JSON.stringify(profile);
        return new Blob([jsonStr]).size;
    } catch (error) {
        console.error('calculateProfileSize: エラー', error);
        return 0;
    }
}

// ================
// プロファイル管理関数
// ================

/**
 * 現在の状態をプロファイルとして保存する
 *
 * @export
 * @param {string} profileName - プロファイル名
 * @returns {boolean} 保存に成功したかどうか
 *
 * @example
 * saveProfile('オフィスマップ');
 */
export function saveProfile(profileName) {
    if (!profileName || typeof profileName !== 'string' || profileName.trim() === '') {
        console.error('saveProfile: 無効なプロファイル名');
        return false;
    }

    const trimmedName = profileName.trim();

    try {
        // マップ境界情報を取得（3Dレンダラーから）
        let mapBounds = null;
        if (window.getMapBounds && typeof window.getMapBounds === 'function') {
            mapBounds = window.getMapBounds();
        }

        // 現在の状態をシリアライズ
        const profile = {
            name: trimmedName,
            timestamp: Date.now(),
            image: imageToBase64(mapState.image),
            imageFileName: mapState.imageFileName,
            yamlFileName: mapState.yamlFileName,
            metadata: mapState.metadata,
            mapBounds: mapBounds, // マップ境界情報を保存
            imageCropOffset: mapState.imageCropOffset || null, // クロップオフセット情報
            originalImageSize: mapState.originalImageSize || null, // 元の画像サイズ
            scale: mapState.scale,
            offsetX: mapState.offsetX,
            offsetY: mapState.offsetY,
            layers: mapState.layers,
            overlaySettings: mapState.overlaySettings,
            layerStack: serializeLayerStack(mapState.layerStack),
            drawingState: {
                color: mapState.drawingState.color,
                brushSize: mapState.drawingState.brushSize
            },
            rectangleToolState: {
                enabled: mapState.rectangleToolState.enabled,
                rectangles: mapState.rectangleToolState.rectangles,
                nextRectangleId: mapState.rectangleToolState.nextRectangleId
            }
        };

        // プロファイルサイズをチェック
        const profileSize = calculateProfileSize(profile);
        if (profileSize > MAX_PROFILE_SIZE) {
            console.error('saveProfile: プロファイルサイズが大きすぎます', profileSize);
            showError('プロファイルのサイズが大きすぎます（最大5MB）。画像サイズを縮小してください。', 2000);
            return false;
        }

        // LocalStorageに保存
        const profileKey = PROFILE_PREFIX + trimmedName;
        localStorage.setItem(profileKey, JSON.stringify(profile));

        // プロファイル一覧を更新
        let profiles = listProfiles();
        if (!profiles.includes(trimmedName)) {
            profiles.push(trimmedName);
            localStorage.setItem(PROFILES_LIST_KEY, JSON.stringify(profiles));
        }

        // 最後に使用したプロファイルとして設定
        setLastProfile(trimmedName);

        // 現在のプロファイルとして設定
        currentProfileName = trimmedName;
        localStorage.setItem(CURRENT_PROFILE_KEY, trimmedName);

        console.log('saveProfile: プロファイル保存成功', trimmedName);
        return true;
    } catch (error) {
        console.error('saveProfile: 保存エラー', error);
        if (error.name === 'QuotaExceededError') {
            showError('ストレージ容量が不足しています。不要なプロファイルを削除してください。', 2000);
        }
        return false;
    }
}

/**
 * プロファイルを読み込んで状態を復元する
 *
 * @export
 * @param {string} profileName - プロファイル名
 * @returns {Promise<boolean>} 読み込みに成功したかどうか
 *
 * @example
 * await loadProfile('オフィスマップ');
 */
export async function loadProfile(profileName) {
    if (!profileName || typeof profileName !== 'string' || profileName.trim() === '') {
        console.error('loadProfile: 無効なプロファイル名');
        return false;
    }

    try {
        const profileKey = PROFILE_PREFIX + profileName.trim();
        const profileJson = localStorage.getItem(profileKey);

        if (!profileJson) {
            console.error('loadProfile: プロファイルが見つかりません', profileName);
            return false;
        }

        const profile = JSON.parse(profileJson);

        // 画像を復元
        if (profile.image) {
            mapState.image = await base64ToImage(profile.image);
        } else {
            mapState.image = null;
        }

        // ファイル名を復元
        mapState.imageFileName = profile.imageFileName || null;
        mapState.yamlFileName = profile.yamlFileName || null;

        // メタデータを復元
        mapState.metadata = profile.metadata || null;

        // クロップオフセット情報を復元
        mapState.imageCropOffset = profile.imageCropOffset || null;
        mapState.originalImageSize = profile.originalImageSize || null;

        // マップ境界情報を復元（3Dレンダラーに設定）
        if (profile.mapBounds && window.setMapBounds && typeof window.setMapBounds === 'function') {
            window.setMapBounds(profile.mapBounds);
        }

        // ビューポート設定を復元
        mapState.scale = profile.scale || 1.0;
        mapState.offsetX = profile.offsetX || 0;
        mapState.offsetY = profile.offsetY || 0;

        // レイヤー設定を復元
        mapState.layers = profile.layers || {
            image: true,
            metadataOverlay: true
        };

        // オーバーレイ設定を復元
        mapState.overlaySettings = profile.overlaySettings || {
            showGrid: true,
            gridSpacingMeters: 1,
            showOrigin: true,
            showScaleBar: true
        };

        // 描画ツール設定を復元
        if (profile.drawingState) {
            mapState.drawingState.color = profile.drawingState.color || '#FF0000';
            mapState.drawingState.brushSize = profile.drawingState.brushSize || 5;
        }

        // 四角形ツール設定を復元
        if (profile.rectangleToolState) {
            // 四角形データは復元するが、ツール自体は常にオフにする
            mapState.rectangleToolState.enabled = false;
            mapState.rectangleToolState.rectangles = profile.rectangleToolState.rectangles || [];
            mapState.rectangleToolState.nextRectangleId = profile.rectangleToolState.nextRectangleId || 1;
            // 選択状態はクリア
            mapState.rectangleToolState.selectedRectangleId = null;
            mapState.rectangleToolState.rectangles.forEach(r => r.selected = false);
        }

        // レイヤースタックを復元（別途処理が必要）
        // この関数は状態のみを復元し、実際のレイヤー再構築は呼び出し側で行う
        mapState._profileLayerStack = profile.layerStack || [];

        // 最後に使用したプロファイルとして設定
        setLastProfile(profileName.trim());

        // 現在のプロファイルとして設定
        currentProfileName = profileName.trim();
        localStorage.setItem(CURRENT_PROFILE_KEY, profileName.trim());

        console.log('loadProfile: プロファイル読み込み成功', profileName);
        return true;
    } catch (error) {
        console.error('loadProfile: 読み込みエラー', error);
        return false;
    }
}

/**
 * プロファイルを削除する
 *
 * @export
 * @param {string} profileName - プロファイル名
 * @returns {boolean} 削除に成功したかどうか
 *
 * @example
 * deleteProfile('オフィスマップ');
 */
export function deleteProfile(profileName) {
    if (!profileName || typeof profileName !== 'string' || profileName.trim() === '') {
        console.error('deleteProfile: 無効なプロファイル名');
        return false;
    }

    try {
        const trimmedName = profileName.trim();
        const profileKey = PROFILE_PREFIX + trimmedName;

        // LocalStorageから削除
        localStorage.removeItem(profileKey);

        // プロファイル一覧を更新
        let profiles = listProfiles();
        profiles = profiles.filter(p => p !== trimmedName);
        localStorage.setItem(PROFILES_LIST_KEY, JSON.stringify(profiles));

        // 最後に使用したプロファイルがこれだった場合、クリア
        if (getLastProfile() === trimmedName) {
            localStorage.removeItem(LAST_PROFILE_KEY);
        }

        console.log('deleteProfile: プロファイル削除成功', trimmedName);
        return true;
    } catch (error) {
        console.error('deleteProfile: 削除エラー', error);
        return false;
    }
}

/**
 * プロファイル一覧を取得する
 *
 * @export
 * @returns {Array<string>} プロファイル名の配列
 *
 * @example
 * const profiles = listProfiles();
 */
export function listProfiles() {
    try {
        const profilesJson = localStorage.getItem(PROFILES_LIST_KEY);
        return profilesJson ? JSON.parse(profilesJson) : [];
    } catch (error) {
        console.error('listProfiles: エラー', error);
        return [];
    }
}

/**
 * 最後に使用したプロファイル名を取得する
 *
 * @export
 * @returns {string|null} プロファイル名
 *
 * @example
 * const lastProfile = getLastProfile();
 */
export function getLastProfile() {
    return localStorage.getItem(LAST_PROFILE_KEY);
}

/**
 * 最後に使用したプロファイル名を設定する
 *
 * @export
 * @param {string} profileName - プロファイル名
 * @returns {void}
 *
 * @example
 * setLastProfile('オフィスマップ');
 */
export function setLastProfile(profileName) {
    if (profileName && typeof profileName === 'string') {
        localStorage.setItem(LAST_PROFILE_KEY, profileName);
    }
}

/**
 * プロファイルをファイルにエクスポートする
 *
 * @export
 * @param {string} profileName - プロファイル名
 * @returns {void}
 *
 * @example
 * exportProfileToFile('オフィスマップ');
 */
export function exportProfileToFile(profileName) {
    if (!profileName || typeof profileName !== 'string' || profileName.trim() === '') {
        console.error('exportProfileToFile: 無効なプロファイル名');
        return;
    }

    try {
        const profileKey = PROFILE_PREFIX + profileName.trim();
        const profileJson = localStorage.getItem(profileKey);

        if (!profileJson) {
            alert('プロファイルが見つかりません');
            return;
        }

        // BOMを追加してUTF-8で保存
        const blob = new Blob(['\uFEFF' + profileJson], { type: 'application/json;charset=utf-8' });
        const url = URL.createObjectURL(blob);
        const a = document.createElement('a');
        a.href = url;
        a.download = `${profileName.trim()}_profile.json`;
        document.body.appendChild(a);
        a.click();
        document.body.removeChild(a);
        URL.revokeObjectURL(url);

        console.log('exportProfileToFile: エクスポート成功', profileName);

        // 成功メッセージを表示
        if (window.showSuccess && typeof window.showSuccess === 'function') {
            const fileName = `${profileName.trim()}_profile.json`;
            window.showSuccess(`プロファイルをエクスポートしました\nファイル名: ${fileName}\n保存場所: ブラウザのダウンロードフォルダ`, 3000);
        }
    } catch (error) {
        console.error('exportProfileToFile: エラー', error);
        alert('プロファイルのエクスポートに失敗しました');
    }
}

/**
 * プロファイルをファイルからインポートする
 *
 * @export
 * @param {File} file - インポートするファイル
 * @returns {Promise<boolean>} インポートに成功したかどうか
 *
 * @example
 * await importProfileFromFile(file);
 */
export async function importProfileFromFile(file) {
    if (!file) {
        console.error('importProfileFromFile: ファイルが指定されていません');
        return false;
    }

    try {
        const text = await file.text();
        const profile = JSON.parse(text);

        if (!profile.name) {
            alert('無効なプロファイルファイルです');
            return false;
        }

        // プロファイルサイズをチェック
        const profileSize = calculateProfileSize(profile);
        if (profileSize > MAX_PROFILE_SIZE) {
            alert('プロファイルのサイズが大きすぎます（最大5MB）');
            return false;
        }

        // LocalStorageに保存
        const profileKey = PROFILE_PREFIX + profile.name;
        localStorage.setItem(profileKey, JSON.stringify(profile));

        // プロファイル一覧を更新
        let profiles = listProfiles();
        if (!profiles.includes(profile.name)) {
            profiles.push(profile.name);
            localStorage.setItem(PROFILES_LIST_KEY, JSON.stringify(profiles));
        }

        console.log('importProfileFromFile: インポート成功', profile.name);
        return true;
    } catch (error) {
        console.error('importProfileFromFile: エラー', error);
        alert('プロファイルのインポートに失敗しました');
        return false;
    }
}

/**
 * プロファイルの詳細情報を取得する
 *
 * @export
 * @param {string} profileName - プロファイル名
 * @returns {Object|null} プロファイル情報
 *
 * @example
 * const info = getProfileInfo('オフィスマップ');
 */
export function getProfileInfo(profileName) {
    if (!profileName || typeof profileName !== 'string' || profileName.trim() === '') {
        return null;
    }

    try {
        const profileKey = PROFILE_PREFIX + profileName.trim();
        const profileJson = localStorage.getItem(profileKey);

        if (!profileJson) {
            return null;
        }

        const profile = JSON.parse(profileJson);

        return {
            name: profile.name,
            timestamp: profile.timestamp,
            hasImage: !!profile.image,
            hasMetadata: !!profile.metadata,
            layerCount: (profile.layerStack || []).length,
            size: calculateProfileSize(profile)
        };
    } catch (error) {
        console.error('getProfileInfo: エラー', error);
        return null;
    }
}

/**
 * 現在参照しているプロファイル名を取得する
 *
 * @export
 * @returns {string|null} 現在のプロファイル名、なければnull
 *
 * @example
 * const currentProfile = getCurrentProfile();
 */
export function getCurrentProfile() {
    // メモリ上にない場合はLocalStorageから復元
    if (!currentProfileName) {
        currentProfileName = localStorage.getItem(CURRENT_PROFILE_KEY);
    }
    return currentProfileName;
}

/**
 * 現在参照しているプロファイルに上書き保存する
 *
 * @export
 * @returns {boolean} 保存に成功したかどうか
 *
 * @example
 * saveCurrentProfile();
 */
export function saveCurrentProfile() {
    const current = getCurrentProfile();

    if (!current) {
        console.warn('saveCurrentProfile: 現在のプロファイルが設定されていません');
        return false;
    }

    return saveProfile(current);
}

/**
 * 名前を付けてプロファイルを保存する
 *
 * @export
 * @param {string} newProfileName - 新しいプロファイル名
 * @returns {boolean} 保存に成功したかどうか
 *
 * @example
 * saveProfileAs('新しいマップ');
 */
export function saveProfileAs(newProfileName) {
    return saveProfile(newProfileName);
}
