/**
 * @file threeHelpers.js
 * @description Three.jsユーティリティ関数
 *
 * Three.jsに関連する共通のヘルパー関数を提供します。
 * - 色操作（明るく/暗く）
 * - メッシュの破棄（メモリ管理）
 * - グループのクリア
 *
 * @exports lightenColor - Three.Colorを明るくする
 * @exports darkenColor - Three.Colorを暗くする
 * @exports lightenColorHex - 16進数色を明るくする
 * @exports darkenColorHex - 16進数色を暗くする
 * @exports disposeMesh - メッシュを破棄する
 * @exports clearGroup - グループをクリアする
 */

import * as THREE from 'three';

/**
 * 色を明るくする（THREE.Color用）
 *
 * @param {THREE.Color} color - 元の色
 * @param {number} percent - 明るくする割合（0-100）
 * @returns {THREE.Color} 明るくした色
 *
 * @example
 * const lightColor = lightenColor(new THREE.Color(0xff0000), 20);
 */
export function lightenColor(color, percent) {
    const hsl = {};
    color.getHSL(hsl);
    hsl.l = Math.min(1, hsl.l + percent / 100);
    const newColor = new THREE.Color();
    newColor.setHSL(hsl.h, hsl.s, hsl.l);
    return newColor;
}

/**
 * 色を暗くする（THREE.Color用）
 *
 * @param {THREE.Color} color - 元の色
 * @param {number} percent - 暗くする割合（0-100）
 * @returns {THREE.Color} 暗くした色
 *
 * @example
 * const darkColor = darkenColor(new THREE.Color(0xff0000), 20);
 */
export function darkenColor(color, percent) {
    const hsl = {};
    color.getHSL(hsl);
    hsl.l = Math.max(0, hsl.l - percent / 100);
    const newColor = new THREE.Color();
    newColor.setHSL(hsl.h, hsl.s, hsl.l);
    return newColor;
}

/**
 * 16進数色を明るくする
 *
 * @param {string} hexColor - 16進数色（例: "#ff0000"）
 * @param {number} percent - 明るくする割合（0-100）
 * @returns {string} 明るくした16進数色
 *
 * @example
 * const lightHex = lightenColorHex("#ff0000", 20);
 */
export function lightenColorHex(hexColor, percent) {
    const color = new THREE.Color(hexColor);
    const lightened = lightenColor(color, percent);
    return '#' + lightened.getHexString();
}

/**
 * 16進数色を暗くする
 *
 * @param {string} hexColor - 16進数色（例: "#ff0000"）
 * @param {number} percent - 暗くする割合（0-100）
 * @returns {string} 暗くした16進数色
 *
 * @example
 * const darkHex = darkenColorHex("#ff0000", 20);
 */
export function darkenColorHex(hexColor, percent) {
    const color = new THREE.Color(hexColor);
    const darkened = darkenColor(color, percent);
    return '#' + darkened.getHexString();
}

/**
 * メッシュを破棄する（メモリリーク防止）
 *
 * ジオメトリ、マテリアル、子メッシュを再帰的に破棄します。
 *
 * @param {THREE.Object3D} mesh - 破棄するメッシュ
 *
 * @example
 * disposeMesh(someMesh);
 */
export function disposeMesh(mesh) {
    if (!mesh) return;

    // ジオメトリを破棄
    if (mesh.geometry) {
        mesh.geometry.dispose();
    }

    // マテリアルを破棄
    if (mesh.material) {
        if (Array.isArray(mesh.material)) {
            mesh.material.forEach(m => {
                if (m.map) m.map.dispose();
                m.dispose();
            });
        } else {
            if (mesh.material.map) mesh.material.map.dispose();
            mesh.material.dispose();
        }
    }

    // グループの場合は子も破棄
    if (mesh.children && mesh.children.length > 0) {
        // 子を配列にコピーしてから処理（forEach中の削除を避けるため）
        const children = [...mesh.children];
        children.forEach(child => disposeMesh(child));
    }
}

/**
 * グループ内のすべてのオブジェクトをクリアする
 *
 * @param {THREE.Group} group - クリアするグループ
 *
 * @example
 * clearGroup(myGroup);
 */
export function clearGroup(group) {
    if (!group) return;

    while (group.children.length > 0) {
        const child = group.children[0];
        group.remove(child);
        disposeMesh(child);
    }
}
