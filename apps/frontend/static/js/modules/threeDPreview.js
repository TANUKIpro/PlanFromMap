/**
 * @file threeDPreview.js
 * @description プロパティプレビュー用3D描画モジュール
 *
 * オブジェクトカタログのプロパティパネルに表示される
 * 3Dプレビューの描画を担当します。
 *
 * @requires three
 * @requires ./threeDObjects.js
 * @requires ../modules/rectangleManager.js
 * @requires ../modules/objectPropertyManager.js
 * @requires ../utils/threeHelpers.js
 *
 * @exports initializePropertyPreview - プロパティプレビュー初期化
 * @exports renderPropertyPreview - プロパティプレビュー描画
 * @exports getPreviewState - プレビュー状態取得
 * @exports drawPreviewModel - Canvas 2D互換性用（非推奨）
 * @exports drawPreviewFrontDirection - Canvas 2D互換性用（非推奨）
 * @exports worldToPreviewIso - Canvas 2D互換性用（非推奨）
 */

import * as THREE from 'three';
import { create3DObjectMesh } from './threeDObjects.js';
import { getRectangleById } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { disposeMesh } from '../utils/threeHelpers.js';

// ================
// プレビューシーン状態
// ================

let previewScene = null;
let previewCamera = null;
let previewRenderer = null;

// プレビュー状態
const previewState = {
    canvas: null,
    container: null,
    rotation: 30,
    tilt: 20,
    scale: 15,
    currentRectangleId: null,
    objectMesh: null,
    initialized: false
};

/**
 * プロパティプレビューを初期化
 *
 * @returns {void}
 */
export function initializePropertyPreview() {
    const canvas = document.getElementById('propertyPreviewCanvas');
    if (!canvas) {
        console.warn('initializePropertyPreview: プレビューCanvas要素が見つかりません');
        return;
    }

    previewState.canvas = canvas;
    previewState.container = canvas.parentElement;

    // Three.jsプレビューシーンの初期化
    previewScene = new THREE.Scene();
    previewScene.background = new THREE.Color(0xf7fafc);

    const width = previewState.container.clientWidth;
    const height = previewState.container.clientHeight;

    previewCamera = new THREE.PerspectiveCamera(50, width / height, 0.1, 100);
    previewCamera.position.set(2, 2, 2);
    previewCamera.lookAt(0, 0, 0);

    previewRenderer = new THREE.WebGLRenderer({
        canvas: previewState.canvas,
        antialias: true
    });
    previewRenderer.setSize(width, height);
    previewRenderer.setPixelRatio(window.devicePixelRatio);

    // ライティング
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    previewScene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(5, 5, 5);
    previewScene.add(directionalLight);

    previewState.initialized = true;

    // プレビューアニメーションループ
    function animatePreview() {
        requestAnimationFrame(animatePreview);
        if (previewRenderer && previewScene && previewCamera) {
            previewRenderer.render(previewScene, previewCamera);
        }
    }
    animatePreview();
}

/**
 * プロパティプレビューを描画
 *
 * @param {string} rectangleId - 四角形ID
 * @returns {void}
 */
export function renderPropertyPreview(rectangleId) {
    if (!previewState.initialized) {
        initializePropertyPreview();
    }

    if (!previewScene || !previewCamera || !previewRenderer) {
        console.warn('renderPropertyPreview: プレビューが初期化されていません');
        return;
    }

    previewState.currentRectangleId = rectangleId;

    // 既存のオブジェクトを削除
    if (previewState.objectMesh) {
        previewScene.remove(previewState.objectMesh);
        disposeMesh(previewState.objectMesh);
    }

    const coords3D = get3DCoordinates(rectangleId);
    if (!coords3D) return;

    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return;

    // プレビュー用メッシュを作成
    const mesh = create3DObjectMesh(coords3D, rectangle, false);
    if (mesh) {
        mesh.position.set(0, 0, 0);
        previewScene.add(mesh);
        previewState.objectMesh = mesh;

        // カメラを調整してオブジェクト全体が見えるように
        const box = new THREE.Box3().setFromObject(mesh);
        const size = box.getSize(new THREE.Vector3());
        const maxDim = Math.max(size.x, size.y, size.z);
        const distance = maxDim * 2;

        previewCamera.position.set(distance, distance, distance);
        previewCamera.lookAt(0, 0, 0);
    }
}

/**
 * プレビュー状態を取得
 *
 * @returns {Object} プレビュー状態
 */
export function getPreviewState() {
    return previewState;
}

// ================
// Canvas 2D互換性用関数（非推奨）
// ================

/**
 * プレビュー用3Dモデルを描画（Canvas 2D互換性のため）
 *
 * @deprecated Three.jsで直接描画されるため、この関数は互換性のために残されています
 * @param {CanvasRenderingContext2D} ctx - Canvas 2Dコンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - 色
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 * @param {string} objectType - オブジェクトタイプ
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 */
export function drawPreviewModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
    // Three.jsでは使用されないが、互換性のために保持
    console.warn('drawPreviewModel: Three.jsでは直接描画されます');
}

/**
 * プレビュー用前面方向矢印を描画（Canvas 2D互換性のため）
 *
 * @deprecated Three.jsで直接描画されるため、この関数は互換性のために残されています
 * @param {CanvasRenderingContext2D} ctx - Canvas 2Dコンテキスト
 * @param {Object} coords3D - 3D座標情報
 * @param {string} frontDirection - 前面方向
 * @param {number} centerX - 中心X座標
 * @param {number} centerY - 中心Y座標
 */
export function drawPreviewFrontDirection(ctx, coords3D, frontDirection, centerX, centerY) {
    // Three.jsでは使用されないが、互換性のために保持
    console.warn('drawPreviewFrontDirection: Three.jsでは直接描画されます');
}

/**
 * プレビュー用等角投影変換（Canvas 2D互換性のため）
 *
 * @deprecated Three.jsでは使用されません
 * @param {number} x - X座標
 * @param {number} y - Y座標
 * @param {number} z - Z座標
 * @returns {Object} 投影座標 {x, y}
 */
export function worldToPreviewIso(x, y, z) {
    // Three.jsでは使用されないが、互換性のために保持
    return { x: 0, y: 0 };
}
