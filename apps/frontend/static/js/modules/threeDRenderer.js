/**
 * @file threeDRenderer.js
 * @description 2Dマップから3D構造を生成・描画（Three.js使用）
 *
 * Three.jsを使用して本格的な3D表現を行います。
 * 座標系はROS互換性を維持します。
 *
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../modules/objectPropertyManager.js - プロパティ管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ../ui/viewCube.js - ViewCube（視点切り替えコントロール）
 *
 * @exports initialize3DView - 3Dビュー初期化
 * @exports render3DScene - 3Dシーン描画
 * @exports update3DObject - 特定オブジェクト更新
 * @exports resize3DView - 3Dビューのリサイズ
 * @exports set3DViewRotation - 3Dビューの回転角度設定
 * @exports reset3DView - 3Dビューのリセット
 * @exports select3DObject - オブジェクト選択
 * @exports deselect3DObject - オブジェクト選択解除
 * @exports goto2DMap - 2Dマップへ移動
 * @exports gotoObjectCatalog - オブジェクトカタログへ移動
 * @exports getMapBounds - マップ境界取得
 * @exports setMapBounds - マップ境界設定
 * @exports initializePropertyPreview - プロパティプレビュー初期化
 * @exports renderPropertyPreview - プロパティプレビュー描画
 * @exports getPreviewState - プレビュー状態取得
 * @exports drawPreviewModel - プレビュー用3Dモデル描画
 * @exports drawPreviewFrontDirection - プレビュー用前面方向矢印描画
 * @exports worldToPreviewIso - プレビュー用等角投影変換
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mapState } from '../state/mapState.js';
import { getAllRectangles, getRectangleById, getRectangleLayer } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { initializeViewCube, updateViewCube } from '../ui/viewCube.js';
import { analyzeMapBounds } from '../utils/imageProcessing.js';
import { disposeMesh, clearGroup } from '../utils/threeHelpers.js';
import { create3DObjectMesh } from './threeDObjects.js';
import {
    initializePropertyPreview as initPreview,
    renderPropertyPreview as renderPreview,
    getPreviewState as getPreview,
    drawPreviewModel,
    drawPreviewFrontDirection,
    worldToPreviewIso
} from './threeDPreview.js';

// ================
// グローバル変数 (Three.js)
// ================

let scene = null;
let camera = null;
let renderer = null;
let controls = null;

// オブジェクトグループ
let mapGroup = null;          // マップテクスチャ
let gridGroup = null;         // グリッド
let originGroup = null;       // 原点マーカー
let objectsGroup = null;      // 3Dオブジェクト

// レイキャスター（オブジェクト選択用）
let raycaster = null;
let mouse = new THREE.Vector2();

// ================
// 3Dビュー状態
// ================

const view3DState = {
    canvas: null,
    container: null,
    isVisible: false,
    rotation: 45,           // 回転角度（度）- ViewCube互換性のため保持
    tilt: 30,              // 傾き角度（度）- ViewCube互換性のため保持
    selectedObjectId: null, // 選択されたオブジェクトのID
    mapBounds: null,        // マップの有効領域 {minX, minY, maxX, maxY, centerX, centerY}
    objectMeshes: new Map(), // rectangleId -> THREE.Mesh のマッピング
    initialized: false
};

// ================
// 初期化
// ================

/**
 * 3Dビューを初期化する
 *
 * @returns {void}
 */
export function initialize3DView() {
    const canvas = document.getElementById('view3DCanvas');
    if (!canvas) {
        console.error('initialize3DView: 3DビューのCanvas要素が見つかりません');
        return;
    }

    view3DState.canvas = canvas;
    view3DState.container = canvas.parentElement;

    // Three.jsの初期化
    initThreeJS();

    // イベントリスナーを設定
    setupEventListeners();

    // ViewCubeを初期化
    const viewCubeCanvas = document.getElementById('viewCubeCanvas');
    if (viewCubeCanvas) {
        initializeViewCube(viewCubeCanvas, handleViewCubeChange);
    }

    view3DState.initialized = true;
}

/**
 * Three.jsの初期化
 * @private
 */
function initThreeJS() {
    // シーンの作成
    scene = new THREE.Scene();
    scene.background = new THREE.Color(0xf7fafc);

    // カメラの作成（透視投影）
    const width = view3DState.container.clientWidth;
    const height = view3DState.container.clientHeight;
    camera = new THREE.PerspectiveCamera(50, width / height, 0.1, 1000);

    // カメラの初期位置を設定（等角投影風の視点）
    updateCameraPosition();

    // レンダラーの作成
    renderer = new THREE.WebGLRenderer({
        canvas: view3DState.canvas,
        antialias: true
    });
    renderer.setSize(width, height);
    renderer.setPixelRatio(window.devicePixelRatio);
    renderer.shadowMap.enabled = true;
    renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // OrbitControlsの設定（マウス操作）
    controls = new OrbitControls(camera, renderer.domElement);
    controls.enableDamping = true;
    controls.dampingFactor = 0.05;
    controls.screenSpacePanning = false;
    controls.minDistance = 2;
    controls.maxDistance = 100;
    controls.maxPolarAngle = Math.PI / 2.1; // 地面より下を見れないように

    // カメラ変更時のコールバック
    controls.addEventListener('change', () => {
        updateViewCubeFromCamera();
    });

    // ライティングの設定
    setupLighting();

    // グループの作成
    mapGroup = new THREE.Group();
    gridGroup = new THREE.Group();
    originGroup = new THREE.Group();
    objectsGroup = new THREE.Group();

    scene.add(mapGroup);
    scene.add(gridGroup);
    scene.add(originGroup);
    scene.add(objectsGroup);

    // レイキャスターの初期化
    raycaster = new THREE.Raycaster();

    // アニメーションループの開始
    animate();
}

/**
 * ライティングの設定
 * @private
 */
function setupLighting() {
    // 環境光（全体を明るく）
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    scene.add(ambientLight);

    // 指向性ライト（太陽光のような）
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.5);
    directionalLight.position.set(5, 10, 7.5);
    directionalLight.castShadow = true;
    directionalLight.shadow.camera.near = 0.1;
    directionalLight.shadow.camera.far = 50;
    directionalLight.shadow.camera.left = -10;
    directionalLight.shadow.camera.right = 10;
    directionalLight.shadow.camera.top = 10;
    directionalLight.shadow.camera.bottom = -10;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    scene.add(directionalLight);

    // 補助ライト（影を柔らかく）
    const fillLight = new THREE.DirectionalLight(0xffffff, 0.2);
    fillLight.position.set(-5, 5, -5);
    scene.add(fillLight);
}

/**
 * カメラ位置を更新（rotation/tiltに基づいて）
 * @private
 */
function updateCameraPosition() {
    if (!camera) return;

    const distance = 10; // カメラの距離
    const rotRad = view3DState.rotation * Math.PI / 180;
    const tiltRad = view3DState.tilt * Math.PI / 180;

    // 等角投影風の視点を作成
    const x = distance * Math.sin(tiltRad) * Math.cos(rotRad);
    const y = distance * Math.cos(tiltRad);
    const z = distance * Math.sin(tiltRad) * Math.sin(rotRad);

    camera.position.set(x, y, z);
    camera.lookAt(0, 0, 0);

    if (controls) {
        controls.target.set(0, 0, 0);
        controls.update();
    }
}

/**
 * ViewCubeからカメラの状態を更新
 * @private
 */
function updateViewCubeFromCamera() {
    if (!camera || !controls) return;

    // カメラの方向から rotation と tilt を計算
    const direction = new THREE.Vector3();
    camera.getWorldDirection(direction);
    direction.negate(); // カメラが見ている方向

    const rotation = Math.atan2(direction.z, direction.x) * 180 / Math.PI;
    const tilt = Math.acos(direction.y) * 180 / Math.PI;

    view3DState.rotation = rotation;
    view3DState.tilt = tilt;

    updateViewCube(rotation, tilt);
}

/**
 * ViewCubeからの視点変更を処理
 *
 * @private
 * @param {number} rotation - 回転角度
 * @param {number} tilt - 傾き角度
 */
function handleViewCubeChange(rotation, tilt) {
    view3DState.rotation = rotation;
    view3DState.tilt = tilt;
    updateCameraPosition();
}

/**
 * アニメーションループ
 * @private
 */
function animate() {
    requestAnimationFrame(animate);

    if (controls) {
        controls.update();
    }

    if (renderer && scene && camera) {
        renderer.render(scene, camera);
    }
}

/**
 * イベントリスナーをセットアップ
 *
 * @private
 */
function setupEventListeners() {
    if (!view3DState.canvas) return;

    // マウスクリック（オブジェクト選択）
    view3DState.canvas.addEventListener('click', handle3DClick);

    // ウィンドウリサイズ
    window.addEventListener('resize', () => resize3DView());
}

// ================
// 描画
// ================

/**
 * 3Dシーン全体を描画する
 *
 * @returns {void}
 */
export function render3DScene() {
    if (!scene || !camera || !renderer) {
        console.warn('render3DScene: Three.jsが初期化されていません');
        return;
    }

    // マップの有効領域を解析（初回のみ）
    if (!view3DState.mapBounds && mapState.image) {
        view3DState.mapBounds = analyzeMapBounds(mapState.image);
    }

    // マップテクスチャを更新
    updateMapTexture();

    // グリッドを更新
    if (mapState.overlaySettings.showGrid) {
        updateGrid();
    } else {
        clearGroup(gridGroup);
    }

    // 原点を更新
    if (mapState.overlaySettings.showOrigin) {
        updateOrigin();
    } else {
        clearGroup(originGroup);
    }

    // オブジェクトを更新
    const rectangleLayer = getRectangleLayer();
    if (rectangleLayer && rectangleLayer.visible) {
        updateObjects();
    } else {
        clearGroup(objectsGroup);
        view3DState.objectMeshes.clear();
    }

    // レンダリング（animateループで自動実行されるため、ここでは不要）
}

/**
 * マップテクスチャを更新
 * @private
 */
function updateMapTexture() {
    // 既存のマップをクリア
    clearGroup(mapGroup);

    if (!mapState.image || !view3DState.mapBounds) {
        return;
    }

    const image = mapState.image;
    const bounds = view3DState.mapBounds;
    const resolution = mapState.metadata?.resolution || 0.05;

    // マップの原点を取得（ROSのマップ原点: マップ左下の実世界座標）
    const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
    const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;

    // 有効領域の実世界座標を計算
    const boundsMinX = originX + bounds.minX * resolution;
    const boundsMinY = originY + (image.height - bounds.maxY) * resolution;
    const boundsMaxX = originX + bounds.maxX * resolution;
    const boundsMaxY = originY + (image.height - bounds.minY) * resolution;

    const boundsWidthMeters = boundsMaxX - boundsMinX;
    const boundsHeightMeters = boundsMaxY - boundsMinY;

    // マップ画像をテクスチャとして作成
    const texture = new THREE.Texture(image);
    texture.needsUpdate = true;
    texture.minFilter = THREE.LinearFilter;
    texture.magFilter = THREE.LinearFilter;

    // UV座標を計算（有効領域のみ）
    const uvMinX = bounds.minX / image.width;
    const uvMinY = bounds.minY / image.height;
    const uvMaxX = bounds.maxX / image.width;
    const uvMaxY = bounds.maxY / image.height;

    // 床面のジオメトリを作成
    const geometry = new THREE.PlaneGeometry(boundsWidthMeters, boundsHeightMeters);

    // UV座標を設定
    const uvAttr = geometry.attributes.uv;
    uvAttr.setXY(0, uvMinX, 1 - uvMaxY);  // 左下
    uvAttr.setXY(1, uvMaxX, 1 - uvMaxY);  // 右下
    uvAttr.setXY(2, uvMinX, 1 - uvMinY);  // 左上
    uvAttr.setXY(3, uvMaxX, 1 - uvMinY);  // 右上

    const material = new THREE.MeshBasicMaterial({
        map: texture,
        side: THREE.DoubleSide
    });

    const mesh = new THREE.Mesh(geometry, material);

    // 床面に配置（Z=0、XY平面）
    mesh.rotation.x = -Math.PI / 2;

    // マップの中心を原点に配置
    const centerX = (boundsMinX + boundsMaxX) / 2;
    const centerY = (boundsMinY + boundsMaxY) / 2;
    mesh.position.set(centerX, 0, -centerY); // Y軸反転に注意（ROS座標系）

    mesh.receiveShadow = true;

    mapGroup.add(mesh);
}

/**
 * グリッドを更新
 * @private
 */
function updateGrid() {
    clearGroup(gridGroup);

    const gridSize = mapState.gridWidthInMeters || 1;

    if (!view3DState.mapBounds || !mapState.image) {
        // デフォルトのグリッド
        const grid = new THREE.GridHelper(20, 20 / gridSize, 0xcbd5e0, 0xcbd5e0);
        gridGroup.add(grid);
        return;
    }

    const resolution = mapState.metadata?.resolution || 0.05;
    const bounds = view3DState.mapBounds;
    const image = mapState.image;

    const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
    const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;

    const boundsMinX = originX + bounds.minX * resolution;
    const boundsMinY = originY + (image.height - bounds.maxY) * resolution;
    const boundsMaxX = originX + bounds.maxX * resolution;
    const boundsMaxY = originY + (image.height - bounds.minY) * resolution;

    const gridStartX = Math.floor(boundsMinX / gridSize) * gridSize;
    const gridStartY = Math.floor(boundsMinY / gridSize) * gridSize;
    const gridEndX = Math.ceil(boundsMaxX / gridSize) * gridSize;
    const gridEndY = Math.ceil(boundsMaxY / gridSize) * gridSize;

    const material = new THREE.LineBasicMaterial({ color: 0xcbd5e0 });

    // X方向のグリッド線
    for (let x = gridStartX; x <= gridEndX; x += gridSize) {
        const geometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(x, 0, -gridStartY),
            new THREE.Vector3(x, 0, -gridEndY)
        ]);
        const line = new THREE.Line(geometry, material);
        gridGroup.add(line);
    }

    // Y方向のグリッド線
    for (let y = gridStartY; y <= gridEndY; y += gridSize) {
        const geometry = new THREE.BufferGeometry().setFromPoints([
            new THREE.Vector3(gridStartX, 0, -y),
            new THREE.Vector3(gridEndX, 0, -y)
        ]);
        const line = new THREE.Line(geometry, material);
        gridGroup.add(line);
    }
}

/**
 * 原点マーカーを更新
 * @private
 */
function updateOrigin() {
    clearGroup(originGroup);

    if (!mapState.metadata || !mapState.metadata.origin) {
        return;
    }

    const crossSize = 0.15;  // 15cm
    const lineWidth = 0.02;  // 2cm
    const color = 0xe74c3c;  // 赤

    const material = new THREE.MeshBasicMaterial({ color: color });

    // 十字を作成
    const crossGeometry1 = new THREE.BoxGeometry(crossSize * 2, lineWidth, lineWidth);
    const crossGeometry2 = new THREE.BoxGeometry(lineWidth, lineWidth, crossSize * 2);

    const cross1 = new THREE.Mesh(crossGeometry1, material);
    const cross2 = new THREE.Mesh(crossGeometry2, material);

    cross1.position.y = lineWidth / 2;
    cross2.position.y = lineWidth / 2;

    originGroup.add(cross1);
    originGroup.add(cross2);

    // 方向矢印
    const theta = Array.isArray(mapState.metadata.origin) && mapState.metadata.origin.length >= 3
        ? mapState.metadata.origin[2]
        : 0;

    const arrowLength = 0.5;  // 50cm
    const arrowWidth = 0.015; // 1.5cm

    // 矢印の軸
    const shaftGeometry = new THREE.BoxGeometry(arrowWidth, arrowWidth, arrowLength);
    const shaft = new THREE.Mesh(shaftGeometry, material);
    shaft.position.set(
        Math.cos(theta) * arrowLength / 2,
        arrowWidth / 2,
        -Math.sin(theta) * arrowLength / 2
    );
    shaft.rotation.y = -theta;
    originGroup.add(shaft);

    // 矢印の先端（円錐）
    const headGeometry = new THREE.ConeGeometry(0.04, 0.1, 8);
    const head = new THREE.Mesh(headGeometry, material);
    head.position.set(
        Math.cos(theta) * arrowLength,
        0.05,
        -Math.sin(theta) * arrowLength
    );
    head.rotation.set(Math.PI / 2, 0, theta);
    originGroup.add(head);
}

/**
 * オブジェクトを更新
 * @private
 */
function updateObjects() {
    const rectangles = getAllRectangles();
    const currentIds = new Set(rectangles.map(r => r.id));

    // 削除されたオブジェクトを除去
    for (const [id, mesh] of view3DState.objectMeshes) {
        if (!currentIds.has(id)) {
            objectsGroup.remove(mesh);
            disposeMesh(mesh);
            view3DState.objectMeshes.delete(id);
        }
    }

    // オブジェクトを更新または作成
    rectangles.forEach(rectangle => {
        updateOrCreate3DObject(rectangle);
    });
}

/**
 * 3Dオブジェクトを更新または作成
 * @private
 */
function updateOrCreate3DObject(rectangle) {
    const coords3D = get3DCoordinates(rectangle.id);
    if (!coords3D) return;

    // 既存のメッシュを削除
    const existingMesh = view3DState.objectMeshes.get(rectangle.id);
    if (existingMesh) {
        objectsGroup.remove(existingMesh);
        disposeMesh(existingMesh);
    }

    // 新しいメッシュを作成
    const isSelected = view3DState.selectedObjectId === rectangle.id;
    const mesh = create3DObjectMesh(coords3D, rectangle, isSelected);

    if (mesh) {
        mesh.userData.rectangleId = rectangle.id;
        objectsGroup.add(mesh);
        view3DState.objectMeshes.set(rectangle.id, mesh);
    }
}


// ================
// オブジェクト選択
// ================

/**
 * オブジェクトを選択
 *
 * @param {string} objectId - オブジェクトID
 */
export function select3DObject(objectId) {
    if (view3DState.selectedObjectId === objectId) return;

    // 以前選択されていたオブジェクトを更新
    if (view3DState.selectedObjectId) {
        const prevRectangle = getRectangleById(view3DState.selectedObjectId);
        if (prevRectangle) {
            updateOrCreate3DObject(prevRectangle);
        }
    }

    view3DState.selectedObjectId = objectId;

    // 選択されたオブジェクトを更新
    const rectangle = getRectangleById(objectId);
    if (rectangle) {
        updateOrCreate3DObject(rectangle);
    }
}

/**
 * オブジェクト選択を解除
 */
export function deselect3DObject() {
    if (!view3DState.selectedObjectId) return;

    const previousId = view3DState.selectedObjectId;
    view3DState.selectedObjectId = null;

    // 以前選択されていたオブジェクトを更新
    const rectangle = getRectangleById(previousId);
    if (rectangle) {
        updateOrCreate3DObject(rectangle);
    }
}

/**
 * マウスクリックハンドラ（オブジェクト選択）
 * @private
 */
function handle3DClick(event) {
    if (!camera || !raycaster) return;

    // マウス座標を正規化
    const rect = view3DState.canvas.getBoundingClientRect();
    mouse.x = ((event.clientX - rect.left) / rect.width) * 2 - 1;
    mouse.y = -((event.clientY - rect.top) / rect.height) * 2 + 1;

    // レイキャスト
    raycaster.setFromCamera(mouse, camera);

    const intersects = raycaster.intersectObjects(objectsGroup.children, true);

    if (intersects.length > 0) {
        // 最も近いオブジェクトを取得
        let intersectedObject = intersects[0].object;

        // 親グループまで遡る
        while (intersectedObject.parent && !intersectedObject.userData.rectangleId) {
            intersectedObject = intersectedObject.parent;
        }

        if (intersectedObject.userData.rectangleId) {
            select3DObject(intersectedObject.userData.rectangleId);
        }
    } else {
        deselect3DObject();
    }
}

// ================
// ナビゲーション
// ================

/**
 * 2Dマップビューに移動
 */
export function goto2DMap() {
    // 2Dマップサブタブに切り替え
    const map2DTab = document.querySelector('[onclick*="switchMapSubTab(\'map2D\'"]');
    if (map2DTab) {
        map2DTab.click();
    }
}

/**
 * オブジェクトカタログビューに移動
 */
export function gotoObjectCatalog() {
    // オブジェクトカタログタブに切り替え
    const catalogTab = document.querySelector('[onclick*="switchTab(\'objectCatalog\'"]');
    if (catalogTab) {
        catalogTab.click();
    }
}

// ================
// ビュー制御
// ================

/**
 * 特定オブジェクトを更新
 *
 * @param {string} rectangleId - 四角形ID
 */
export function update3DObject(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (rectangle) {
        updateOrCreate3DObject(rectangle);
    }
}

/**
 * 3Dビューをリサイズ
 */
export function resize3DView() {
    if (!renderer || !camera || !view3DState.container) return;

    const width = view3DState.container.clientWidth;
    const height = view3DState.container.clientHeight;

    camera.aspect = width / height;
    camera.updateProjectionMatrix();

    renderer.setSize(width, height);
}

/**
 * 3Dビューの回転角度を設定
 *
 * @param {number} rotation - 回転角度
 * @param {number} tilt - 傾き角度
 */
export function set3DViewRotation(rotation, tilt) {
    view3DState.rotation = rotation;
    view3DState.tilt = tilt;
    updateCameraPosition();
}

/**
 * 3Dビューをリセット
 */
export function reset3DView() {
    view3DState.rotation = 45;
    view3DState.tilt = 30;
    updateCameraPosition();

    if (controls) {
        controls.target.set(0, 0, 0);
        controls.update();
    }
}

/**
 * マップ境界を取得
 *
 * @returns {Object|null} マップ境界
 */
export function getMapBounds() {
    return view3DState.mapBounds;
}

/**
 * マップ境界を設定
 *
 * @param {Object} bounds - マップ境界
 */
export function setMapBounds(bounds) {
    view3DState.mapBounds = bounds;
}

// ================
// プロパティプレビュー（モジュールから再エクスポート）
// ================

/**
 * プロパティプレビューを初期化
 */
export function initializePropertyPreview() {
    return initPreview();
}

/**
 * プロパティプレビューを描画
 *
 * @param {string} rectangleId - 四角形ID
 */
export function renderPropertyPreview(rectangleId) {
    return renderPreview(rectangleId);
}

/**
 * プレビュー状態を取得
 *
 * @returns {Object} プレビュー状態
 */
export function getPreviewState() {
    return getPreview();
}

// Canvas 2D互換性用（非推奨）- モジュールから再エクスポート
export { drawPreviewModel, drawPreviewFrontDirection, worldToPreviewIso };
