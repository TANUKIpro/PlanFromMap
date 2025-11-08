/**
 * @file threeDRenderer.js
 * @description 2Dマップから3D構造を生成・描画（Three.js使用）
 *
 * Three.jsを使用して、WebGLベースの3D表現を行います。
 * OrthographicCameraを使用した等角投影で、既存のCanvas 2D実装と互換性を保ちます。
 *
 * @requires three - Three.js library (CDN経由でインポート)
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../modules/objectPropertyManager.js - プロパティ管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 * @requires ../ui/viewCube.js - ViewCube（視点切り替えコントロール）
 * @requires ../utils/imageProcessing.js - 画像処理
 *
 * @exports initialize3DView - 3Dビュー初期化
 * @exports render3DScene - 3Dシーン描画
 * @exports update3DObject - 特定オブジェクト更新
 * @exports resize3DView - 3Dビューのリサイズ
 * @exports set3DViewRotation - 3Dビューの回転角度設定
 * @exports reset3DView - 3Dビューのリセット
 * @exports select3DObject - オブジェクト選択
 * @exports deselect3DObject - オブジェクト選択解除
 * @exports goto2DMap - 2Dマップビューへ遷移
 * @exports gotoObjectCatalog - オブジェクトカタログへ遷移
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
import { mapState } from '../state/mapState.js';
import { getAllRectangles, getRectangleById, getRectangleLayer } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';
import { initializeViewCube, updateViewCube } from '../ui/viewCube.js';
import { analyzeMapBounds } from '../utils/imageProcessing.js';

// ================
// Three.js 3Dビュー状態
// ================

const view3DState = {
    // Three.js コアオブジェクト
    scene: null,
    camera: null,
    renderer: null,
    canvas: null,

    // ライティング
    ambientLight: null,
    directionalLight: null,

    // オブジェクト管理
    objectMeshes: new Map(), // rectangleId -> THREE.Group
    gridHelper: null,
    originMarker: null,
    floorPlane: null,

    // カメラ制御
    rotation: 45,           // 回転角度（度）
    tilt: 30,              // 傾き角度（度）
    distance: 20,          // カメラ距離
    scale: 1.0,            // ズームスケール
    offsetX: 0,            // パンX
    offsetY: 0,            // パンY

    // インタラクション
    isDragging: false,
    lastMouseX: 0,
    lastMouseY: 0,
    mouseDownX: 0,
    mouseDownY: 0,
    selectedObjectId: null,

    // マップ情報
    mapBounds: null,       // {minX, minY, maxX, maxY, centerX, centerY}

    // 状態フラグ
    isVisible: false,
    needsRender: true
};

// ================
// プレビュー状態（プロパティパネル用）
// ================

const previewState = {
    canvas: null,
    ctx: null,
    rotation: 45,
    tilt: 30,
    scale: 15
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

    // シーンを作成
    view3DState.scene = new THREE.Scene();
    view3DState.scene.background = new THREE.Color(0xf7fafc);

    // カメラを作成（OrthographicCamera for isometric projection）
    const aspect = canvas.clientWidth / canvas.clientHeight;
    const frustumSize = 20;
    view3DState.camera = new THREE.OrthographicCamera(
        frustumSize * aspect / -2,
        frustumSize * aspect / 2,
        frustumSize / 2,
        frustumSize / -2,
        0.1,
        1000
    );

    // カメラ位置を設定（等角投影のため）
    updateCameraPosition();

    // レンダラーを作成
    view3DState.renderer = new THREE.WebGLRenderer({
        canvas: canvas,
        antialias: true,
        alpha: false
    });
    view3DState.renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    view3DState.renderer.setPixelRatio(window.devicePixelRatio);
    view3DState.renderer.shadowMap.enabled = true;
    view3DState.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // ライティングを設定
    setupLighting();

    // グリッドと原点を作成
    createGrid();
    createOriginMarker();

    // 床面を作成
    createFloor();

    // イベントリスナーを設定
    setupEventListeners();

    // ViewCubeを初期化
    const viewCubeCanvas = document.getElementById('viewCubeCanvas');
    if (viewCubeCanvas) {
        initializeViewCube(viewCubeCanvas, handleViewCubeChange);
        console.log('ViewCubeを初期化しました');
    }

    console.log('3Dビュー（Three.js）を初期化しました');

    // 初期レンダリング
    render3DScene();
}

/**
 * カメラ位置を更新（rotation/tiltに基づく）
 *
 * @private
 */
function updateCameraPosition() {
    if (!view3DState.camera) return;

    const rad = view3DState.rotation * Math.PI / 180;
    const tiltRad = view3DState.tilt * Math.PI / 180;

    // カメラを遠くに配置
    const distance = view3DState.distance;
    const x = distance * Math.cos(rad) * Math.cos(tiltRad);
    const y = -distance * Math.sin(rad) * Math.cos(tiltRad);
    const z = distance * Math.sin(tiltRad);

    view3DState.camera.position.set(x, y, z);
    view3DState.camera.lookAt(view3DState.offsetX, view3DState.offsetY, 0);

    // OrthographicCameraのズームを設定
    view3DState.camera.zoom = view3DState.scale;
    view3DState.camera.updateProjectionMatrix();
}

/**
 * ライティングをセットアップ
 *
 * @private
 */
function setupLighting() {
    // 環境光（全体を明るく）
    view3DState.ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    view3DState.scene.add(view3DState.ambientLight);

    // 指向性ライト（影を作る）
    view3DState.directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    view3DState.directionalLight.position.set(10, 10, 15);
    view3DState.directionalLight.castShadow = true;

    // 影の設定
    view3DState.directionalLight.shadow.mapSize.width = 2048;
    view3DState.directionalLight.shadow.mapSize.height = 2048;
    view3DState.directionalLight.shadow.camera.left = -20;
    view3DState.directionalLight.shadow.camera.right = 20;
    view3DState.directionalLight.shadow.camera.top = 20;
    view3DState.directionalLight.shadow.camera.bottom = -20;
    view3DState.directionalLight.shadow.camera.near = 0.5;
    view3DState.directionalLight.shadow.camera.far = 50;

    view3DState.scene.add(view3DState.directionalLight);
}

/**
 * グリッドを作成
 *
 * @private
 */
function createGrid() {
    // 既存のグリッドを削除
    if (view3DState.gridHelper) {
        view3DState.scene.remove(view3DState.gridHelper);
    }

    // グリッドヘルパーを作成（Z=0平面）
    const size = 20;
    const divisions = 40;
    view3DState.gridHelper = new THREE.GridHelper(size, divisions, 0xcccccc, 0xe0e0e0);

    // GridHelperはデフォルトでY=0に作成されるため、Z=0に回転
    view3DState.gridHelper.rotation.x = Math.PI / 2;

    view3DState.scene.add(view3DState.gridHelper);
}

/**
 * 原点マーカーを作成
 *
 * @private
 */
function createOriginMarker() {
    if (view3DState.originMarker) {
        view3DState.scene.remove(view3DState.originMarker);
    }

    view3DState.originMarker = new THREE.Group();

    // 十字マーカー（赤）
    const crossSize = 0.15;
    const lineWidth = 0.02;
    const crossMaterial = new THREE.MeshBasicMaterial({ color: 0xe74c3c });

    // 4本の棒で十字を作成
    const createBar = (width, height, x, y) => {
        const geometry = new THREE.BoxGeometry(width, height, 0.001);
        const mesh = new THREE.Mesh(geometry, crossMaterial);
        mesh.position.set(x, y, 0.001);
        mesh.rotation.x = Math.PI / 2;
        return mesh;
    };

    view3DState.originMarker.add(createBar(crossSize, lineWidth, -crossSize/2, 0));
    view3DState.originMarker.add(createBar(crossSize, lineWidth, crossSize/2, 0));
    view3DState.originMarker.add(createBar(lineWidth, crossSize, 0, crossSize/2));
    view3DState.originMarker.add(createBar(lineWidth, crossSize, 0, -crossSize/2));

    // 方向矢印
    if (mapState.metadata?.origin) {
        const theta = Array.isArray(mapState.metadata.origin) && mapState.metadata.origin.length >= 3
            ? mapState.metadata.origin[2]
            : 0;

        const arrowLength = 0.5;
        const arrowDir = new THREE.Vector3(Math.cos(theta), Math.sin(theta), 0);
        const arrowHelper = new THREE.ArrowHelper(arrowDir, new THREE.Vector3(0, 0, 0.001), arrowLength, 0xe74c3c, 0.1, 0.08);
        view3DState.originMarker.add(arrowHelper);
    }

    view3DState.scene.add(view3DState.originMarker);
}

/**
 * 床面を作成（マップテクスチャ用）
 *
 * @private
 */
function createFloor() {
    if (view3DState.floorPlane) {
        view3DState.scene.remove(view3DState.floorPlane);
    }

    // マップ画像がある場合はテクスチャとして適用
    if (mapState.image) {
        const texture = new THREE.CanvasTexture(mapState.image);
        texture.minFilter = THREE.LinearFilter;
        texture.magFilter = THREE.LinearFilter;

        const material = new THREE.MeshStandardMaterial({
            map: texture,
            transparent: true,
            opacity: 0.8,
            side: THREE.DoubleSide
        });

        // マップのサイズを計算
        const resolution = mapState.metadata?.resolution || 0.05;
        const width = mapState.image.width * resolution;
        const height = mapState.image.height * resolution;

        const geometry = new THREE.PlaneGeometry(width, height);
        view3DState.floorPlane = new THREE.Mesh(geometry, material);
        view3DState.floorPlane.rotation.x = Math.PI / 2;
        view3DState.floorPlane.receiveShadow = true;

        // 原点位置を考慮
        const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
        const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;

        view3DState.floorPlane.position.set(
            originX + width / 2,
            originY + height / 2,
            0
        );

        view3DState.scene.add(view3DState.floorPlane);
    }
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
    view3DState.needsRender = true;
    render3DScene();
}

/**
 * イベントリスナーをセットアップ
 *
 * @private
 */
function setupEventListeners() {
    if (!view3DState.canvas) return;

    // マウスイベント（回転・パン）
    view3DState.canvas.addEventListener('mousedown', handle3DMouseDown);
    view3DState.canvas.addEventListener('mousemove', handle3DMouseMove);
    view3DState.canvas.addEventListener('mouseup', handle3DMouseUp);
    view3DState.canvas.addEventListener('mouseleave', handle3DMouseUp);

    // ホイールイベント（ズーム）
    view3DState.canvas.addEventListener('wheel', handle3DWheel);

    // ウィンドウリサイズ
    window.addEventListener('resize', resize3DView);
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
    if (!view3DState.renderer || !view3DState.scene || !view3DState.camera) {
        return;
    }

    // グリッドと原点の表示切替（2Dマップの設定に同期）
    if (view3DState.gridHelper) {
        view3DState.gridHelper.visible = mapState.overlaySettings?.showGrid || false;
    }
    if (view3DState.originMarker) {
        view3DState.originMarker.visible = mapState.overlaySettings?.showOrigin || false;
    }

    // すべての四角形を3Dで描画
    updateAllObjects();

    // レンダリング
    view3DState.renderer.render(view3DState.scene, view3DState.camera);

    // ViewCubeを更新
    updateViewCube(view3DState.rotation, view3DState.tilt);

    view3DState.needsRender = false;
}

/**
 * すべてのオブジェクトを更新
 *
 * @private
 */
function updateAllObjects() {
    const rectangleLayer = getRectangleLayer();
    if (!rectangleLayer || !rectangleLayer.visible) {
        // レイヤーが非表示の場合は全オブジェクトを非表示
        view3DState.objectMeshes.forEach(mesh => {
            mesh.visible = false;
        });
        return;
    }

    const rectangles = getAllRectangles();
    const currentIds = new Set(rectangles.map(r => r.id));

    // 削除されたオブジェクトをクリーンアップ
    view3DState.objectMeshes.forEach((mesh, id) => {
        if (!currentIds.has(id)) {
            view3DState.scene.remove(mesh);
            view3DState.objectMeshes.delete(id);
        }
    });

    // オブジェクトを更新または作成
    rectangles.forEach(rect => {
        update3DObject(rect.id);
    });
}

/**
 * 特定のオブジェクトを更新
 *
 * @param {string} rectangleId - 四角形ID
 * @returns {void}
 */
export function update3DObject(rectangleId) {
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) {
        // オブジェクトが削除された場合
        const mesh = view3DState.objectMeshes.get(rectangleId);
        if (mesh) {
            view3DState.scene.remove(mesh);
            view3DState.objectMeshes.delete(rectangleId);
        }
        return;
    }

    const coords3D = get3DCoordinates(rectangleId);
    if (!coords3D) return;

    // 既存のメッシュを削除
    const existingMesh = view3DState.objectMeshes.get(rectangleId);
    if (existingMesh) {
        view3DState.scene.remove(existingMesh);
    }

    // 新しいメッシュを作成
    const objectGroup = create3DObjectMesh(rectangle, coords3D);
    if (objectGroup) {
        view3DState.objectMeshes.set(rectangleId, objectGroup);
        view3DState.scene.add(objectGroup);
    }

    view3DState.needsRender = true;
}

/**
 * 3Dオブジェクトのメッシュを作成
 *
 * @private
 * @param {Object} rectangle - 四角形データ
 * @param {Object} coords3D - 3D座標データ
 * @returns {THREE.Group} オブジェクトグループ
 */
function create3DObjectMesh(rectangle, coords3D) {
    const group = new THREE.Group();
    const { x, y, z, width, depth, height } = coords3D;

    // オブジェクトタイプに応じた色
    const colorHex = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    const color = new THREE.Color(colorHex);

    // 選択状態の場合は明るくする
    const isSelected = view3DState.selectedObjectId === rectangle.id;
    if (isSelected) {
        color.offsetHSL(0, 0, 0.2);
    }

    // オブジェクトタイプに応じたジオメトリを作成
    let mesh;
    switch (rectangle.objectType) {
        case OBJECT_TYPES.SHELF:
            mesh = createShelfMesh(coords3D, color, rectangle.frontDirection, rectangle.objectProperties);
            break;
        case OBJECT_TYPES.BOX:
            mesh = createBoxMesh(coords3D, color, rectangle.frontDirection, rectangle.objectProperties);
            break;
        case OBJECT_TYPES.TABLE:
            mesh = createTableMesh(coords3D, color);
            break;
        case OBJECT_TYPES.DOOR:
            mesh = createDoorMesh(coords3D, color);
            break;
        case OBJECT_TYPES.WALL:
            mesh = createWallMesh(coords3D, color);
            break;
        case OBJECT_TYPES.NONE:
        default:
            mesh = createDefaultBoxMesh(coords3D, color);
            break;
    }

    if (mesh) {
        group.add(mesh);
    }

    // 前面方向の矢印を追加
    if (rectangle.objectType !== OBJECT_TYPES.NONE && rectangle.frontDirection) {
        const arrow = createFrontDirectionArrow(coords3D, rectangle.frontDirection);
        if (arrow) {
            group.add(arrow);
        }
    }

    // 選択時のハイライト枠を追加
    if (isSelected) {
        const outline = createSelectionOutline(coords3D);
        if (outline) {
            group.add(outline);
        }
    }

    group.position.set(x, y, z);

    return group;
}

/**
 * 棚のメッシュを作成
 *
 * @private
 */
function createShelfMesh(coords3D, color, frontDirection, objectProperties) {
    const { width, depth, height } = coords3D;
    const group = new THREE.Group();

    const material = new THREE.MeshStandardMaterial({
        color: color,
        roughness: 0.7,
        metalness: 0.1
    });

    // 外枠（前面を開けた箱）
    const thickness = 0.02;

    // 背面
    if (frontDirection !== 'bottom') {
        const backGeometry = new THREE.BoxGeometry(width, thickness, height);
        const backMesh = new THREE.Mesh(backGeometry, material);
        backMesh.position.set(0, depth/2, 0);
        backMesh.castShadow = true;
        backMesh.receiveShadow = true;
        group.add(backMesh);
    }

    // 前面
    if (frontDirection !== 'top') {
        const frontGeometry = new THREE.BoxGeometry(width, thickness, height);
        const frontMesh = new THREE.Mesh(frontGeometry, material);
        frontMesh.position.set(0, -depth/2, 0);
        frontMesh.castShadow = true;
        frontMesh.receiveShadow = true;
        group.add(frontMesh);
    }

    // 左面
    if (frontDirection !== 'left') {
        const leftGeometry = new THREE.BoxGeometry(thickness, depth, height);
        const leftMesh = new THREE.Mesh(leftGeometry, material);
        leftMesh.position.set(-width/2, 0, 0);
        leftMesh.castShadow = true;
        leftMesh.receiveShadow = true;
        group.add(leftMesh);
    }

    // 右面
    if (frontDirection !== 'right') {
        const rightGeometry = new THREE.BoxGeometry(thickness, depth, height);
        const rightMesh = new THREE.Mesh(rightGeometry, material);
        rightMesh.position.set(width/2, 0, 0);
        rightMesh.castShadow = true;
        rightMesh.receiveShadow = true;
        group.add(rightMesh);
    }

    // 上面・底面
    const topGeometry = new THREE.BoxGeometry(width, depth, thickness);
    const topMesh = new THREE.Mesh(topGeometry, material);
    topMesh.position.set(0, 0, height/2);
    topMesh.castShadow = true;
    topMesh.receiveShadow = true;
    group.add(topMesh);

    const bottomGeometry = new THREE.BoxGeometry(width, depth, thickness);
    const bottomMesh = new THREE.Mesh(bottomGeometry, material);
    bottomMesh.position.set(0, 0, -height/2);
    bottomMesh.castShadow = true;
    bottomMesh.receiveShadow = true;
    group.add(bottomMesh);

    // 棚板を追加
    if (objectProperties?.shelfLevels) {
        const levels = objectProperties.shelfLevels;
        const shelfMaterial = new THREE.MeshStandardMaterial({
            color: color.clone().offsetHSL(0, 0, -0.1),
            roughness: 0.7,
            metalness: 0.1
        });

        for (let i = 1; i < levels; i++) {
            const levelZ = -height/2 + (height / levels) * i;
            const shelfGeometry = new THREE.BoxGeometry(width - thickness*2, depth - thickness*2, thickness);
            const shelfMesh = new THREE.Mesh(shelfGeometry, shelfMaterial);
            shelfMesh.position.set(0, 0, levelZ);
            shelfMesh.castShadow = true;
            shelfMesh.receiveShadow = true;
            group.add(shelfMesh);
        }
    }

    return group;
}

/**
 * 箱のメッシュを作成
 *
 * @private
 */
function createBoxMesh(coords3D, color, frontDirection, objectProperties) {
    const { width, depth, height } = coords3D;
    const group = new THREE.Group();

    const material = new THREE.MeshStandardMaterial({
        color: color,
        roughness: 0.8,
        metalness: 0.05
    });

    const thickness = 0.02;

    // 5面の箱（上部が開いている）
    // 底面
    const bottomGeometry = new THREE.BoxGeometry(width, depth, thickness);
    const bottomMesh = new THREE.Mesh(bottomGeometry, material);
    bottomMesh.position.set(0, 0, -height/2);
    bottomMesh.castShadow = true;
    bottomMesh.receiveShadow = true;
    group.add(bottomMesh);

    // 4つの側面
    const frontGeometry = new THREE.BoxGeometry(width, thickness, height);
    const frontMesh = new THREE.Mesh(frontGeometry, material);
    frontMesh.position.set(0, -depth/2, 0);
    frontMesh.castShadow = true;
    frontMesh.receiveShadow = true;
    group.add(frontMesh);

    const backGeometry = new THREE.BoxGeometry(width, thickness, height);
    const backMesh = new THREE.Mesh(backGeometry, material);
    backMesh.position.set(0, depth/2, 0);
    backMesh.castShadow = true;
    backMesh.receiveShadow = true;
    group.add(backMesh);

    const leftGeometry = new THREE.BoxGeometry(thickness, depth, height);
    const leftMesh = new THREE.Mesh(leftGeometry, material);
    leftMesh.position.set(-width/2, 0, 0);
    leftMesh.castShadow = true;
    leftMesh.receiveShadow = true;
    group.add(leftMesh);

    const rightGeometry = new THREE.BoxGeometry(thickness, depth, height);
    const rightMesh = new THREE.Mesh(rightGeometry, material);
    rightMesh.position.set(width/2, 0, 0);
    rightMesh.castShadow = true;
    rightMesh.receiveShadow = true;
    group.add(rightMesh);

    return group;
}

/**
 * テーブルのメッシュを作成
 *
 * @private
 */
function createTableMesh(coords3D, color) {
    const { width, depth, height } = coords3D;
    const group = new THREE.Group();

    const material = new THREE.MeshStandardMaterial({
        color: color,
        roughness: 0.6,
        metalness: 0.2
    });

    const topThickness = height * 0.1;
    const legWidth = Math.min(width, depth) * 0.1;
    const legHeight = height - topThickness;

    // 天板
    const topGeometry = new THREE.BoxGeometry(width, depth, topThickness);
    const topMesh = new THREE.Mesh(topGeometry, material);
    topMesh.position.set(0, 0, height/2 - topThickness/2);
    topMesh.castShadow = true;
    topMesh.receiveShadow = true;
    group.add(topMesh);

    // 4本の脚
    const legGeometry = new THREE.BoxGeometry(legWidth, legWidth, legHeight);
    const legPositions = [
        [-width/2 + legWidth/2, -depth/2 + legWidth/2],
        [width/2 - legWidth/2, -depth/2 + legWidth/2],
        [width/2 - legWidth/2, depth/2 - legWidth/2],
        [-width/2 + legWidth/2, depth/2 - legWidth/2]
    ];

    legPositions.forEach(([x, y]) => {
        const legMesh = new THREE.Mesh(legGeometry, material);
        legMesh.position.set(x, y, -height/2 + legHeight/2);
        legMesh.castShadow = true;
        legMesh.receiveShadow = true;
        group.add(legMesh);
    });

    return group;
}

/**
 * 扉のメッシュを作成
 *
 * @private
 */
function createDoorMesh(coords3D, color) {
    const { width, depth, height } = coords3D;

    const material = new THREE.MeshStandardMaterial({
        color: color,
        roughness: 0.5,
        metalness: 0.3
    });

    // 薄い直方体
    const geometry = new THREE.BoxGeometry(width, Math.max(depth, 0.05), height);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    return mesh;
}

/**
 * 壁のメッシュを作成
 *
 * @private
 */
function createWallMesh(coords3D, color) {
    const { width, depth, height } = coords3D;

    const material = new THREE.MeshStandardMaterial({
        color: color,
        roughness: 0.9,
        metalness: 0.0
    });

    const geometry = new THREE.BoxGeometry(width, depth, height);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    return mesh;
}

/**
 * デフォルトの箱メッシュを作成
 *
 * @private
 */
function createDefaultBoxMesh(coords3D, color) {
    const { width, depth, height } = coords3D;

    const material = new THREE.MeshStandardMaterial({
        color: color,
        roughness: 0.7,
        metalness: 0.1,
        transparent: true,
        opacity: 0.8
    });

    const geometry = new THREE.BoxGeometry(width, depth, height);
    const mesh = new THREE.Mesh(geometry, material);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    return mesh;
}

/**
 * 前面方向の矢印を作成
 *
 * @private
 */
function createFrontDirectionArrow(coords3D, frontDirection) {
    if (!frontDirection) return null;

    const { width, depth, height } = coords3D;
    const arrowLength = Math.max(width, depth) * 0.3;

    let direction, origin;
    switch (frontDirection) {
        case 'top':
            direction = new THREE.Vector3(0, -1, 0);
            origin = new THREE.Vector3(0, depth/2, height/2 + 0.1);
            break;
        case 'bottom':
            direction = new THREE.Vector3(0, 1, 0);
            origin = new THREE.Vector3(0, -depth/2, height/2 + 0.1);
            break;
        case 'left':
            direction = new THREE.Vector3(-1, 0, 0);
            origin = new THREE.Vector3(-width/2, 0, height/2 + 0.1);
            break;
        case 'right':
            direction = new THREE.Vector3(1, 0, 0);
            origin = new THREE.Vector3(width/2, 0, height/2 + 0.1);
            break;
        default:
            return null;
    }

    const arrow = new THREE.ArrowHelper(
        direction,
        origin,
        arrowLength,
        0x00ff00,
        arrowLength * 0.2,
        arrowLength * 0.15
    );

    return arrow;
}

/**
 * 選択時のアウトラインを作成
 *
 * @private
 */
function createSelectionOutline(coords3D) {
    const { width, depth, height } = coords3D;

    const geometry = new THREE.BoxGeometry(width + 0.05, depth + 0.05, height + 0.05);
    const edges = new THREE.EdgesGeometry(geometry);
    const line = new THREE.LineSegments(
        edges,
        new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 2 })
    );

    return line;
}

// ================
// オブジェクト選択
// ================

/**
 * オブジェクトを選択
 *
 * @param {string} objectId - オブジェクトID
 * @returns {void}
 */
export function select3DObject(objectId) {
    if (view3DState.selectedObjectId === objectId) return;

    // 前の選択を解除
    deselect3DObject();

    view3DState.selectedObjectId = objectId;
    update3DObject(objectId);
    view3DState.needsRender = true;
    render3DScene();
}

/**
 * オブジェクトの選択を解除
 *
 * @returns {void}
 */
export function deselect3DObject() {
    if (!view3DState.selectedObjectId) return;

    const previousId = view3DState.selectedObjectId;
    view3DState.selectedObjectId = null;
    update3DObject(previousId);
    view3DState.needsRender = true;
    render3DScene();
}

// ================
// イベントハンドラ
// ================

/**
 * マウスダウンハンドラ
 *
 * @private
 */
function handle3DMouseDown(event) {
    view3DState.isDragging = true;
    view3DState.lastMouseX = event.clientX;
    view3DState.lastMouseY = event.clientY;
    view3DState.mouseDownX = event.clientX;
    view3DState.mouseDownY = event.clientY;
}

/**
 * マウスムーブハンドラ
 *
 * @private
 */
function handle3DMouseMove(event) {
    if (!view3DState.isDragging) return;

    const deltaX = event.clientX - view3DState.lastMouseX;
    const deltaY = event.clientY - view3DState.lastMouseY;

    // 回転
    view3DState.rotation += deltaX * 0.5;
    view3DState.tilt = Math.max(-89, Math.min(89, view3DState.tilt - deltaY * 0.5));

    updateCameraPosition();
    view3DState.needsRender = true;
    render3DScene();

    view3DState.lastMouseX = event.clientX;
    view3DState.lastMouseY = event.clientY;
}

/**
 * マウスアップハンドラ
 *
 * @private
 */
function handle3DMouseUp(event) {
    if (!view3DState.isDragging) return;

    view3DState.isDragging = false;

    // クリック判定（ドラッグでない場合）
    const dragDistance = Math.sqrt(
        Math.pow(event.clientX - view3DState.mouseDownX, 2) +
        Math.pow(event.clientY - view3DState.mouseDownY, 2)
    );

    if (dragDistance < 5) {
        // クリックとして処理（オブジェクト選択）
        handleClick(event);
    }
}

/**
 * クリックハンドラ（オブジェクト選択）
 *
 * @private
 */
function handleClick(event) {
    // Raycasterでオブジェクトをピック
    const rect = view3DState.canvas.getBoundingClientRect();
    const mouse = new THREE.Vector2(
        ((event.clientX - rect.left) / rect.width) * 2 - 1,
        -((event.clientY - rect.top) / rect.height) * 2 + 1
    );

    const raycaster = new THREE.Raycaster();
    raycaster.setFromCamera(mouse, view3DState.camera);

    // すべてのオブジェクトメッシュをチェック
    const intersectables = [];
    view3DState.objectMeshes.forEach((group, id) => {
        group.traverse(obj => {
            if (obj.isMesh) {
                obj.userData.rectangleId = id;
                intersectables.push(obj);
            }
        });
    });

    const intersects = raycaster.intersectObjects(intersectables, false);

    if (intersects.length > 0) {
        const rectangleId = intersects[0].object.userData.rectangleId;
        if (rectangleId) {
            // 2Dビューの選択と同期
            const rectangle = getRectangleById(rectangleId);
            if (rectangle) {
                // rectangleManager経由で選択（2Dビューと同期）
                import('../modules/rectangleManager.js').then(module => {
                    module.selectRectangle(rectangleId);
                });
            }
        }
    } else {
        // 何もない場所をクリックした場合は選択解除
        deselect3DObject();
    }
}

/**
 * マウスホイールハンドラ（ズーム）
 *
 * @private
 */
function handle3DWheel(event) {
    event.preventDefault();

    const delta = event.deltaY > 0 ? 0.9 : 1.1;
    view3DState.scale *= delta;
    view3DState.scale = Math.max(0.1, Math.min(10, view3DState.scale));

    updateCameraPosition();
    view3DState.needsRender = true;
    render3DScene();
}

// ================
// ビューコントロール
// ================

/**
 * 3Dビューのリサイズ
 *
 * @returns {void}
 */
export function resize3DView() {
    if (!view3DState.canvas || !view3DState.camera || !view3DState.renderer) return;

    const width = view3DState.canvas.clientWidth;
    const height = view3DState.canvas.clientHeight;

    // カメラのアスペクト比を更新
    const aspect = width / height;
    const frustumSize = 20;
    view3DState.camera.left = frustumSize * aspect / -2;
    view3DState.camera.right = frustumSize * aspect / 2;
    view3DState.camera.top = frustumSize / 2;
    view3DState.camera.bottom = frustumSize / -2;
    view3DState.camera.updateProjectionMatrix();

    // レンダラーのサイズを更新
    view3DState.renderer.setSize(width, height);
    view3DState.renderer.setPixelRatio(window.devicePixelRatio);

    view3DState.needsRender = true;
    render3DScene();
}

/**
 * 3Dビューの回転角度を設定
 *
 * @param {number} rotation - 回転角度（度）
 * @param {number} tilt - 傾き角度（度）
 * @returns {void}
 */
export function set3DViewRotation(rotation, tilt) {
    view3DState.rotation = rotation;
    view3DState.tilt = tilt;
    updateCameraPosition();
    view3DState.needsRender = true;
    render3DScene();
}

/**
 * 3Dビューをリセット
 *
 * @returns {void}
 */
export function reset3DView() {
    view3DState.rotation = 45;
    view3DState.tilt = 30;
    view3DState.scale = 1.0;
    view3DState.offsetX = 0;
    view3DState.offsetY = 0;
    updateCameraPosition();
    view3DState.needsRender = true;
    render3DScene();
}

// ================
// マップ境界
// ================

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
 * @param {Object} bounds - マップ境界 {minX, minY, maxX, maxY, centerX, centerY}
 * @returns {void}
 */
export function setMapBounds(bounds) {
    view3DState.mapBounds = bounds;

    // 床面を更新
    createFloor();

    view3DState.needsRender = true;
    render3DScene();
}

// ================
// ナビゲーション
// ================

/**
 * 2Dマップビューへ遷移
 *
 * @returns {void}
 */
export function goto2DMap() {
    // サブタブを2Dマップに切り替え
    const map2DButton = document.querySelector('.map-subtab-button[onclick*="map2D"]');
    if (map2DButton) {
        map2DButton.click();
    }
}

/**
 * オブジェクトカタログへ遷移
 *
 * @param {string} objectId - オブジェクトID
 * @returns {void}
 */
export function gotoObjectCatalog(objectId = null) {
    // オブジェクトカタログタブに切り替え
    const catalogButton = document.querySelector('.tab-button[onclick*="objectCatalog"]');
    if (catalogButton) {
        catalogButton.click();
    }

    // 特定のオブジェクトを選択
    if (objectId) {
        setTimeout(() => {
            const objectItem = document.querySelector(`[data-object-id="${objectId}"]`);
            if (objectItem) {
                objectItem.scrollIntoView({ behavior: 'smooth', block: 'center' });
                objectItem.click();
            }
        }, 100);
    }
}

// ================
// プレビュー（プロパティパネル用）
// ================

/**
 * プロパティプレビューを初期化
 *
 * @returns {void}
 */
export function initializePropertyPreview() {
    const canvas = document.getElementById('propertyPreviewCanvas');
    if (!canvas) {
        console.error('initializePropertyPreview: プレビューCanvas要素が見つかりません');
        return;
    }

    previewState.canvas = canvas;
    previewState.ctx = canvas.getContext('2d');

    // Canvasサイズを設定
    canvas.width = 200;
    canvas.height = 200;

    console.log('プロパティプレビューを初期化しました');
}

/**
 * プロパティプレビューを描画
 *
 * @param {string} rectangleId - 四角形ID
 * @returns {void}
 */
export function renderPropertyPreview(rectangleId) {
    if (!previewState.ctx || !previewState.canvas) return;

    const ctx = previewState.ctx;
    const width = previewState.canvas.width;
    const height = previewState.canvas.height;

    // キャンバスをクリア
    ctx.clearRect(0, 0, width, height);
    ctx.fillStyle = '#f7fafc';
    ctx.fillRect(0, 0, width, height);

    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return;

    const coords3D = get3DCoordinates(rectangleId);
    if (!coords3D) return;

    const centerX = width / 2;
    const centerY = height / 2;

    // オブジェクトタイプに応じた色
    const colorHex = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    // プレビュー用の簡易3D描画（Canvas 2Dを使用）
    drawPreviewModel(ctx, coords3D, colorHex, centerX, centerY, rectangle.objectType, rectangle.frontDirection, rectangle.objectProperties);

    // 前面方向矢印
    if (rectangle.objectType !== OBJECT_TYPES.NONE) {
        drawPreviewFrontDirection(ctx, coords3D, rectangle.frontDirection, centerX, centerY);
    }
}

/**
 * プレビュー用3Dモデルを描画（Canvas 2D使用）
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標
 * @param {string} color - 色
 * @param {number} centerX - 中心X
 * @param {number} centerY - 中心Y
 * @param {string} objectType - オブジェクトタイプ
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 * @returns {void}
 */
export function drawPreviewModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
    // 簡易的なボックス描画（等角投影風）
    const { width, depth, height } = coords3D;

    // プレビュー用の等角投影変換
    const scale = previewState.scale;

    const vertices = [
        worldToPreviewIso(-width/2, -depth/2, -height/2),
        worldToPreviewIso(width/2, -depth/2, -height/2),
        worldToPreviewIso(width/2, depth/2, -height/2),
        worldToPreviewIso(-width/2, depth/2, -height/2),
        worldToPreviewIso(-width/2, -depth/2, height/2),
        worldToPreviewIso(width/2, -depth/2, height/2),
        worldToPreviewIso(width/2, depth/2, height/2),
        worldToPreviewIso(-width/2, depth/2, height/2),
    ];

    const screen = vertices.map(v => ({
        x: centerX + v.x * scale,
        y: centerY - v.y * scale
    }));

    ctx.save();

    // 色の調整用ヘルパー
    const lighten = (col, percent) => {
        const num = parseInt(col.replace("#",""), 16);
        const amt = Math.round(2.55 * percent);
        const R = Math.min(255, (num >> 16) + amt);
        const G = Math.min(255, (num >> 8 & 0x00FF) + amt);
        const B = Math.min(255, (num & 0x0000FF) + amt);
        return "#" + (0x1000000 + R * 0x10000 + G * 0x100 + B).toString(16).slice(1);
    };

    const darken = (col, percent) => {
        const num = parseInt(col.replace("#",""), 16);
        const amt = Math.round(2.55 * percent);
        const R = Math.max(0, (num >> 16) - amt);
        const G = Math.max(0, (num >> 8 & 0x00FF) - amt);
        const B = Math.max(0, (num & 0x0000FF) - amt);
        return "#" + (0x1000000 + R * 0x10000 + G * 0x100 + B).toString(16).slice(1);
    };

    // 上面
    ctx.fillStyle = lighten(color, 20);
    ctx.beginPath();
    ctx.moveTo(screen[4].x, screen[4].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[7].x, screen[7].y);
    ctx.closePath();
    ctx.fill();
    ctx.strokeStyle = darken(color, 20);
    ctx.lineWidth = 1;
    ctx.stroke();

    // 右面
    ctx.fillStyle = color;
    ctx.beginPath();
    ctx.moveTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[6].x, screen[6].y);
    ctx.lineTo(screen[2].x, screen[2].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    // 前面
    ctx.fillStyle = darken(color, 10);
    ctx.beginPath();
    ctx.moveTo(screen[0].x, screen[0].y);
    ctx.lineTo(screen[1].x, screen[1].y);
    ctx.lineTo(screen[5].x, screen[5].y);
    ctx.lineTo(screen[4].x, screen[4].y);
    ctx.closePath();
    ctx.fill();
    ctx.stroke();

    ctx.restore();
}

/**
 * プレビュー用前面方向矢印を描画
 *
 * @param {CanvasRenderingContext2D} ctx - 描画コンテキスト
 * @param {Object} coords3D - 3D座標
 * @param {string} frontDirection - 前面方向
 * @param {number} centerX - 中心X
 * @param {number} centerY - 中心Y
 * @returns {void}
 */
export function drawPreviewFrontDirection(ctx, coords3D, frontDirection, centerX, centerY) {
    if (!frontDirection) return;

    const { width, depth, height } = coords3D;
    const scale = previewState.scale;

    let arrowStart, arrowEnd;
    switch (frontDirection) {
        case 'top':
            arrowStart = worldToPreviewIso(0, depth/2, height/2);
            arrowEnd = worldToPreviewIso(0, depth/2 + 0.3, height/2);
            break;
        case 'bottom':
            arrowStart = worldToPreviewIso(0, -depth/2, height/2);
            arrowEnd = worldToPreviewIso(0, -depth/2 - 0.3, height/2);
            break;
        case 'left':
            arrowStart = worldToPreviewIso(-width/2, 0, height/2);
            arrowEnd = worldToPreviewIso(-width/2 - 0.3, 0, height/2);
            break;
        case 'right':
            arrowStart = worldToPreviewIso(width/2, 0, height/2);
            arrowEnd = worldToPreviewIso(width/2 + 0.3, 0, height/2);
            break;
        default:
            return;
    }

    const startScreen = {
        x: centerX + arrowStart.x * scale,
        y: centerY - arrowStart.y * scale
    };
    const endScreen = {
        x: centerX + arrowEnd.x * scale,
        y: centerY - arrowEnd.y * scale
    };

    // 矢印を描画
    ctx.save();
    ctx.strokeStyle = '#00ff00';
    ctx.fillStyle = '#00ff00';
    ctx.lineWidth = 2;

    ctx.beginPath();
    ctx.moveTo(startScreen.x, startScreen.y);
    ctx.lineTo(endScreen.x, endScreen.y);
    ctx.stroke();

    // 矢印の先端
    const angle = Math.atan2(endScreen.y - startScreen.y, endScreen.x - startScreen.x);
    const headLength = 8;

    ctx.beginPath();
    ctx.moveTo(endScreen.x, endScreen.y);
    ctx.lineTo(
        endScreen.x - headLength * Math.cos(angle - Math.PI/6),
        endScreen.y - headLength * Math.sin(angle - Math.PI/6)
    );
    ctx.lineTo(
        endScreen.x - headLength * Math.cos(angle + Math.PI/6),
        endScreen.y - headLength * Math.sin(angle + Math.PI/6)
    );
    ctx.closePath();
    ctx.fill();

    ctx.restore();
}

/**
 * プレビュー用等角投影変換
 *
 * @param {number} x - X座標
 * @param {number} y - Y座標
 * @param {number} z - Z座標
 * @returns {Object} 変換後の座標 {x, y}
 */
export function worldToPreviewIso(x, y, z) {
    const rad = previewState.rotation * Math.PI / 180;
    const tiltRad = previewState.tilt * Math.PI / 180;

    const rotX = x * Math.cos(rad) - y * Math.sin(rad);
    const rotY = x * Math.sin(rad) + y * Math.cos(rad);

    return {
        x: -rotX,
        y: z - rotY * Math.sin(tiltRad)
    };
}

/**
 * プレビュー状態を取得
 *
 * @returns {Object} プレビュー状態
 */
export function getPreviewState() {
    return previewState;
}
