/**
 * @file threeDRendererThree.js
 * @description Three.jsを使用した3D描画システム
 *
 * 既存のthreeDRenderer.jsと同じAPIを提供しつつ、
 * 内部実装をThree.jsに移行したバージョン。
 *
 * @requires three - Three.js本体
 * @requires three/addons/controls/OrbitControls.js - カメラコントロール
 * @requires ../state/mapState.js - アプリケーション状態管理
 * @requires ../modules/rectangleManager.js - 四角形管理
 * @requires ../modules/objectPropertyManager.js - プロパティ管理
 * @requires ../models/objectTypes.js - オブジェクトタイプ定義
 *
 * @exports initialize3DView - 3Dビュー初期化
 * @exports render3DScene - 3Dシーン描画
 * @exports select3DObject - オブジェクト選択
 * @exports deselect3DObject - 選択解除
 * @exports update3DObject - オブジェクト更新
 * @exports resize3DView - ビューリサイズ
 * @exports set3DViewRotation - 回転設定
 * @exports reset3DView - ビューリセット
 * @exports initializePropertyPreview - プレビュー初期化
 * @exports renderPropertyPreview - プレビュー描画
 */

import * as THREE from 'three';
import { OrbitControls } from 'three/addons/controls/OrbitControls.js';
import { mapState } from '../state/mapState.js';
import { getAllRectangles, getRectangleById, getRectangleLayer } from '../modules/rectangleManager.js';
import { get3DCoordinates } from '../modules/objectPropertyManager.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';

// ================
// 3Dビュー状態
// ================

const view3DState = {
    // Three.js コアオブジェクト
    scene: null,
    camera: null,
    renderer: null,
    controls: null,

    // Canvas要素
    canvas: null,

    // オブジェクト管理
    objectMeshes: new Map(),  // rectangleId -> Mesh のマッピング
    selectedObjectId: null,

    // 補助要素
    gridHelper: null,
    originMarker: null,
    mapMesh: null,

    // 状態フラグ
    isVisible: false,
    mapBounds: null,

    // デフォルトカメラ位置
    defaultCameraPosition: { x: 10, y: 10, z: 10 },
    defaultCameraTarget: { x: 0, y: 0, z: 0 },
};

// ================
// プレビュー状態
// ================

const previewState = {
    scene: null,
    camera: null,
    renderer: null,
    canvas: null,
    objectMesh: null,
    gridHelper: null,
    arrowHelper: null,
    isInitialized: false,
};

// ================
// 座標系設定
// ================

/**
 * 座標系の設定と変換
 *
 * 【アプリケーションの座標系】(元の実装)
 * - X軸: 右方向が正
 * - Y軸: 奥方向が正（画面の下方向）
 * - Z軸: 上方向が正
 *
 * 【Three.jsの座標系】(デフォルト)
 * - X軸: 右方向が正
 * - Y軸: 上方向が正
 * - Z軸: 手前方向が正（カメラ向き、奥がマイナス）
 *
 * 【座標変換ルール】
 * アプリ(x, y, z) → Three.js(x, z, -y)
 * - X: そのまま
 * - Y(奥) → -Z(Three.js)  ※奥方向がマイナスZ
 * - Z(上) → Y(Three.js)
 */

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

    // Three.js のシーンを作成
    view3DState.scene = new THREE.Scene();
    view3DState.scene.background = new THREE.Color(0xf7fafc);

    // カメラを作成（PerspectiveCamera）
    const aspect = canvas.clientWidth / canvas.clientHeight;
    view3DState.camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 1000);
    view3DState.camera.position.set(
        view3DState.defaultCameraPosition.x,
        view3DState.defaultCameraPosition.y,
        view3DState.defaultCameraPosition.z
    );
    view3DState.camera.lookAt(
        view3DState.defaultCameraTarget.x,
        view3DState.defaultCameraTarget.y,
        view3DState.defaultCameraTarget.z
    );

    // レンダラーを作成
    view3DState.renderer = new THREE.WebGLRenderer({
        canvas: canvas,
        antialias: true,
        alpha: true,
    });
    view3DState.renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    view3DState.renderer.setPixelRatio(window.devicePixelRatio);
    view3DState.renderer.shadowMap.enabled = true;
    view3DState.renderer.shadowMap.type = THREE.PCFSoftShadowMap;

    // OrbitControls を作成
    view3DState.controls = new OrbitControls(view3DState.camera, canvas);
    view3DState.controls.enableDamping = true;
    view3DState.controls.dampingFactor = 0.05;
    view3DState.controls.screenSpacePanning = false;
    view3DState.controls.minDistance = 1;
    view3DState.controls.maxDistance = 100;
    view3DState.controls.maxPolarAngle = Math.PI / 2; // 地面より下に行かない

    // ライトを追加
    addLights();

    // グリッドを追加（初期状態）
    if (mapState.overlaySettings.showGrid) {
        addGrid();
    }

    // 原点マーカーを追加（初期状態）
    if (mapState.overlaySettings.showOrigin) {
        addOriginMarker();
    }

    // リサイズイベント
    window.addEventListener('resize', handleResize);

    // アニメーションループを開始
    animate();

    view3DState.isVisible = true;

    console.log('3Dビュー（Three.js版）を初期化しました');
}

/**
 * ライトをシーンに追加
 * @private
 */
function addLights() {
    // 環境光
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    view3DState.scene.add(ambientLight);

    // 指向性ライト（太陽光のような）
    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(10, 20, 10);
    directionalLight.castShadow = true;
    directionalLight.shadow.mapSize.width = 2048;
    directionalLight.shadow.mapSize.height = 2048;
    directionalLight.shadow.camera.near = 0.5;
    directionalLight.shadow.camera.far = 50;
    directionalLight.shadow.camera.left = -20;
    directionalLight.shadow.camera.right = 20;
    directionalLight.shadow.camera.top = 20;
    directionalLight.shadow.camera.bottom = -20;
    view3DState.scene.add(directionalLight);
}

/**
 * グリッドをシーンに追加
 * @private
 */
function addGrid() {
    if (view3DState.gridHelper) {
        view3DState.scene.remove(view3DState.gridHelper);
    }

    const gridSize = mapState.gridWidthInMeters || 1; // m
    const gridDivisions = 20;

    view3DState.gridHelper = new THREE.GridHelper(
        gridSize * gridDivisions,
        gridDivisions,
        0xcbd5e0,
        0xcbd5e0
    );
    view3DState.gridHelper.position.y = 0; // 床面
    view3DState.scene.add(view3DState.gridHelper);
}

/**
 * 原点マーカーをシーンに追加
 * @private
 */
function addOriginMarker() {
    if (view3DState.originMarker) {
        view3DState.scene.remove(view3DState.originMarker);
    }

    const markerGroup = new THREE.Group();

    // 軸ヘルパー
    const axesHelper = new THREE.AxesHelper(0.5);
    markerGroup.add(axesHelper);

    // 赤い十字マーカー
    const crossGeometry = new THREE.BoxGeometry(0.3, 0.02, 0.02);
    const crossMaterial = new THREE.MeshBasicMaterial({ color: 0xe74c3c });
    const crossX = new THREE.Mesh(crossGeometry, crossMaterial);
    markerGroup.add(crossX);

    const crossZ = new THREE.Mesh(new THREE.BoxGeometry(0.02, 0.02, 0.3), crossMaterial);
    markerGroup.add(crossZ);

    // マップメタデータの原点の向きに応じた矢印
    if (mapState.metadata && mapState.metadata.origin) {
        const theta = Array.isArray(mapState.metadata.origin) && mapState.metadata.origin.length >= 3
            ? mapState.metadata.origin[2]
            : 0;

        const arrowDir = new THREE.Vector3(Math.cos(theta), 0, Math.sin(theta));
        const arrowHelper = new THREE.ArrowHelper(arrowDir, new THREE.Vector3(0, 0, 0), 0.5, 0xe74c3c, 0.1, 0.08);
        markerGroup.add(arrowHelper);
    }

    markerGroup.position.set(0, 0.01, 0); // 床面の少し上
    view3DState.originMarker = markerGroup;
    view3DState.scene.add(markerGroup);
}

/**
 * ウィンドウリサイズ処理
 * @private
 */
function handleResize() {
    if (!view3DState.renderer || !view3DState.camera || !view3DState.canvas) return;

    const width = view3DState.canvas.clientWidth;
    const height = view3DState.canvas.clientHeight;

    view3DState.camera.aspect = width / height;
    view3DState.camera.updateProjectionMatrix();

    view3DState.renderer.setSize(width, height);
}

/**
 * アニメーションループ
 * @private
 */
function animate() {
    requestAnimationFrame(animate);

    if (view3DState.controls) {
        view3DState.controls.update();
    }

    if (view3DState.renderer && view3DState.scene && view3DState.camera) {
        view3DState.renderer.render(view3DState.scene, view3DState.camera);
    }
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
    if (!view3DState.scene) return;

    // 既存のオブジェクトメッシュをクリア
    view3DState.objectMeshes.forEach((mesh) => {
        view3DState.scene.remove(mesh);
    });
    view3DState.objectMeshes.clear();

    // グリッドの表示/非表示
    if (mapState.overlaySettings.showGrid && !view3DState.gridHelper) {
        addGrid();
    } else if (!mapState.overlaySettings.showGrid && view3DState.gridHelper) {
        view3DState.scene.remove(view3DState.gridHelper);
        view3DState.gridHelper = null;
    }

    // 原点マーカーの表示/非表示
    if (mapState.overlaySettings.showOrigin && !view3DState.originMarker) {
        addOriginMarker();
    } else if (!mapState.overlaySettings.showOrigin && view3DState.originMarker) {
        view3DState.scene.remove(view3DState.originMarker);
        view3DState.originMarker = null;
    }

    // マップテクスチャの床面への適用
    if (mapState.image && !view3DState.mapMesh) {
        addMapTexture();
    }

    // すべての四角形を3Dで描画（レイヤー可視性を反映）
    const rectangleLayer = getRectangleLayer();
    if (rectangleLayer && rectangleLayer.visible) {
        const rectangles = getAllRectangles();
        rectangles.forEach(rect => {
            const mesh = create3DObjectMesh(rect);
            if (mesh) {
                view3DState.objectMeshes.set(rect.id, mesh);
                view3DState.scene.add(mesh);
            }
        });
    }
}

/**
 * マップテクスチャを床面に追加
 * @private
 */
function addMapTexture() {
    if (!mapState.image) return;

    // 既存のマップメッシュを削除
    if (view3DState.mapMesh) {
        view3DState.scene.remove(view3DState.mapMesh);
    }

    const resolution = mapState.metadata?.resolution || 0.05; // m/pixel
    const imageWidth = mapState.image.width;
    const imageHeight = mapState.image.height;

    // 実世界サイズ（メートル）
    const realWidth = imageWidth * resolution;
    const realDepth = imageHeight * resolution;

    // テクスチャを作成
    const texture = new THREE.Texture(mapState.image);
    texture.needsUpdate = true;
    texture.minFilter = THREE.LinearFilter;
    texture.magFilter = THREE.LinearFilter;

    // 床面の平面ジオメトリ (XZ平面)
    const geometry = new THREE.PlaneGeometry(realWidth, realDepth);
    const material = new THREE.MeshBasicMaterial({
        map: texture,
        side: THREE.DoubleSide,
        transparent: true,
        opacity: 0.8,
    });

    const mesh = new THREE.Mesh(geometry, material);

    // PlaneGeometryはデフォルトでXY平面なので、XZ平面に回転
    mesh.rotation.x = -Math.PI / 2;

    // ROSマップの原点を取得（マップ左下の実世界座標）
    const originX = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[0] : 0;
    const originY = Array.isArray(mapState.metadata?.origin) ? mapState.metadata.origin[1] : 0;

    // マップ中心の実世界座標を計算
    // ROSマップは左下が原点、画像は左上が原点
    const centerWorldX = originX + realWidth / 2;
    const centerWorldY = originY + realDepth / 2;

    // 座標変換: アプリ(x, y, 0) → Three.js(x, 0, -y)
    // 床面なのでY=0（高さ）
    mesh.position.set(centerWorldX, 0, -centerWorldY);

    mesh.receiveShadow = true;

    view3DState.mapMesh = mesh;
    view3DState.scene.add(mesh);
}

/**
 * 3Dオブジェクトメッシュを作成
 * @private
 * @param {Object} rectangle - 四角形オブジェクト
 * @returns {THREE.Group|null} メッシュグループ
 */
function create3DObjectMesh(rectangle) {
    const coords3D = get3DCoordinates(rectangle.id);
    if (!coords3D) return null;

    const group = new THREE.Group();

    // オブジェクトタイプに応じた色
    let color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    // 選択されている場合は色を明るくする
    const isSelected = view3DState.selectedObjectId === rectangle.id;
    if (isSelected) {
        color = lightenColor(color, 40);
    }

    // オブジェクトタイプに応じた3Dモデルを作成
    const objectMesh = create3DModelMesh(
        coords3D,
        color,
        rectangle.objectType,
        rectangle.frontDirection,
        rectangle.objectProperties
    );

    if (objectMesh) {
        group.add(objectMesh);
    }

    // 前面方向矢印を追加
    if (rectangle.objectType !== OBJECT_TYPES.NONE && rectangle.frontDirection) {
        const arrow = createFrontDirectionArrow(coords3D, rectangle.frontDirection, color);
        if (arrow) {
            group.add(arrow);
        }
    }

    // 選択ハイライト
    if (isSelected) {
        const highlight = createSelectionHighlight(coords3D);
        if (highlight) {
            group.add(highlight);
        }
    }

    return group;
}

/**
 * オブジェクトタイプ別の3Dモデルメッシュを作成
 * @private
 */
function create3DModelMesh(coords3D, color, objectType, frontDirection, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;

    // 色をTHREE.Colorに変換
    const threeColor = new THREE.Color(color);

    switch (objectType) {
        case OBJECT_TYPES.SHELF:
            return createShelfMesh(coords3D, threeColor, frontDirection, objectProperties);
        case OBJECT_TYPES.BOX:
            return createBoxMesh(coords3D, threeColor, frontDirection, objectProperties);
        case OBJECT_TYPES.TABLE:
            return createTableMesh(coords3D, threeColor, objectProperties);
        case OBJECT_TYPES.DOOR:
            return createDoorMesh(coords3D, threeColor);
        case OBJECT_TYPES.WALL:
            return createWallMesh(coords3D, threeColor);
        default:
            return createDefaultMesh(coords3D, threeColor);
    }
}

/**
 * デフォルトメッシュ（直方体）を作成
 * @private
 */
function createDefaultMesh(coords3D, color) {
    const { x, y, z, width, depth, height } = coords3D;

    // BoxGeometryは中心が原点
    // coords3D.zは既に高さの中心座標（heightMeters / 2）
    const geometry = new THREE.BoxGeometry(width, height, depth);
    const material = new THREE.MeshStandardMaterial({
        color: color,
        transparent: true,
        opacity: 0.8,
    });

    const mesh = new THREE.Mesh(geometry, material);
    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    mesh.position.set(x, z, -y);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    return mesh;
}

/**
 * 棚メッシュを作成
 * @private
 */
function createShelfMesh(coords3D, color, frontDirection, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;
    const group = new THREE.Group();

    // 棚のプロパティ
    const shelfLevels = objectProperties?.shelfLevels || 3;
    const shelfDividers = objectProperties?.shelfDividers || 0;
    const shelfType = objectProperties?.shelfType || 'open';

    // 基本的な箱型（前面が開いている）
    const wallThickness = Math.min(width, depth) * 0.05;
    const material = new THREE.MeshStandardMaterial({ color: color });

    // 背面（奥側）
    const backGeometry = new THREE.BoxGeometry(width, height, wallThickness);
    const back = new THREE.Mesh(backGeometry, material);
    // 元の座標系(0, height/2, -depth/2) → Three.js(0, height/2, depth/2)
    back.position.set(0, height / 2, depth / 2 - wallThickness / 2);
    back.castShadow = true;
    back.receiveShadow = true;
    group.add(back);

    // 左側面
    const leftGeometry = new THREE.BoxGeometry(wallThickness, height, depth);
    const left = new THREE.Mesh(leftGeometry, material);
    left.position.set(-width / 2 + wallThickness / 2, height / 2, 0);
    left.castShadow = true;
    left.receiveShadow = true;
    group.add(left);

    // 右側面
    const right = new THREE.Mesh(leftGeometry, material);
    right.position.set(width / 2 - wallThickness / 2, height / 2, 0);
    right.castShadow = true;
    right.receiveShadow = true;
    group.add(right);

    // 上面
    const topGeometry = new THREE.BoxGeometry(width, wallThickness, depth);
    const top = new THREE.Mesh(topGeometry, material);
    top.position.set(0, height - wallThickness / 2, 0);
    top.castShadow = true;
    top.receiveShadow = true;
    group.add(top);

    // 底面
    const bottom = new THREE.Mesh(topGeometry, material);
    bottom.position.set(0, wallThickness / 2, 0);
    bottom.castShadow = true;
    bottom.receiveShadow = true;
    group.add(bottom);

    // 棚板
    for (let i = 1; i < shelfLevels; i++) {
        const shelfY = (height / shelfLevels) * i;
        const shelf = new THREE.Mesh(topGeometry, material);
        shelf.position.set(0, shelfY, 0);
        shelf.castShadow = true;
        shelf.receiveShadow = true;
        group.add(shelf);
    }

    // 前面方向に応じて回転
    applyFrontDirectionRotation(group, frontDirection);

    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    group.position.set(x, z, -y);

    return group;
}

/**
 * 箱メッシュを作成（上部開放）
 * @private
 */
function createBoxMesh(coords3D, color, frontDirection, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;
    const group = new THREE.Group();

    const wallThickness = Math.min(width, depth) * 0.05;
    const material = new THREE.MeshStandardMaterial({ color: color });

    // 底面
    const bottomGeometry = new THREE.BoxGeometry(width, wallThickness, depth);
    const bottom = new THREE.Mesh(bottomGeometry, material);
    bottom.position.set(0, wallThickness / 2, 0);
    bottom.castShadow = true;
    bottom.receiveShadow = true;
    group.add(bottom);

    // 4つの側面
    const sideHeight = height - wallThickness;

    // 前面（手前側）
    const frontGeometry = new THREE.BoxGeometry(width, sideHeight, wallThickness);
    const front = new THREE.Mesh(frontGeometry, material);
    front.position.set(0, wallThickness + sideHeight / 2, -depth / 2 + wallThickness / 2);
    front.castShadow = true;
    front.receiveShadow = true;
    group.add(front);

    // 背面（奥側）
    const back = new THREE.Mesh(frontGeometry, material);
    back.position.set(0, wallThickness + sideHeight / 2, depth / 2 - wallThickness / 2);
    back.castShadow = true;
    back.receiveShadow = true;
    group.add(back);

    // 左側面
    const sideGeometry = new THREE.BoxGeometry(wallThickness, sideHeight, depth);
    const left = new THREE.Mesh(sideGeometry, material);
    left.position.set(-width / 2 + wallThickness / 2, wallThickness + sideHeight / 2, 0);
    left.castShadow = true;
    left.receiveShadow = true;
    group.add(left);

    // 右側面
    const right = new THREE.Mesh(sideGeometry, material);
    right.position.set(width / 2 - wallThickness / 2, wallThickness + sideHeight / 2, 0);
    right.castShadow = true;
    right.receiveShadow = true;
    group.add(right);

    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    group.position.set(x, z, -y);

    return group;
}

/**
 * テーブルメッシュを作成
 * @private
 */
function createTableMesh(coords3D, color, objectProperties) {
    const { x, y, z, width, depth, height } = coords3D;
    const group = new THREE.Group();

    const material = new THREE.MeshStandardMaterial({ color: color });

    // 天板
    const topThickness = height * 0.1;
    const topGeometry = new THREE.BoxGeometry(width, topThickness, depth);
    const top = new THREE.Mesh(topGeometry, material);
    top.position.set(0, height - topThickness / 2, 0);
    top.castShadow = true;
    top.receiveShadow = true;
    group.add(top);

    // 脚（4本）
    const legThickness = Math.min(width, depth) * 0.1;
    const legHeight = height - topThickness;
    const legGeometry = new THREE.BoxGeometry(legThickness, legHeight, legThickness);

    const legPositions = [
        { x: -width / 2 + legThickness, z: -depth / 2 + legThickness },
        { x: width / 2 - legThickness, z: -depth / 2 + legThickness },
        { x: -width / 2 + legThickness, z: depth / 2 - legThickness },
        { x: width / 2 - legThickness, z: depth / 2 - legThickness },
    ];

    legPositions.forEach(pos => {
        const leg = new THREE.Mesh(legGeometry, material);
        leg.position.set(pos.x, legHeight / 2, pos.z);
        leg.castShadow = true;
        leg.receiveShadow = true;
        group.add(leg);
    });

    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    group.position.set(x, z, -y);

    return group;
}

/**
 * 扉メッシュを作成（薄型）
 * @private
 */
function createDoorMesh(coords3D, color) {
    const { x, y, z, width, depth, height } = coords3D;

    // 非常に薄い扉
    const doorThickness = Math.min(width, depth) * 0.1;
    const geometry = new THREE.BoxGeometry(width, height, doorThickness);
    const material = new THREE.MeshStandardMaterial({ color: color });

    const mesh = new THREE.Mesh(geometry, material);
    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    mesh.position.set(x, z, -y);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    return mesh;
}

/**
 * 壁メッシュを作成
 * @private
 */
function createWallMesh(coords3D, color) {
    const { x, y, z, width, depth, height } = coords3D;

    const geometry = new THREE.BoxGeometry(width, height, depth);
    const material = new THREE.MeshStandardMaterial({ color: color });

    const mesh = new THREE.Mesh(geometry, material);
    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    mesh.position.set(x, z, -y);
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    return mesh;
}

/**
 * 前面方向に応じた回転を適用
 * @private
 */
function applyFrontDirectionRotation(group, frontDirection) {
    // 棚の前面方向の回転
    // デフォルトで前面は-Z方向（手前）を向いている
    // 元の座標系: top=奥, bottom=手前, left=左, right=右
    switch (frontDirection) {
        case 'top':
            // 奥方向（+Z） = 180度回転
            group.rotation.y = Math.PI;
            break;
        case 'bottom':
            // 手前方向（-Z） = 回転なし
            break;
        case 'left':
            // 左方向（-X） = 90度右回転（-Y軸周り）
            group.rotation.y = -Math.PI / 2;
            break;
        case 'right':
            // 右方向（+X） = 90度左回転（+Y軸周り）
            group.rotation.y = Math.PI / 2;
            break;
    }
}

/**
 * 前面方向矢印を作成
 * @private
 */
function createFrontDirectionArrow(coords3D, frontDirection, color) {
    if (!frontDirection) return null;

    const { x, y, z, width, depth, height } = coords3D;

    // 矢印の向きと原点（Three.js座標系で）
    let dir = new THREE.Vector3();
    let origin = new THREE.Vector3();

    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    const threeX = x;
    const threeY = z + height; // オブジェクトの上端
    const threeZ = -y;

    switch (frontDirection) {
        case 'top':
            // 元の座標系で「奥」方向 → Three.jsで+Z方向
            dir.set(0, 0, 1);
            origin.set(threeX, threeY, threeZ + depth / 2);
            break;
        case 'bottom':
            // 元の座標系で「手前」方向 → Three.jsで-Z方向
            dir.set(0, 0, -1);
            origin.set(threeX, threeY, threeZ - depth / 2);
            break;
        case 'left':
            // 左方向（X軸負方向）
            dir.set(-1, 0, 0);
            origin.set(threeX - width / 2, threeY, threeZ);
            break;
        case 'right':
            // 右方向（X軸正方向）
            dir.set(1, 0, 0);
            origin.set(threeX + width / 2, threeY, threeZ);
            break;
    }

    const arrowLength = Math.max(width, depth) * 0.4;
    const arrowColor = new THREE.Color(color);

    const arrowHelper = new THREE.ArrowHelper(dir, origin, arrowLength, arrowColor, arrowLength * 0.2, arrowLength * 0.15);

    return arrowHelper;
}

/**
 * 選択ハイライトを作成
 * @private
 */
function createSelectionHighlight(coords3D) {
    const { x, y, z, width, depth, height } = coords3D;

    const geometry = new THREE.BoxGeometry(width * 1.1, height * 1.1, depth * 1.1);
    const edges = new THREE.EdgesGeometry(geometry);
    const material = new THREE.LineBasicMaterial({ color: 0xffff00, linewidth: 2 });
    const lineSegments = new THREE.LineSegments(edges, material);

    // 座標変換: アプリ(x, y, z) → Three.js(x, z, -y)
    lineSegments.position.set(x, z, -y);

    return lineSegments;
}

/**
 * 色を明るくする
 * @private
 */
function lightenColor(hex, percent) {
    const color = new THREE.Color(hex);
    color.offsetHSL(0, 0, percent / 100);
    return '#' + color.getHexString();
}

// ================
// オブジェクト選択
// ================

/**
 * 3Dオブジェクトを選択
 * @param {string} objectId - オブジェクトID
 */
export function select3DObject(objectId) {
    view3DState.selectedObjectId = objectId;
    render3DScene();
}

/**
 * 3Dオブジェクトの選択を解除
 */
export function deselect3DObject() {
    view3DState.selectedObjectId = null;
    render3DScene();

    const infoPanel = document.getElementById('view3DSelectedInfo');
    if (infoPanel) {
        infoPanel.style.display = 'none';
    }
}

/**
 * 2Dマップに移動
 */
export function goto2DMap() {
    // 実装は既存のコードに依存
    console.log('goto2DMap: 2Dマップタブに切り替え');
}

/**
 * オブジェクトカタログに移動
 */
export function gotoObjectCatalog() {
    // 実装は既存のコードに依存
    console.log('gotoObjectCatalog: カタログタブに切り替え');
}

/**
 * 特定のオブジェクトを更新
 * @param {string} rectangleId - 四角形ID
 */
export function update3DObject(rectangleId) {
    // 既存のメッシュを削除
    const oldMesh = view3DState.objectMeshes.get(rectangleId);
    if (oldMesh) {
        view3DState.scene.remove(oldMesh);
        view3DState.objectMeshes.delete(rectangleId);
    }

    // 新しいメッシュを作成
    const rectangle = getRectangleById(rectangleId);
    if (rectangle) {
        const mesh = create3DObjectMesh(rectangle);
        if (mesh) {
            view3DState.objectMeshes.set(rectangleId, mesh);
            view3DState.scene.add(mesh);
        }
    }
}

/**
 * ビューをリサイズ
 */
export function resize3DView() {
    handleResize();
}

/**
 * 回転角度を設定
 * @param {number} rotation - 回転角度（度）
 * @param {number} tilt - 傾き角度（度）
 */
export function set3DViewRotation(rotation, tilt) {
    // OrbitControlsを使用しているため、カメラ位置を直接設定
    if (!view3DState.camera || !view3DState.controls) return;

    const radius = 15; // カメラの距離
    const rotationRad = rotation * Math.PI / 180;
    const tiltRad = tilt * Math.PI / 180;

    const x = radius * Math.cos(tiltRad) * Math.sin(rotationRad);
    const y = radius * Math.sin(tiltRad);
    const z = radius * Math.cos(tiltRad) * Math.cos(rotationRad);

    view3DState.camera.position.set(x, y, z);
    view3DState.controls.update();
}

/**
 * ビューをリセット
 */
export function reset3DView() {
    if (!view3DState.camera || !view3DState.controls) return;

    view3DState.camera.position.set(
        view3DState.defaultCameraPosition.x,
        view3DState.defaultCameraPosition.y,
        view3DState.defaultCameraPosition.z
    );
    view3DState.controls.target.set(
        view3DState.defaultCameraTarget.x,
        view3DState.defaultCameraTarget.y,
        view3DState.defaultCameraTarget.z
    );
    view3DState.controls.update();
}

/**
 * マップ境界を取得
 */
export function getMapBounds() {
    return view3DState.mapBounds;
}

/**
 * マップ境界を設定
 * @param {Object} bounds - 境界情報
 */
export function setMapBounds(bounds) {
    view3DState.mapBounds = bounds;
}

// ================
// プレビュー機能
// ================

/**
 * プロパティプレビューを初期化
 */
export function initializePropertyPreview() {
    const canvas = document.getElementById('propertyPreviewCanvas');
    if (!canvas) {
        console.error('initializePropertyPreview: プレビューCanvas要素が見つかりません');
        return;
    }

    previewState.canvas = canvas;

    // Three.js のシーンを作成
    previewState.scene = new THREE.Scene();
    previewState.scene.background = new THREE.Color(0xf7fafc);

    // カメラを作成
    const aspect = canvas.clientWidth / canvas.clientHeight;
    previewState.camera = new THREE.PerspectiveCamera(50, aspect, 0.1, 100);
    previewState.camera.position.set(3, 3, 3);
    previewState.camera.lookAt(0, 0, 0);

    // レンダラーを作成
    previewState.renderer = new THREE.WebGLRenderer({
        canvas: canvas,
        antialias: true,
        alpha: true,
    });
    previewState.renderer.setSize(canvas.clientWidth, canvas.clientHeight);
    previewState.renderer.setPixelRatio(window.devicePixelRatio);

    // ライトを追加
    const ambientLight = new THREE.AmbientLight(0xffffff, 0.6);
    previewState.scene.add(ambientLight);

    const directionalLight = new THREE.DirectionalLight(0xffffff, 0.8);
    directionalLight.position.set(5, 5, 5);
    previewState.scene.add(directionalLight);

    // 簡易グリッド
    previewState.gridHelper = new THREE.GridHelper(2, 10, 0xcbd5e0, 0xcbd5e0);
    previewState.scene.add(previewState.gridHelper);

    previewState.isInitialized = true;

    console.log('プロパティプレビュー（Three.js版）を初期化しました');
}

/**
 * プロパティプレビューを描画
 * @param {string} rectangleId - 四角形ID
 */
export function renderPropertyPreview(rectangleId) {
    if (!previewState.isInitialized) {
        initializePropertyPreview();
    }

    if (!previewState.scene || !previewState.renderer || !previewState.camera) return;

    // 既存のオブジェクトメッシュを削除
    if (previewState.objectMesh) {
        previewState.scene.remove(previewState.objectMesh);
        previewState.objectMesh = null;
    }

    // 既存の矢印を削除
    if (previewState.arrowHelper) {
        previewState.scene.remove(previewState.arrowHelper);
        previewState.arrowHelper = null;
    }

    // 四角形を取得
    const rectangle = getRectangleById(rectangleId);
    if (!rectangle) return;

    // 3D座標を取得
    const coords3D = get3DCoordinates(rectangleId);
    if (!coords3D) return;

    // プレビュー用に座標を正規化（原点中心、床面に配置）
    // Three.js座標系では、底面をy=0に配置するため、zをheight/2に設定
    const previewCoords = {
        x: 0,
        y: 0,
        z: coords3D.height / 2,  // 底面がy=0になるように
        width: coords3D.width,
        depth: coords3D.depth,
        height: coords3D.height,
    };

    // オブジェクトメッシュを作成
    const color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    const mesh = create3DModelMesh(
        previewCoords,
        color,
        rectangle.objectType,
        rectangle.frontDirection,
        rectangle.objectProperties
    );

    if (mesh) {
        previewState.objectMesh = mesh;
        previewState.scene.add(mesh);
    }

    // 前面方向矢印を追加
    if (rectangle.frontDirection && rectangle.objectType !== OBJECT_TYPES.NONE) {
        const arrow = createFrontDirectionArrow(previewCoords, rectangle.frontDirection, color);
        if (arrow) {
            previewState.arrowHelper = arrow;
            previewState.scene.add(arrow);
        }
    }

    // カメラ位置を調整してオブジェクトが見えるようにする
    const maxDim = Math.max(coords3D.width, coords3D.depth, coords3D.height);
    const distance = maxDim * 2.5;
    previewState.camera.position.set(distance, distance, distance);
    previewState.camera.lookAt(0, coords3D.height / 2, 0);

    // レンダリング
    previewState.renderer.render(previewState.scene, previewState.camera);
}

/**
 * プレビュー状態を取得
 * @returns {Object} プレビュー状態
 */
export function getPreviewState() {
    return previewState;
}

// ================
// 互換性関数（Canvas版のAPI）
// ================

/**
 * プレビュー用グリッド描画（互換性のため空実装）
 */
export function drawPreviewGrid(ctx, centerX, centerY) {
    // Three.js版では不要
}

/**
 * プレビュー用3Dモデル描画（互換性のため空実装）
 */
export function drawPreviewModel(ctx, coords3D, color, centerX, centerY, objectType, frontDirection, objectProperties) {
    // Three.js版では不要
}

/**
 * プレビュー用前面方向矢印描画（互換性のため空実装）
 */
export function drawPreviewFrontDirection(ctx, coords3D, frontDirection, centerX, centerY) {
    // Three.js版では不要
}

/**
 * プレビュー用等角投影変換（互換性のため空実装）
 */
export function worldToPreviewIso(x, y, z) {
    // Three.js版では不要（Three.jsが自動的に投影変換を行う）
    return { x, y: z - y };
}
