/**
 * @file threeDObjects.js
 * @description 3Dオブジェクトメッシュ生成
 *
 * 各オブジェクトタイプ（SHELF, BOX, TABLE, DOOR, WALL）の
 * Three.jsメッシュを生成する責務を持ちます。
 *
 * @requires three
 * @requires ../utils/threeHelpers.js
 * @requires ../models/objectTypes.js
 *
 * @exports create3DObjectMesh - 3Dオブジェクトメッシュを作成
 * @exports createShelfMesh - 棚メッシュを作成
 * @exports createBoxMesh - 箱メッシュを作成
 * @exports createTableMesh - テーブルメッシュを作成
 * @exports createDoorMesh - ドアメッシュを作成
 * @exports createWallMesh - 壁メッシュを作成
 * @exports createDefaultBoxMesh - デフォルトボックスメッシュを作成
 * @exports addSelectionHighlight - 選択ハイライトを追加
 * @exports addFrontDirectionArrow - 前面方向矢印を追加
 */

import * as THREE from 'three';
import { lightenColor, darkenColor, lightenColorHex } from '../utils/threeHelpers.js';
import { OBJECT_TYPES, OBJECT_TYPE_COLORS } from '../models/objectTypes.js';

/**
 * 3Dオブジェクトのメッシュを作成
 *
 * @param {Object} coords3D - 3D座標情報 {x, y, z, width, depth, height}
 * @param {Object} rectangle - 矩形オブジェクト
 * @param {boolean} isSelected - 選択されているか
 * @returns {THREE.Object3D} 3Dメッシュ
 */
export function create3DObjectMesh(coords3D, rectangle, isSelected) {
    const { x, y, z, width, depth, height } = coords3D;

    let color = rectangle.objectType && rectangle.objectType !== OBJECT_TYPES.NONE
        ? OBJECT_TYPE_COLORS[rectangle.objectType]
        : OBJECT_TYPE_COLORS.none;

    // 選択されている場合は色を明るくする
    if (isSelected) {
        color = lightenColorHex(color, 40);
    }

    const objectType = rectangle.objectType || OBJECT_TYPES.NONE;
    const frontDirection = rectangle.frontDirection || 'top';
    const objectProperties = rectangle.objectProperties || {};

    let mesh;
    switch (objectType) {
        case OBJECT_TYPES.SHELF:
            mesh = createShelfMesh(coords3D, color, frontDirection, objectProperties);
            break;
        case OBJECT_TYPES.BOX:
            mesh = createBoxMesh(coords3D, color, frontDirection, objectProperties);
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

    // 位置を設定（ROS座標系: Y軸反転）
    if (mesh) {
        mesh.position.set(x, z, -y);
        mesh.castShadow = true;
        mesh.receiveShadow = true;

        // 選択ハイライトを追加
        if (isSelected) {
            addSelectionHighlight(mesh, coords3D);
        }

        // 前面方向矢印を追加
        if (objectType !== OBJECT_TYPES.NONE && frontDirection) {
            addFrontDirectionArrow(mesh, coords3D, frontDirection);
        }
    }

    return mesh;
}

/**
 * 棚のメッシュを作成（前面が開いている）
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - 色
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 * @returns {THREE.Group} 棚メッシュ
 */
export function createShelfMesh(coords3D, color, frontDirection, objectProperties) {
    const { width, depth, height } = coords3D;
    const group = new THREE.Group();

    const baseColor = new THREE.Color(color);
    const lightColor = lightenColor(baseColor, 20);
    const darkColor = darkenColor(baseColor, 10);

    // 前面が開いている箱を作成
    const materials = [
        new THREE.MeshLambertMaterial({ color: baseColor }),      // 右
        new THREE.MeshLambertMaterial({ color: darkColor }),      // 左
        new THREE.MeshLambertMaterial({ color: lightColor }),     // 上
        new THREE.MeshLambertMaterial({ color: darkColor }),      // 下
        new THREE.MeshLambertMaterial({ color: 0x000000, opacity: 0, transparent: true }), // 前（透明）
        new THREE.MeshLambertMaterial({ color: darkColor })       // 後
    ];

    // 前面の向きに応じて透明化する面を変更
    switch (frontDirection) {
        case 'top':    // 前面（Z負方向）を開く
            materials[4].opacity = 0;
            break;
        case 'bottom': // 背面（Z正方向）を開く
            materials[5].opacity = 0;
            materials[4].opacity = 1;
            materials[4].color = darkColor;
            break;
        case 'left':   // 左面を開く
            materials[1].opacity = 0;
            materials[4].opacity = 1;
            materials[4].color = darkColor;
            break;
        case 'right':  // 右面を開く
            materials[0].opacity = 0;
            materials[4].opacity = 1;
            materials[4].color = darkColor;
            break;
    }

    const geometry = new THREE.BoxGeometry(width, height, depth);
    const mesh = new THREE.Mesh(geometry, materials);
    group.add(mesh);

    // 棚板を追加
    if (objectProperties && objectProperties.shelfLevels && objectProperties.shelfLevels > 1) {
        const levels = objectProperties.shelfLevels;
        const shelfMaterial = new THREE.MeshLambertMaterial({
            color: darkColor,
            side: THREE.DoubleSide
        });

        for (let i = 1; i < levels; i++) {
            const shelfY = -height / 2 + (height / levels) * i;
            const shelfGeometry = new THREE.PlaneGeometry(width * 0.95, depth * 0.95);
            const shelf = new THREE.Mesh(shelfGeometry, shelfMaterial);
            shelf.rotation.x = -Math.PI / 2;
            shelf.position.y = shelfY;
            group.add(shelf);
        }
    }

    return group;
}

/**
 * 箱のメッシュを作成（上部が開いている）
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - 色
 * @param {string} frontDirection - 前面方向
 * @param {Object} objectProperties - オブジェクトプロパティ
 * @returns {THREE.Group} 箱メッシュ
 */
export function createBoxMesh(coords3D, color, frontDirection, objectProperties) {
    const { width, depth, height } = coords3D;
    const group = new THREE.Group();

    const baseColor = new THREE.Color(color);
    const lightColor = lightenColor(baseColor, 20);
    const darkColor = darkenColor(baseColor, 10);

    // 上部が開いている箱
    const materials = [
        new THREE.MeshLambertMaterial({ color: baseColor }),      // 右
        new THREE.MeshLambertMaterial({ color: darkColor }),      // 左
        new THREE.MeshLambertMaterial({ color: 0x000000, opacity: 0, transparent: true }), // 上（透明）
        new THREE.MeshLambertMaterial({ color: darkColor }),      // 下
        new THREE.MeshLambertMaterial({ color: darkColor }),      // 前
        new THREE.MeshLambertMaterial({ color: darkColor })       // 後
    ];

    const geometry = new THREE.BoxGeometry(width, height, depth);
    const mesh = new THREE.Mesh(geometry, materials);
    group.add(mesh);

    return group;
}

/**
 * テーブルのメッシュを作成
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - 色
 * @returns {THREE.Group} テーブルメッシュ
 */
export function createTableMesh(coords3D, color) {
    const { width, depth, height } = coords3D;
    const group = new THREE.Group();

    const baseColor = new THREE.Color(color);
    const legThickness = 0.05; // 5cm
    const tableTopThickness = 0.03; // 3cm

    // 天板
    const topGeometry = new THREE.BoxGeometry(width, tableTopThickness, depth);
    const topMaterial = new THREE.MeshLambertMaterial({ color: lightenColor(baseColor, 20) });
    const top = new THREE.Mesh(topGeometry, topMaterial);
    top.position.y = height / 2 - tableTopThickness / 2;
    group.add(top);

    // 脚（4本）
    const legGeometry = new THREE.BoxGeometry(legThickness, height - tableTopThickness, legThickness);
    const legMaterial = new THREE.MeshLambertMaterial({ color: baseColor });

    const legPositions = [
        [-width / 2 + legThickness, -(tableTopThickness / 2), -depth / 2 + legThickness],
        [width / 2 - legThickness, -(tableTopThickness / 2), -depth / 2 + legThickness],
        [-width / 2 + legThickness, -(tableTopThickness / 2), depth / 2 - legThickness],
        [width / 2 - legThickness, -(tableTopThickness / 2), depth / 2 - legThickness]
    ];

    legPositions.forEach(pos => {
        const leg = new THREE.Mesh(legGeometry, legMaterial);
        leg.position.set(pos[0], pos[1], pos[2]);
        group.add(leg);
    });

    return group;
}

/**
 * ドアのメッシュを作成
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - 色
 * @returns {THREE.Mesh} ドアメッシュ
 */
export function createDoorMesh(coords3D, color) {
    const { width, depth, height } = coords3D;

    // ドアは薄い
    const doorThickness = Math.min(depth, 0.05);

    const geometry = new THREE.BoxGeometry(width, height, doorThickness);
    const material = new THREE.MeshLambertMaterial({ color: color });
    const mesh = new THREE.Mesh(geometry, material);

    return mesh;
}

/**
 * 壁のメッシュを作成
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - 色
 * @returns {THREE.Mesh} 壁メッシュ
 */
export function createWallMesh(coords3D, color) {
    return createDefaultBoxMesh(coords3D, color);
}

/**
 * デフォルトの箱メッシュを作成
 *
 * @param {Object} coords3D - 3D座標情報
 * @param {string} color - 色
 * @returns {THREE.Mesh} デフォルトメッシュ
 */
export function createDefaultBoxMesh(coords3D, color) {
    const { width, depth, height } = coords3D;

    const geometry = new THREE.BoxGeometry(width, height, depth);
    const material = new THREE.MeshLambertMaterial({ color: color });
    const mesh = new THREE.Mesh(geometry, material);

    return mesh;
}

/**
 * 選択ハイライトを追加
 *
 * @param {THREE.Object3D} mesh - メッシュ
 * @param {Object} coords3D - 3D座標情報
 */
export function addSelectionHighlight(mesh, coords3D) {
    const { width, depth, height } = coords3D;

    const geometry = new THREE.BoxGeometry(width * 1.1, height * 1.1, depth * 1.1);
    const edges = new THREE.EdgesGeometry(geometry);
    const material = new THREE.LineBasicMaterial({
        color: 0x667eea,
        linewidth: 3
    });
    const highlight = new THREE.LineSegments(edges, material);

    // メッシュの子として追加
    if (mesh.isGroup) {
        mesh.add(highlight);
    }
}

/**
 * 前面方向矢印を追加
 *
 * @param {THREE.Object3D} mesh - メッシュ
 * @param {Object} coords3D - 3D座標情報
 * @param {string} frontDirection - 前面方向
 */
export function addFrontDirectionArrow(mesh, coords3D, frontDirection) {
    const { width, depth, height } = coords3D;

    const arrowLength = 0.3;
    const arrowColor = 0xe74c3c;

    let dir, origin;

    switch (frontDirection) {
        case 'top':    // Z負方向
            dir = new THREE.Vector3(0, 0, -1);
            origin = new THREE.Vector3(0, 0, -depth / 2);
            break;
        case 'bottom': // Z正方向
            dir = new THREE.Vector3(0, 0, 1);
            origin = new THREE.Vector3(0, 0, depth / 2);
            break;
        case 'left':   // X負方向
            dir = new THREE.Vector3(-1, 0, 0);
            origin = new THREE.Vector3(-width / 2, 0, 0);
            break;
        case 'right':  // X正方向
            dir = new THREE.Vector3(1, 0, 0);
            origin = new THREE.Vector3(width / 2, 0, 0);
            break;
        default:
            dir = new THREE.Vector3(0, 0, -1);
            origin = new THREE.Vector3(0, 0, -depth / 2);
    }

    const arrow = new THREE.ArrowHelper(
        dir,
        origin,
        arrowLength,
        arrowColor,
        arrowLength * 0.3,
        arrowLength * 0.2
    );

    if (mesh.isGroup) {
        mesh.add(arrow);
    }
}
