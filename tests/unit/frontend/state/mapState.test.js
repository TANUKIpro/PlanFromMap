/**
 * @file mapState.test.js
 * @description mapState.jsのユニットテスト
 */

import { describe, it, expect, beforeEach } from 'vitest';
import {
  mapState,
  getState,
  updateState,
  resetState,
  addLayerToStack,
  removeLayerFromStack,
  getLayerById,
  getSelectedLayer,
  getNextLayerId
} from '@state/mapState.js';

describe('mapState', () => {
  beforeEach(() => {
    // 各テスト前にstateをリセット
    resetState();
  });

  describe('初期状態', () => {
    it('mapStateオブジェクトが存在する', () => {
      expect(mapState).toBeDefined();
    });

    it('layerStackが空配列で初期化されている', () => {
      expect(mapState.layerStack).toEqual([]);
    });

    it('scaleが1.0で初期化されている', () => {
      expect(mapState.scale).toBe(1.0);
    });

    it('offsetX, offsetYが0で初期化されている', () => {
      expect(mapState.offsetX).toBe(0);
      expect(mapState.offsetY).toBe(0);
    });

    it('selectedLayerIdがnullで初期化されている', () => {
      expect(mapState.selectedLayerId).toBeNull();
    });

    it('mapImageがnullで初期化されている', () => {
      expect(mapState.mapImage).toBeNull();
    });

    it('metadataがnullで初期化されている', () => {
      expect(mapState.metadata).toBeNull();
    });
  });

  describe('getState', () => {
    it('存在するキーの値を取得できる', () => {
      expect(getState('scale')).toBe(1.0);
    });

    it('存在しないキーの場合はundefinedを返す', () => {
      expect(getState('nonExistentKey')).toBeUndefined();
    });
  });

  describe('updateState', () => {
    it('単一のプロパティを更新できる', () => {
      updateState({ scale: 2.0 });
      expect(mapState.scale).toBe(2.0);
    });

    it('複数のプロパティを同時に更新できる', () => {
      updateState({
        scale: 1.5,
        offsetX: 100,
        offsetY: 200
      });

      expect(mapState.scale).toBe(1.5);
      expect(mapState.offsetX).toBe(100);
      expect(mapState.offsetY).toBe(200);
    });

    it('他のプロパティは影響を受けない', () => {
      const originalOffsetY = mapState.offsetY;

      updateState({ offsetX: 50 });

      expect(mapState.offsetX).toBe(50);
      expect(mapState.offsetY).toBe(originalOffsetY);
    });
  });

  describe('resetState', () => {
    it('状態を初期値にリセットできる', () => {
      // 状態を変更
      updateState({
        scale: 2.0,
        offsetX: 100,
        selectedLayerId: 'test-layer'
      });

      // リセット
      resetState();

      // 初期値に戻っていることを確認
      expect(mapState.scale).toBe(1.0);
      expect(mapState.offsetX).toBe(0);
      expect(mapState.selectedLayerId).toBeNull();
    });

    it('layerStackが空になる', () => {
      addLayerToStack({ id: 'layer1', name: 'Test Layer' });

      resetState();

      expect(mapState.layerStack).toEqual([]);
    });
  });

  describe('addLayerToStack', () => {
    it('新しいレイヤーをスタックに追加できる', () => {
      const layer = { id: 'layer1', name: 'Test Layer', visible: true };

      addLayerToStack(layer);

      expect(mapState.layerStack).toHaveLength(1);
      expect(mapState.layerStack[0]).toEqual(layer);
    });

    it('複数のレイヤーを追加できる', () => {
      addLayerToStack({ id: 'layer1', name: 'Layer 1' });
      addLayerToStack({ id: 'layer2', name: 'Layer 2' });

      expect(mapState.layerStack).toHaveLength(2);
    });
  });

  describe('removeLayerFromStack', () => {
    it('指定されたIDのレイヤーを削除できる', () => {
      addLayerToStack({ id: 'layer1', name: 'Layer 1' });
      addLayerToStack({ id: 'layer2', name: 'Layer 2' });

      removeLayerFromStack('layer1');

      expect(mapState.layerStack).toHaveLength(1);
      expect(mapState.layerStack[0].id).toBe('layer2');
    });

    it('存在しないIDを指定した場合、何も変更されない', () => {
      addLayerToStack({ id: 'layer1', name: 'Layer 1' });

      removeLayerFromStack('nonExistentLayer');

      expect(mapState.layerStack).toHaveLength(1);
    });
  });

  describe('getLayerById', () => {
    beforeEach(() => {
      addLayerToStack({ id: 'layer1', name: 'Layer 1', type: 'image' });
      addLayerToStack({ id: 'layer2', name: 'Layer 2', type: 'drawing' });
    });

    it('存在するIDのレイヤーを取得できる', () => {
      const layer = getLayerById('layer1');

      expect(layer).toBeDefined();
      expect(layer.id).toBe('layer1');
      expect(layer.name).toBe('Layer 1');
    });

    it('存在しないIDの場合はundefinedを返す', () => {
      const layer = getLayerById('nonExistentLayer');

      expect(layer).toBeUndefined();
    });
  });

  describe('getSelectedLayer', () => {
    beforeEach(() => {
      addLayerToStack({ id: 'layer1', name: 'Layer 1' });
      addLayerToStack({ id: 'layer2', name: 'Layer 2' });
    });

    it('selectedLayerIdが設定されている場合、そのレイヤーを返す', () => {
      updateState({ selectedLayerId: 'layer2' });

      const selectedLayer = getSelectedLayer();

      expect(selectedLayer).toBeDefined();
      expect(selectedLayer.id).toBe('layer2');
    });

    it('selectedLayerIdがnullの場合、undefinedを返す', () => {
      updateState({ selectedLayerId: null });

      const selectedLayer = getSelectedLayer();

      expect(selectedLayer).toBeUndefined();
    });
  });

  describe('getNextLayerId', () => {
    it('連続したIDを生成する', () => {
      const id1 = getNextLayerId();
      const id2 = getNextLayerId();
      const id3 = getNextLayerId();

      expect(id1).toBe('layer-1');
      expect(id2).toBe('layer-2');
      expect(id3).toBe('layer-3');
    });

    it('resetState後、IDカウンターがリセットされる', () => {
      getNextLayerId(); // layer-1
      getNextLayerId(); // layer-2

      resetState();

      const newId = getNextLayerId();
      expect(newId).toBe('layer-1');
    });
  });

  describe('統合テスト', () => {
    it('レイヤーの追加、選択、削除の一連の操作が正しく動作する', () => {
      // レイヤー追加
      const layer1 = { id: getNextLayerId(), name: 'Layer 1', type: 'image' };
      const layer2 = { id: getNextLayerId(), name: 'Layer 2', type: 'drawing' };

      addLayerToStack(layer1);
      addLayerToStack(layer2);

      expect(mapState.layerStack).toHaveLength(2);

      // レイヤー選択
      updateState({ selectedLayerId: layer2.id });
      const selected = getSelectedLayer();
      expect(selected.id).toBe(layer2.id);

      // レイヤー削除
      removeLayerFromStack(layer1.id);
      expect(mapState.layerStack).toHaveLength(1);
      expect(getLayerById(layer1.id)).toBeUndefined();

      // まだlayer2は存在する
      expect(getLayerById(layer2.id)).toBeDefined();
    });
  });
});
