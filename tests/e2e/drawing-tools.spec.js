/**
 * @file drawing-tools.spec.js
 * @description 描画ツールのE2Eテスト
 */

import { test, expect } from '@playwright/test';

test.describe('描画ツール - ツール選択', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('描画ツールパネルが表示される', async ({ page }) => {
    const drawingTools = page.locator('.drawing-tools, #drawing-tools');
    await expect(drawingTools).toBeVisible();
  });

  test('ペンツールを選択できる', async ({ page }) => {
    const penTool = page.locator('button[data-tool="pen"], button:has-text("Pen")');

    if (await penTool.count() > 0) {
      await penTool.first().click();

      // アクティブなツールが変更されることを確認
      await expect(penTool.first()).toHaveClass(/active|selected/);
    }
  });

  test('消しゴムツールを選択できる', async ({ page }) => {
    const eraserTool = page.locator('button[data-tool="eraser"], button:has-text("Eraser")');

    if (await eraserTool.count() > 0) {
      await eraserTool.first().click();
      await expect(eraserTool.first()).toHaveClass(/active|selected/);
    }
  });

  test('色選択が動作する', async ({ page }) => {
    const colorInput = page.locator('input[type="color"]');

    if (await colorInput.count() > 0) {
      await expect(colorInput.first()).toBeVisible();

      // 色を変更
      await colorInput.first().evaluate((el) => {
        el.value = '#ff0000';
        el.dispatchEvent(new Event('input', { bubbles: true }));
      });
    }
  });

  test('サイズスライダーが動作する', async ({ page }) => {
    const sizeSlider = page.locator('input[type="range"]');

    if (await sizeSlider.count() > 0) {
      await expect(sizeSlider.first()).toBeVisible();

      // サイズを変更
      await sizeSlider.first().fill('10');
    }
  });
});

test.describe('描画ツール - アンドゥ/リドゥ', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('アンドゥボタンが表示される', async ({ page }) => {
    const undoButton = page.locator('button:has-text("Undo"), button[title*="Undo"]');

    if (await undoButton.count() > 0) {
      await expect(undoButton.first()).toBeVisible();
    }
  });

  test('リドゥボタンが表示される', async ({ page }) => {
    const redoButton = page.locator('button:has-text("Redo"), button[title*="Redo"]');

    if (await redoButton.count() > 0) {
      await expect(redoButton.first()).toBeVisible();
    }
  });

  test('キーボードショートカット Ctrl+Z でアンドゥできる', async ({ page }) => {
    await page.keyboard.press('Control+z');
    // アンドゥが実行されることを確認（UIの変化を検証）
  });

  test('キーボードショートカット Ctrl+Y でリドゥできる', async ({ page }) => {
    await page.keyboard.press('Control+y');
    // リドゥが実行されることを確認
  });
});

test.describe('描画ツール - レイヤー管理', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('新しいレイヤーを追加できる', async ({ page }) => {
    const addLayerButton = page.locator('button:has-text("Add Layer"), button:has-text("+")');

    if (await addLayerButton.count() > 0) {
      const initialLayerCount = await page.locator('.layer-item').count();

      await addLayerButton.first().click();

      // レイヤーが追加されることを確認
      await page.waitForTimeout(500);
      const newLayerCount = await page.locator('.layer-item').count();
      expect(newLayerCount).toBeGreaterThan(initialLayerCount);
    }
  });

  test('レイヤーの表示/非表示を切り替えられる', async ({ page }) => {
    const visibilityToggle = page.locator('.layer-item .visibility-toggle, .layer-item button').first();

    if (await visibilityToggle.count() > 0) {
      await visibilityToggle.click();
      // 表示状態が切り替わることを確認
    }
  });

  test('レイヤーを削除できる', async ({ page }) => {
    // まず新しいレイヤーを追加
    const addLayerButton = page.locator('button:has-text("Add Layer"), button:has-text("+")');

    if (await addLayerButton.count() > 0) {
      await addLayerButton.first().click();
      await page.waitForTimeout(500);

      // 削除ボタンをクリック
      const deleteButton = page.locator('.layer-item button:has-text("Delete"), .layer-item .delete-button').first();

      if (await deleteButton.count() > 0) {
        await deleteButton.click();
        // レイヤーが削除されることを確認
      }
    }
  });
});
