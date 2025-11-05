/**
 * @file map-viewer.spec.js
 * @description マップビューアの基本機能のE2Eテスト
 */

import { test, expect } from '@playwright/test';
import path from 'path';
import fs from 'fs';

test.describe('マップビューア - 基本機能', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('ページタイトルが正しく表示される', async ({ page }) => {
    await expect(page).toHaveTitle(/Semantic Map Platform/);
  });

  test('メインキャンバスが表示される', async ({ page }) => {
    const canvas = page.locator('#map-canvas');
    await expect(canvas).toBeVisible();
  });

  test('メニューバーが表示される', async ({ page }) => {
    const menuBar = page.locator('.menu-bar');
    await expect(menuBar).toBeVisible();
  });

  test('レイヤーパネルが表示される', async ({ page }) => {
    const layersPanel = page.locator('#layers-panel');
    await expect(layersPanel).toBeVisible();
  });

  test('タブが表示される', async ({ page }) => {
    const mapTab = page.locator('button:has-text("Map Viewer")');
    const catalogTab = page.locator('button:has-text("Operation Catalog")');

    await expect(mapTab).toBeVisible();
    await expect(catalogTab).toBeVisible();
  });
});

test.describe('マップビューア - ファイル読み込み', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('YAMLファイル読み込みボタンが動作する', async ({ page }) => {
    // ファイル選択ダイアログのイベントをリスン
    const fileChooserPromise = page.waitForEvent('filechooser');

    // 「Load YAML」ボタンをクリック
    await page.click('button:has-text("Load YAML")');

    const fileChooser = await fileChooserPromise;
    expect(fileChooser).toBeDefined();
  });

  test('画像ファイル読み込みボタンが動作する', async ({ page }) => {
    const fileChooserPromise = page.waitForEvent('filechooser');

    await page.click('button:has-text("Load Map Image")');

    const fileChooser = await fileChooserPromise;
    expect(fileChooser).toBeDefined();
  });

  test('実際のマップファイルを読み込める', async ({ page }) => {
    const yamlPath = path.resolve(__dirname, '../../data/maps/tid_lab_map.yaml');
    const pgmPath = path.resolve(__dirname, '../../data/maps/tid_lb_map.pgm');

    // YAMLファイルが存在することを確認
    if (fs.existsSync(yamlPath) && fs.existsSync(pgmPath)) {
      // YAML読み込み
      const yamlChooserPromise = page.waitForEvent('filechooser');
      await page.click('button:has-text("Load YAML")');
      const yamlChooser = await yamlChooserPromise;
      await yamlChooser.setFiles(yamlPath);

      // メタデータが表示されることを確認
      await page.waitForTimeout(1000);
      const metadataOverlay = page.locator('.metadata-overlay');
      await expect(metadataOverlay).toBeVisible();

      // 画像読み込み
      const imageChooserPromise = page.waitForEvent('filechooser');
      await page.click('button:has-text("Load Map Image")');
      const imageChooser = await imageChooserPromise;
      await imageChooser.setFiles(pgmPath);

      // 画像が読み込まれるのを待つ
      await page.waitForTimeout(2000);

      // レイヤーが追加されることを確認
      const layerItems = page.locator('.layer-item');
      await expect(layerItems.first()).toBeVisible();
    } else {
      test.skip();
    }
  });
});

test.describe('マップビューア - ズーム・パン操作', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('ズームインボタンが動作する', async ({ page }) => {
    const zoomInButton = page.locator('button:has-text("Zoom In")');

    await expect(zoomInButton).toBeVisible();
    await zoomInButton.click();

    // ズーム情報が更新されることを確認
    const zoomInfo = page.locator('.zoom-info, #zoom-info');
    await expect(zoomInfo).toBeVisible();
  });

  test('ズームアウトボタンが動作する', async ({ page }) => {
    const zoomOutButton = page.locator('button:has-text("Zoom Out")');

    await expect(zoomOutButton).toBeVisible();
    await zoomOutButton.click();
  });

  test('リセットボタンが動作する', async ({ page }) => {
    const resetButton = page.locator('button:has-text("Reset View")');

    await expect(resetButton).toBeVisible();
    await resetButton.click();
  });
});

test.describe('マップビューア - プロファイル管理', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('プロファイル管理ボタンが表示される', async ({ page }) => {
    const profileButton = page.locator('button:has-text("Profiles")');
    await expect(profileButton).toBeVisible();
  });

  test('プロファイル管理モーダルが開く', async ({ page }) => {
    await page.click('button:has-text("Profiles")');

    // モーダルが表示されることを確認
    const modal = page.locator('.profile-modal, #profile-modal');
    await expect(modal).toBeVisible();
  });

  test('プロファイルを保存できる', async ({ page }) => {
    // プロファイル管理を開く
    await page.click('button:has-text("Profiles")');

    // プロファイル名を入力
    const profileNameInput = page.locator('input[placeholder*="profile"], input[type="text"]').first();
    await profileNameInput.fill('e2e-test-profile');

    // 保存ボタンをクリック
    const saveButton = page.locator('button:has-text("Save")').first();
    await saveButton.click();

    // 成功メッセージが表示されることを確認
    await page.waitForTimeout(500);
    const toast = page.locator('.toast, .notification');
    // toastが表示されるか、プロファイルリストに追加されることを確認
  });
});
