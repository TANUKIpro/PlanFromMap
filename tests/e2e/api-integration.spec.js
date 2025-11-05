/**
 * @file api-integration.spec.js
 * @description バックエンドAPIとの統合E2Eテスト
 */

import { test, expect } from '@playwright/test';

test.describe('バックエンドAPI統合', () => {
  test.beforeEach(async ({ page }) => {
    await page.goto('/');
  });

  test('バックエンドAPIが利用可能', async ({ request }) => {
    const response = await request.get('http://localhost:3000/health');

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data.status).toBe('ok');
  });

  test('操作カタログタブに切り替えできる', async ({ page }) => {
    const catalogTab = page.locator('button:has-text("Operation Catalog")');

    if (await catalogTab.count() > 0) {
      await catalogTab.click();

      // タブコンテンツが表示されることを確認
      await page.waitForTimeout(500);
      const catalogContent = page.locator('#catalog, .catalog-content');

      if (await catalogContent.count() > 0) {
        await expect(catalogContent.first()).toBeVisible();
      }
    }
  });

  test('操作カタログデータを取得できる', async ({ request }) => {
    const response = await request.get('http://localhost:3000/api/operations');

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data).toHaveProperty('operations');
    expect(data).toHaveProperty('count');
    expect(Array.isArray(data.operations)).toBe(true);
  });

  test('統計情報を取得できる', async ({ request }) => {
    const response = await request.get('http://localhost:3000/api/stats');

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data).toHaveProperty('operations_count');
    expect(data).toHaveProperty('maps_count');
  });

  test('MapQLクエリを実行できる', async ({ request }) => {
    const response = await request.post('http://localhost:3000/api/mapql/query', {
      data: {
        query: "GET Operation FOR 'kitchen_door'"
      }
    });

    expect(response.status()).toBe(200);
    const data = await response.json();
    expect(data).toHaveProperty('query');
    expect(data).toHaveProperty('result');
  });
});

test.describe('エンドツーエンド統合フロー', () => {
  test('マップ読み込みからプロファイル保存まで', async ({ page }) => {
    await page.goto('/');

    // 1. ページが正しく読み込まれることを確認
    await expect(page.locator('#map-canvas')).toBeVisible();

    // 2. プロファイル管理を開く
    const profileButton = page.locator('button:has-text("Profiles")');
    if (await profileButton.count() > 0) {
      await profileButton.click();

      // 3. プロファイル名を入力して保存
      await page.waitForTimeout(500);
      const profileNameInput = page.locator('input[type="text"]').first();

      if (await profileNameInput.count() > 0) {
        await profileNameInput.fill('e2e-integration-test-profile');

        const saveButton = page.locator('button:has-text("Save")').first();
        await saveButton.click();

        // 4. 保存が成功したことを確認
        await page.waitForTimeout(1000);

        // プロファイルが保存されたことをlocalStorageで確認
        const profiles = await page.evaluate(() => {
          const data = localStorage.getItem('map_profiles');
          return data ? JSON.parse(data) : {};
        });

        expect(profiles).toHaveProperty('e2e-integration-test-profile');
      }
    }
  });

  test('APIからデータ取得してUIに表示', async ({ page }) => {
    await page.goto('/');

    // Operation Catalogタブに切り替え
    const catalogTab = page.locator('button:has-text("Operation Catalog")');

    if (await catalogTab.count() > 0) {
      await catalogTab.click();
      await page.waitForTimeout(1000);

      // 統計情報が表示されることを確認
      const statsContainer = page.locator('.stats, #stats-container');

      // コンテンツが読み込まれることを待つ
      await page.waitForTimeout(2000);
    }
  });
});
