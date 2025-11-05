import { defineConfig, devices } from '@playwright/test';

/**
 * Playwright E2Eテスト設定
 * @see https://playwright.dev/docs/test-configuration
 */
export default defineConfig({
  testDir: './tests/e2e',

  // タイムアウト設定
  timeout: 30000,
  expect: {
    timeout: 5000
  },

  // 並列実行設定
  fullyParallel: true,
  forbidOnly: !!process.env.CI,
  retries: process.env.CI ? 2 : 0,
  workers: process.env.CI ? 1 : undefined,

  // レポーター設定
  reporter: [
    ['html', { outputFolder: 'tests/reports/playwright' }],
    ['json', { outputFile: 'tests/reports/playwright/results.json' }],
    ['list']
  ],

  // 共通設定
  use: {
    baseURL: 'http://localhost:5173',
    trace: 'on-first-retry',
    screenshot: 'only-on-failure',
    video: 'retain-on-failure',
  },

  // テスト対象ブラウザ
  projects: [
    {
      name: 'chromium',
      use: { ...devices['Desktop Chrome'] },
    },
    {
      name: 'firefox',
      use: { ...devices['Desktop Firefox'] },
    },
    {
      name: 'webkit',
      use: { ...devices['Desktop Safari'] },
    },
  ],

  // 開発サーバーの起動設定
  webServer: [
    {
      command: 'python apps/frontend/server.py',
      port: 5173,
      timeout: 120000,
      reuseExistingServer: !process.env.CI,
    },
    {
      command: 'python apps/backend/server.py',
      port: 3000,
      timeout: 120000,
      reuseExistingServer: !process.env.CI,
    }
  ],
});
