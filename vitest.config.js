import { defineConfig } from 'vitest/config';
import path from 'path';

export default defineConfig({
  test: {
    // テスト環境
    environment: 'jsdom',

    // グローバル変数を使用可能にする
    globals: true,

    // セットアップファイル
    setupFiles: ['./tests/setup.js'],

    // テストファイルのパターン
    include: [
      'tests/unit/frontend/**/*.test.js',
      'tests/integration/frontend/**/*.test.js'
    ],

    // カバレッジ設定
    coverage: {
      provider: 'v8',
      reporter: ['text', 'json', 'html', 'lcov'],
      reportsDirectory: './tests/coverage/frontend',
      include: ['apps/frontend/static/js/**/*.js'],
      exclude: [
        'apps/frontend/static/js/main.js',
        '**/*.test.js',
        '**/node_modules/**',
        '**/tests/**'
      ],
      all: true,
      lines: 70,
      functions: 70,
      branches: 70,
      statements: 70
    },

    // レポート設定
    reporters: ['verbose', 'html', 'json'],
    outputFile: {
      html: './tests/reports/test-results.html',
      json: './tests/reports/test-results.json'
    },

    // タイムアウト設定
    testTimeout: 10000,
    hookTimeout: 10000
  },

  resolve: {
    alias: {
      '@': path.resolve(__dirname, 'apps/frontend/static/js'),
      '@modules': path.resolve(__dirname, 'apps/frontend/static/js/modules'),
      '@utils': path.resolve(__dirname, 'apps/frontend/static/js/utils'),
      '@state': path.resolve(__dirname, 'apps/frontend/static/js/state'),
      '@ui': path.resolve(__dirname, 'apps/frontend/static/js/ui')
    }
  }
});
