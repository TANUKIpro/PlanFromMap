/**
 * Vitest セットアップファイル
 * すべてのテストの前に実行される
 */

import { vi } from 'vitest';
import fs from 'fs';
import path from 'path';

// グローバルなモック設定
global.localStorage = {
  store: {},
  getItem(key) {
    return this.store[key] || null;
  },
  setItem(key, value) {
    this.store[key] = String(value);
  },
  removeItem(key) {
    delete this.store[key];
  },
  clear() {
    this.store = {};
  }
};

global.sessionStorage = {
  store: {},
  getItem(key) {
    return this.store[key] || null;
  },
  setItem(key, value) {
    this.store[key] = String(value);
  },
  removeItem(key) {
    delete this.store[key];
  },
  clear() {
    this.store = {};
  }
};

// Canvas API のモック
HTMLCanvasElement.prototype.getContext = vi.fn(() => ({
  fillStyle: '',
  strokeStyle: '',
  lineWidth: 1,
  globalAlpha: 1,
  fillRect: vi.fn(),
  strokeRect: vi.fn(),
  clearRect: vi.fn(),
  beginPath: vi.fn(),
  closePath: vi.fn(),
  moveTo: vi.fn(),
  lineTo: vi.fn(),
  arc: vi.fn(),
  fill: vi.fn(),
  stroke: vi.fn(),
  drawImage: vi.fn(),
  getImageData: vi.fn(() => ({
    data: new Uint8ClampedArray(4),
    width: 1,
    height: 1
  })),
  putImageData: vi.fn(),
  createImageData: vi.fn(() => ({
    data: new Uint8ClampedArray(4),
    width: 1,
    height: 1
  })),
  setTransform: vi.fn(),
  save: vi.fn(),
  restore: vi.fn(),
  scale: vi.fn(),
  translate: vi.fn(),
  measureText: vi.fn(() => ({ width: 0 })),
  canvas: {
    width: 800,
    height: 600
  }
}));

// Image のモック
class MockImage {
  constructor() {
    this.src = '';
    this.width = 100;
    this.height = 100;
    this.onload = null;
    this.onerror = null;
  }

  set src(value) {
    this._src = value;
    // 非同期でonloadを呼び出す
    setTimeout(() => {
      if (this.onload) {
        this.onload();
      }
    }, 0);
  }

  get src() {
    return this._src;
  }
}

global.Image = MockImage;

// FileReader のモック
class MockFileReader {
  constructor() {
    this.result = null;
    this.onload = null;
    this.onerror = null;
  }

  readAsDataURL(file) {
    this.result = 'data:image/png;base64,mock-data';
    setTimeout(() => {
      if (this.onload) {
        this.onload({ target: this });
      }
    }, 0);
  }

  readAsArrayBuffer(file) {
    this.result = new ArrayBuffer(8);
    setTimeout(() => {
      if (this.onload) {
        this.onload({ target: this });
      }
    }, 0);
  }

  readAsText(file) {
    this.result = 'mock text content';
    setTimeout(() => {
      if (this.onload) {
        this.onload({ target: this });
      }
    }, 0);
  }
}

global.FileReader = MockFileReader;

// テストレポートディレクトリの作成
const reportsDir = path.join(process.cwd(), 'tests', 'reports');
if (!fs.existsSync(reportsDir)) {
  fs.mkdirSync(reportsDir, { recursive: true });
}

// テスト実行前のクリーンアップ
beforeEach(() => {
  // localStorageとsessionStorageをクリア
  global.localStorage.clear();
  global.sessionStorage.clear();
});

// テスト実行後のクリーンアップ
afterEach(() => {
  // モックをリセット
  vi.clearAllMocks();
});

console.log('✅ Vitest setup completed');
