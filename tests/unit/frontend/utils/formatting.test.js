/**
 * @file formatting.test.js
 * @description formatting.jsのユニットテスト
 */

import { describe, it, expect } from 'vitest';
import { formatDistance, getNiceNumber } from '@utils/formatting.js';

describe('formatDistance', () => {
  describe('キロメートル単位（1000m以上）', () => {
    it('1500mを"1.50 km"にフォーマットする', () => {
      expect(formatDistance(1500)).toBe('1.50 km');
    });

    it('2345mを"2.35 km"にフォーマットする', () => {
      expect(formatDistance(2345)).toBe('2.35 km');
    });

    it('10000mを"10.00 km"にフォーマットする', () => {
      expect(formatDistance(10000)).toBe('10.00 km');
    });
  });

  describe('メートル単位（1m以上）', () => {
    it('25.5mを整数"26 m"にフォーマットする', () => {
      expect(formatDistance(25.5)).toBe('26 m');
    });

    it('5.5mを"5.50 m"にフォーマットする（10m未満）', () => {
      expect(formatDistance(5.5)).toBe('5.50 m');
    });

    it('100mを"100 m"にフォーマットする', () => {
      expect(formatDistance(100)).toBe('100 m');
    });

    it('1.234mを"1.23 m"にフォーマットする', () => {
      expect(formatDistance(1.234)).toBe('1.23 m');
    });
  });

  describe('センチメートル単位（1cm以上）', () => {
    it('0.053m（5.3cm）を"5.3 cm"にフォーマットする', () => {
      expect(formatDistance(0.053)).toBe('5.3 cm');
    });

    it('0.15m（15cm）を整数"15 cm"にフォーマットする', () => {
      expect(formatDistance(0.15)).toBe('15 cm');
    });

    it('0.12m（12cm）を整数"12 cm"にフォーマットする', () => {
      expect(formatDistance(0.12)).toBe('12 cm');
    });
  });

  describe('ミリメートル単位（1cm未満）', () => {
    it('0.0082m（8.2mm）を整数"8 mm"にフォーマットする', () => {
      expect(formatDistance(0.0082)).toBe('8 mm');
    });

    it('0.0001m（0.1mm）を"0 mm"にフォーマットする', () => {
      expect(formatDistance(0.0001)).toBe('0 mm');
    });

    it('0.005m（5mm）を"5 mm"にフォーマットする', () => {
      expect(formatDistance(0.005)).toBe('5 mm');
    });
  });

  describe('エッジケース', () => {
    it('0を"0 mm"にフォーマットする', () => {
      expect(formatDistance(0)).toBe('0 mm');
    });

    it('負の値を正しくフォーマットする', () => {
      expect(formatDistance(-100)).toBe('-100 m');
      expect(formatDistance(-1500)).toBe('-1.50 km');
    });

    it('Infinityを空文字列にする', () => {
      expect(formatDistance(Infinity)).toBe('');
    });

    it('NaNを空文字列にする', () => {
      expect(formatDistance(NaN)).toBe('');
    });
  });
});

describe('getNiceNumber', () => {
  describe('基本的な丸め', () => {
    it('0.73を1に丸める', () => {
      expect(getNiceNumber(0.73)).toBe(1);
    });

    it('1.4を1に丸める', () => {
      expect(getNiceNumber(1.4)).toBe(1);
    });

    it('2.8を2に丸める', () => {
      expect(getNiceNumber(2.8)).toBe(2);
    });

    it('4.5を5に丸める', () => {
      expect(getNiceNumber(4.5)).toBe(5);
    });

    it('8.2を10に丸める', () => {
      expect(getNiceNumber(8.2)).toBe(10);
    });
  });

  describe('大きな数値', () => {
    it('37を50に丸める', () => {
      expect(getNiceNumber(37)).toBe(50);
    });

    it('142を100に丸める', () => {
      expect(getNiceNumber(142)).toBe(100);
    });

    it('580を500に丸める', () => {
      expect(getNiceNumber(580)).toBe(500);
    });

    it('1234を1000に丸める', () => {
      expect(getNiceNumber(1234)).toBe(1000);
    });
  });

  describe('小さな数値', () => {
    it('0.073を0.1に丸める', () => {
      expect(getNiceNumber(0.073)).toBe(0.1);
    });

    it('0.028を0.02に丸める', () => {
      expect(getNiceNumber(0.028)).toBe(0.02);
    });

    it('0.0045を0.005に丸める', () => {
      expect(getNiceNumber(0.0045)).toBe(0.005);
    });
  });

  describe('エッジケース', () => {
    it('0を1にする', () => {
      expect(getNiceNumber(0)).toBe(1);
    });

    it('負の値を正しく処理する', () => {
      expect(getNiceNumber(-37)).toBe(-50);
      expect(getNiceNumber(-142)).toBe(-100);
    });

    it('Infinityを1にする', () => {
      expect(getNiceNumber(Infinity)).toBe(1);
    });

    it('NaNを1にする', () => {
      expect(getNiceNumber(NaN)).toBe(1);
    });
  });

  describe('境界値', () => {
    it('1.49を1に丸める', () => {
      expect(getNiceNumber(1.49)).toBe(1);
    });

    it('1.5を2に丸める', () => {
      expect(getNiceNumber(1.5)).toBe(2);
    });

    it('2.99を2に丸める', () => {
      expect(getNiceNumber(2.99)).toBe(2);
    });

    it('3.0を5に丸める', () => {
      expect(getNiceNumber(3.0)).toBe(5);
    });

    it('6.99を5に丸める', () => {
      expect(getNiceNumber(6.99)).toBe(5);
    });

    it('7.0を10に丸める', () => {
      expect(getNiceNumber(7.0)).toBe(10);
    });
  });
});
