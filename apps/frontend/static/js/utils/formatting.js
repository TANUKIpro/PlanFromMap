/**
 * @file formatting.js
 * @description 数値フォーマットユーティリティ関数
 *
 * このファイルには、数値を人間にとって読みやすい形式にフォーマットする
 * ユーティリティ関数が含まれています。
 *
 * @exports formatDistance - 距離をメートル、キロメートル、センチメートル、ミリメートルで表示
 * @exports getNiceNumber - 数値を「きれいな」数値（1, 2, 5, 10の倍数）に丸める
 */

/**
 * 距離を適切な単位で人間にとって読みやすい形式にフォーマット
 *
 * この関数は、メートル単位の距離を以下のルールで最適な単位に変換します：
 * - 1000m以上: キロメートル（km）で表示、小数点以下2桁
 * - 1m以上: メートル（m）で表示、10m以上は整数、未満は小数点以下2桁
 * - 1cm以上: センチメートル（cm）で表示、10cm以上は整数、未満は小数点以下1桁
 * - 1cm未満: ミリメートル（mm）で表示、整数
 *
 * @param {number} meters - メートル単位の距離
 * @returns {string} フォーマットされた距離文字列（例: "1.5 km", "25 m", "5.3 cm", "8 mm"）
 *
 * @example
 * formatDistance(1500)      // "1.50 km"
 * formatDistance(25.5)      // "26 m"
 * formatDistance(5.5)       // "5.50 m"
 * formatDistance(0.053)     // "5.3 cm"
 * formatDistance(0.0082)    // "8 mm"
 */
export function formatDistance(meters) {
    if (!isFinite(meters)) return '';
    const absMeters = Math.abs(meters);
    if (absMeters >= 1000) {
        return `${(meters / 1000).toFixed(2)} km`;
    }
    if (absMeters >= 1) {
        const precision = absMeters >= 10 ? 0 : 2;
        return `${meters.toFixed(precision)} m`;
    }
    const centimeters = meters * 100;
    if (Math.abs(centimeters) >= 1) {
        const precision = Math.abs(centimeters) >= 10 ? 0 : 1;
        return `${centimeters.toFixed(precision)} cm`;
    }
    const millimeters = meters * 1000;
    return `${millimeters.toFixed(0)} mm`;
}

/**
 * 数値を「きれいな」数値に丸める
 *
 * この関数は、任意の数値を1, 2, 5, 10の倍数に丸めます。
 * グラフの軸ラベル、スケールバー、グリッド間隔などに使用されます。
 *
 * アルゴリズム：
 * 1. 数値の桁数（10の指数）を計算
 * 2. 仮数部分（1.0～10.0の範囲）を取得
 * 3. 仮数部分を以下のルールで丸める：
 *    - < 1.5 → 1
 *    - < 3.0 → 2
 *    - < 7.0 → 5
 *    - >= 7.0 → 10
 * 4. 元の桁数を復元
 *
 * @param {number} value - 元の数値
 * @returns {number} 丸められた「きれいな」数値（例: 1, 2, 5, 10, 20, 50, 100, ...）
 *
 * @example
 * getNiceNumber(0.73)   // 1
 * getNiceNumber(1.4)    // 1
 * getNiceNumber(2.8)    // 2
 * getNiceNumber(4.5)    // 5
 * getNiceNumber(8.2)    // 10
 * getNiceNumber(37)     // 50
 * getNiceNumber(142)    // 100
 * getNiceNumber(580)    // 500
 */
export function getNiceNumber(value) {
    if (!isFinite(value) || value === 0) return 1;
    const exponent = Math.floor(Math.log10(Math.abs(value)));
    const fraction = Math.abs(value) / Math.pow(10, exponent);
    let niceFraction;
    if (fraction < 1.5) {
        niceFraction = 1;
    } else if (fraction < 3) {
        niceFraction = 2;
    } else if (fraction < 7) {
        niceFraction = 5;
    } else {
        niceFraction = 10;
    }
    return Math.sign(value) * niceFraction * Math.pow(10, exponent);
}
