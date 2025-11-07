/**
 * @file apiClient.js
 * @description API通信関連の機能を提供するモジュール
 * @requires ../config.js
 * @exports loadOperations
 * @exports executeQuery
 */

import { API_BASE_URL } from '../config.js';

/**
 * 操作カタログを取得
 * @export
 * @returns {Promise<void>}
 */
export async function loadOperations() {
    try {
        const response = await fetch(`${API_BASE_URL}/operations`);
        const data = await response.json();

        const operationsHTML = data.operations.map(op => `
            <div class="operation-item">
                <span class="operation-type">${op.type}</span>
                <h3>${op.name}</h3>
                <div class="operation-details">
                    <p><strong>ID:</strong> ${op.id}</p>
                    <p><strong>位置:</strong> X: ${op.location.x}, Y: ${op.location.y}, Z: ${op.location.z}</p>
                    <p><strong>操作:</strong> ${JSON.stringify(op.operation, null, 2)}</p>
                </div>
            </div>
        `).join('');

        const operationsContent = document.getElementById('operationsContent');
        if (operationsContent) {
            operationsContent.innerHTML = operationsHTML;
        }
    } catch (error) {
        console.error('操作カタログの取得に失敗:', error);
        const operationsContent = document.getElementById('operationsContent');
        if (operationsContent) {
            operationsContent.innerHTML = '<div class="error">操作カタログの取得に失敗しました</div>';
        }
    }
}

/**
 * MapQLクエリを実行
 * @export
 * @returns {Promise<void>}
 */
export async function executeQuery() {
    const queryInput = document.getElementById('queryInput');
    const resultDiv = document.getElementById('queryResult');

    if (!queryInput || !resultDiv) return;

    const query = queryInput.value;

    if (!query) {
        resultDiv.innerHTML = '<div class="error">クエリを入力してください</div>';
        return;
    }

    resultDiv.innerHTML = '<div class="loading">クエリを実行中...</div>';

    try {
        const response = await fetch(`${API_BASE_URL}/mapql/query`, {
            method: 'POST',
            headers: {
                'Content-Type': 'application/json'
            },
            body: JSON.stringify({ query })
        });

        const data = await response.json();

        if (data.result) {
            resultDiv.innerHTML = `
                <div class="success">
                    <h4>クエリ結果:</h4>
                    <pre>${JSON.stringify(data.result, null, 2)}</pre>
                    <p><small>実行時間: ${data.execution_time}</small></p>
                </div>
            `;
        } else {
            resultDiv.innerHTML = `
                <div class="query-result">
                    <p>${data.message || '結果が見つかりませんでした'}</p>
                </div>
            `;
        }
    } catch (error) {
        console.error('クエリの実行に失敗:', error);
        resultDiv.innerHTML = '<div class="error">クエリの実行に失敗しました</div>';
    }
}
