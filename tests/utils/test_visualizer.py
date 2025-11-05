"""
Test Result Visualizer
テスト結果の可視化ユーティリティ
"""
import json
from pathlib import Path
from datetime import datetime
from typing import Dict, List, Any


class TestVisualizer:
    """テスト結果を可視化するクラス"""

    def __init__(self, output_dir: Path):
        """
        初期化

        Args:
            output_dir: 出力ディレクトリ
        """
        self.output_dir = output_dir
        self.output_dir.mkdir(exist_ok=True, parents=True)

    def generate_summary_report(self, test_results: List[Dict[str, Any]],
                                output_filename: str = "test_summary.md"):
        """
        テスト結果のサマリーレポートを生成

        Args:
            test_results: テスト結果のリスト
            output_filename: 出力ファイル名
        """
        report_path = self.output_dir / output_filename

        # 統計情報を計算
        total = len(test_results)
        passed = sum(1 for r in test_results if r.get('status') == 'passed')
        failed = sum(1 for r in test_results if r.get('status') == 'failed')
        skipped = sum(1 for r in test_results if r.get('status') == 'skipped')

        total_duration = sum(r.get('duration', 0) for r in test_results)

        # マークダウンレポートを作成
        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("# Test Summary Report\n\n")
            f.write(f"Generated: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n\n")

            f.write("## Statistics\n\n")
            f.write(f"- **Total Tests**: {total}\n")
            f.write(f"- **Passed**: {passed} ({passed/total*100:.1f}%)\n")
            f.write(f"- **Failed**: {failed} ({failed/total*100:.1f}%)\n")
            f.write(f"- **Skipped**: {skipped} ({skipped/total*100:.1f}%)\n")
            f.write(f"- **Total Duration**: {total_duration:.2f}s\n\n")

            # 成功率のバーチャート（ASCII）
            passed_bar = '█' * int(passed / total * 50)
            failed_bar = '█' * int(failed / total * 50)
            f.write("## Pass Rate\n\n")
            f.write(f"```\nPassed: {passed_bar} {passed}/{total}\n")
            f.write(f"Failed: {failed_bar} {failed}/{total}\n```\n\n")

            # 詳細な結果
            f.write("## Test Results\n\n")
            f.write("| Test Name | Status | Duration |\n")
            f.write("|-----------|--------|----------|\n")

            for result in test_results:
                name = result.get('test_name', 'Unknown')
                status = result.get('status', 'unknown')
                duration = result.get('duration', 0)

                status_emoji = {
                    'passed': '✅',
                    'failed': '❌',
                    'skipped': '⏭️'
                }.get(status, '❓')

                f.write(f"| {name} | {status_emoji} {status} | {duration:.3f}s |\n")

            f.write("\n")

        return report_path

    def generate_json_report(self, test_results: List[Dict[str, Any]],
                            output_filename: str = "test_results.json"):
        """
        テスト結果のJSONレポートを生成

        Args:
            test_results: テスト結果のリスト
            output_filename: 出力ファイル名
        """
        report_path = self.output_dir / output_filename

        report_data = {
            "generated_at": datetime.now().isoformat(),
            "summary": {
                "total": len(test_results),
                "passed": sum(1 for r in test_results if r.get('status') == 'passed'),
                "failed": sum(1 for r in test_results if r.get('status') == 'failed'),
                "skipped": sum(1 for r in test_results if r.get('status') == 'skipped'),
                "total_duration": sum(r.get('duration', 0) for r in test_results)
            },
            "results": test_results
        }

        with open(report_path, 'w', encoding='utf-8') as f:
            json.dump(report_data, f, indent=2, ensure_ascii=False)

        return report_path

    def generate_profile_visualization(self, profile_data: Dict[str, Any],
                                       output_filename: str = "profile_viz.md"):
        """
        プロファイルデータの可視化

        Args:
            profile_data: プロファイルデータ
            output_filename: 出力ファイル名
        """
        report_path = self.output_dir / output_filename

        with open(report_path, 'w', encoding='utf-8') as f:
            f.write("# Profile Visualization\n\n")

            # 基本情報
            f.write("## Basic Information\n\n")
            f.write(f"- **Profile Name**: {profile_data.get('profile_name', 'N/A')}\n")
            f.write(f"- **Created At**: {profile_data.get('created_at', 'N/A')}\n")
            f.write(f"- **Version**: {profile_data.get('version', 'N/A')}\n\n")

            # マップデータ
            if 'map_data' in profile_data and profile_data['map_data']:
                f.write("## Map Data\n\n")
                map_data = profile_data['map_data']

                if 'metadata' in map_data:
                    metadata = map_data['metadata']
                    f.write(f"- **Resolution**: {metadata.get('resolution', 'N/A')} m/pixel\n")
                    f.write(f"- **Origin**: {metadata.get('origin', 'N/A')}\n")

            # レイヤー情報
            if 'layers' in profile_data:
                f.write("## Layers\n\n")
                layers = profile_data['layers']
                f.write(f"Total Layers: {len(layers)}\n\n")

                f.write("| ID | Name | Type | Visible |\n")
                f.write("|----|------|------|----------|\n")

                for layer in layers:
                    layer_id = layer.get('id', 'N/A')
                    name = layer.get('name', 'N/A')
                    layer_type = layer.get('type', 'N/A')
                    visible = '✓' if layer.get('visible', False) else '✗'

                    f.write(f"| {layer_id} | {name} | {layer_type} | {visible} |\n")

                f.write("\n")

            # ビューポート情報
            if 'viewport' in profile_data:
                f.write("## Viewport\n\n")
                viewport = profile_data['viewport']
                f.write(f"- **Scale**: {viewport.get('scale', 1.0)}x\n")
                f.write(f"- **Offset X**: {viewport.get('offsetX', 0)}px\n")
                f.write(f"- **Offset Y**: {viewport.get('offsetY', 0)}px\n\n")

        return report_path


def create_test_summary(test_reports_dir: Path,
                       test_results: List[Dict[str, Any]]):
    """
    テストサマリーを作成する便利関数

    Args:
        test_reports_dir: レポート出力ディレクトリ
        test_results: テスト結果のリスト
    """
    visualizer = TestVisualizer(test_reports_dir)

    # マークダウンサマリー
    md_path = visualizer.generate_summary_report(test_results)
    print(f"✅ Markdown report generated: {md_path}")

    # JSONレポート
    json_path = visualizer.generate_json_report(test_results)
    print(f"✅ JSON report generated: {json_path}")

    return {
        "markdown": md_path,
        "json": json_path
    }
