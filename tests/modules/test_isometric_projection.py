import json
import subprocess
import unittest
from pathlib import Path


PROJECT_ROOT = Path(__file__).resolve().parents[2]
MODULE_PATH = './apps/frontend/static/js/utils/isometricProjection.js'


def run_node(script: str) -> str:
    """Execute a Node.js script with module support and return stdout."""
    result = subprocess.run(
        [
            'node',
            '--input-type=module',
            '-e',
            script
        ],
        cwd=PROJECT_ROOT,
        capture_output=True,
        text=True,
        check=True
    )
    return result.stdout.strip()


class TestIsometricProjection(unittest.TestCase):
    def test_project_point_basic(self):
        script = f"""
        const module = await import('{MODULE_PATH}');
        const {{ createCameraParams, projectPoint }} = module;
        const params = createCameraParams({{ rotation: 0, tilt: 30 }}, {{ center: {{ x: 0, y: 0 }} }});
        const result = projectPoint({{ x: 1, y: 0, z: 0 }}, params);
        console.log(JSON.stringify(result));
        """
        output = run_node(script)
        data = json.loads(output)
        self.assertAlmostEqual(data['camera']['x'], 1.0, places=6)
        self.assertAlmostEqual(data['camera']['y'], 0.0, places=6)
        self.assertAlmostEqual(data['camera']['z'], 0.0, places=6)
        self.assertAlmostEqual(data['screen']['x'], -1.0, places=6)
        self.assertAlmostEqual(data['screen']['y'], 0.0, places=6)

    def test_face_visibility(self):
        script = f"""
        const module = await import('{MODULE_PATH}');
        const {{ createCameraParams, projectVertices, buildFaceRenderList, filterVisibleFaces }} = module;
        const params = createCameraParams({{ rotation: 45, tilt: 30 }}, {{ center: {{ x: 0, y: 0 }} }});
        const vertices = [
            {{ x: -0.5, y: -0.5, z: -0.5 }},
            {{ x: 0.5, y: -0.5, z: -0.5 }},
            {{ x: 0.5, y: 0.5, z: -0.5 }},
            {{ x: -0.5, y: 0.5, z: -0.5 }},
            {{ x: -0.5, y: -0.5, z: 0.5 }},
            {{ x: 0.5, y: -0.5, z: 0.5 }},
            {{ x: 0.5, y: 0.5, z: 0.5 }},
            {{ x: -0.5, y: 0.5, z: 0.5 }}
        ];
        const {{ cameraVertices }} = projectVertices(vertices, params);
        const faces = filterVisibleFaces(buildFaceRenderList(cameraVertices));
        console.log(JSON.stringify(faces.map(face => face.name)));
        """
        output = run_node(script)
        names = json.loads(output)
        self.assertIn('south', names)
        self.assertIn('west', names)
        self.assertNotIn('north', names)
        self.assertNotIn('east', names)

    def test_face_shading_range(self):
        script = f"""
        const module = await import('{MODULE_PATH}');
        const {{ calculateFaceShade }} = module;
        const normals = [
            {{ x: 0, y: -1, z: 0 }},
            {{ x: 0, y: 0, z: 1 }},
            {{ x: 1, y: 0, z: 0 }}
        ];
        const shades = normals.map(n => calculateFaceShade(n));
        console.log(JSON.stringify(shades));
        """
        output = run_node(script)
        shades = json.loads(output)
        for value in shades:
            self.assertGreaterEqual(value, -20.0)
            self.assertLessEqual(value, 20.0)


if __name__ == '__main__':
    unittest.main()
