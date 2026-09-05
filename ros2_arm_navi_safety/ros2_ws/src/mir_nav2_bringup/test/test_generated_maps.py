from pathlib import Path
import subprocess

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
ASSETS = PACKAGE.parent / 'amr_simulation_assets'


def read_pgm(path):
    with path.open('rb') as stream:
        assert stream.readline().strip() == b'P5'
        line = stream.readline()
        while line.startswith(b'#'):
            line = stream.readline()
        width, height = [int(value) for value in line.split()]
        assert stream.readline().strip() == b'255'
        pixels = stream.read()
    assert len(pixels) == width * height
    return width, height, pixels


def pixel_at(pixels, width, height, origin, resolution, x, y):
    column = int((x - origin[0]) / resolution)
    map_row = int((y - origin[1]) / resolution)
    image_row = height - 1 - map_row
    assert 0 <= column < width
    assert 0 <= image_row < height
    return pixels[image_row * width + column]


def test_maps_are_reproducible(tmp_path):
    subprocess.run([
        'python3', str(PACKAGE / 'scripts' / 'generate_maps.py'),
        '--assets-root', str(ASSETS), '--output', str(tmp_path),
    ], check=True)
    for map_yaml in sorted((PACKAGE / 'maps').glob('*.yaml')):
        generated = tmp_path / f'{map_yaml.stem}.pgm'
        assert generated.read_bytes() == (PACKAGE / 'maps' / generated.name).read_bytes()


def test_world_spawn_and_example_goal_are_free():
    worlds = yaml.safe_load((PACKAGE / 'config' / 'worlds.yaml').read_text())['worlds']
    for world_name, definition in worlds.items():
        metadata = yaml.safe_load((PACKAGE / 'maps' / definition['map']).read_text())
        image = PACKAGE / 'maps' / metadata['image']
        width, height, pixels = read_pgm(image)
        for pose_name in ('spawn', 'example_goal'):
            x, y, _ = definition[pose_name]
            assert pixel_at(
                pixels, width, height, metadata['origin'], metadata['resolution'], x, y
            ) == 254, f'{world_name} {pose_name} lies in an occupied cell'


def test_nonempty_worlds_contain_static_obstacles():
    for image in sorted((PACKAGE / 'maps').glob('*.pgm')):
        _, _, pixels = read_pgm(image)
        if image.stem == 'empty':
            assert 0 not in pixels
        else:
            assert 0 in pixels
        assert set(pixels) <= {0, 254}
