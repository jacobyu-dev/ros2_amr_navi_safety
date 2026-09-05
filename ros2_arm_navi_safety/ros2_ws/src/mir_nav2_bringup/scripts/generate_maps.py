#!/usr/bin/env python3
"""Generate deterministic Nav2 occupancy maps from static SDF box collisions."""

import argparse
import math
from pathlib import Path
import xml.etree.ElementTree as ET


WORLD_SPECS = {
    'empty': ('empty/empty.sdf', (-25.0, -25.0, 50.0, 50.0)),
    'maze': ('mir_maze/maze.sdf', (-11.0, -11.0, 22.0, 22.0)),
    'corridor': ('corridor/corridor.sdf', (-11.0, -9.0, 22.0, 18.0)),
    'bookstore': ('bookstore/bookstore.sdf', (-16.0, -12.0, 32.0, 24.0)),
    'warehouse': ('warehouse/warehouse.sdf', (-10.0, -8.0, 20.0, 16.0)),
    'warehouse_detailed': (
        'warehouse_detailed/warehouse_detailed.sdf', (-16.0, -11.0, 32.0, 22.0)),
}
RESOLUTION = 0.05


def parse_pose(element):
    pose = element.find('pose')
    values = [0.0] * 6 if pose is None or not pose.text else [float(v) for v in pose.text.split()]
    values += [0.0] * (6 - len(values))
    return values[0], values[1], values[2], values[5]


def compose(parent, child):
    px, py, pz, pyaw = parent
    cx, cy, cz, cyaw = child
    cosine = math.cos(pyaw)
    sine = math.sin(pyaw)
    return (
        px + cosine * cx - sine * cy,
        py + sine * cx + cosine * cy,
        pz + cz,
        pyaw + cyaw,
    )


def collect_model_boxes(model, parent_pose=(0.0, 0.0, 0.0, 0.0)):
    model_pose = compose(parent_pose, parse_pose(model))
    boxes = []
    for link in model.findall('link'):
        link_pose = compose(model_pose, parse_pose(link))
        for collision in link.findall('collision'):
            box = collision.find('./geometry/box/size')
            if box is None or not box.text:
                continue
            sx, sy, sz = [float(v) for v in box.text.split()]
            collision_pose = compose(link_pose, parse_pose(collision))
            z_min = collision_pose[2] - sz / 2.0
            z_max = collision_pose[2] + sz / 2.0
            # Ignore floors and geometry entirely above the robot body.
            if z_max <= 0.05 or z_min >= 1.50:
                continue
            boxes.append((collision_pose[0], collision_pose[1], collision_pose[3], sx, sy))
    for nested_model in model.findall('model'):
        boxes.extend(collect_model_boxes(nested_model, model_pose))
    return boxes


def collect_world_boxes(world_file, models_root):
    world = ET.parse(world_file).getroot().find('world')
    if world is None:
        raise ValueError(f'No <world> in {world_file}')
    boxes = []
    for model in world.findall('model'):
        if model.findtext('static', default='false').strip().lower() == 'true':
            boxes.extend(collect_model_boxes(model))
    for include in world.findall('include'):
        uri = include.findtext('uri', default='')
        if not uri.startswith('model://'):
            continue
        included_model = models_root / uri.removeprefix('model://') / 'model.sdf'
        if not included_model.exists():
            raise FileNotFoundError(f'Cannot resolve {uri} from {world_file}')
        model = ET.parse(included_model).getroot().find('model')
        if model is None:
            raise ValueError(f'No <model> in {included_model}')
        boxes.extend(collect_model_boxes(model, parse_pose(include)))
    return boxes


def rasterize(boxes, bounds, resolution):
    origin_x, origin_y, width_m, height_m = bounds
    width = round(width_m / resolution)
    height = round(height_m / resolution)
    pixels = bytearray([254]) * (width * height)
    for center_x, center_y, yaw, size_x, size_y in boxes:
        half_x = size_x / 2.0
        half_y = size_y / 2.0
        extent_x = abs(math.cos(yaw)) * half_x + abs(math.sin(yaw)) * half_y
        extent_y = abs(math.sin(yaw)) * half_x + abs(math.cos(yaw)) * half_y
        min_col = max(0, math.floor((center_x - extent_x - origin_x) / resolution))
        max_col = min(width - 1, math.ceil((center_x + extent_x - origin_x) / resolution))
        min_row = max(0, math.floor((center_y - extent_y - origin_y) / resolution))
        max_row = min(height - 1, math.ceil((center_y + extent_y - origin_y) / resolution))
        cosine = math.cos(yaw)
        sine = math.sin(yaw)
        for map_row in range(min_row, max_row + 1):
            world_y = origin_y + (map_row + 0.5) * resolution
            image_row = height - 1 - map_row
            for col in range(min_col, max_col + 1):
                world_x = origin_x + (col + 0.5) * resolution
                dx = world_x - center_x
                dy = world_y - center_y
                local_x = cosine * dx + sine * dy
                local_y = -sine * dx + cosine * dy
                if abs(local_x) <= half_x and abs(local_y) <= half_y:
                    pixels[image_row * width + col] = 0
    return width, height, pixels


def write_pgm(path, width, height, pixels):
    with path.open('wb') as output:
        header = (
            'P5\n# Generated from Gazebo static collision geometry\n'
            f'{width} {height}\n255\n')
        output.write(header.encode())
        output.write(pixels)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--assets-root', required=True, type=Path)
    parser.add_argument('--output', required=True, type=Path)
    args = parser.parse_args()
    worlds_root = args.assets_root / 'worlds'
    models_root = args.assets_root / 'models'
    args.output.mkdir(parents=True, exist_ok=True)
    for world_name, (relative_path, bounds) in WORLD_SPECS.items():
        boxes = collect_world_boxes(worlds_root / relative_path, models_root)
        width, height, pixels = rasterize(boxes, bounds, RESOLUTION)
        output_path = args.output / f'{world_name}.pgm'
        write_pgm(output_path, width, height, pixels)
        occupied = sum(pixel == 0 for pixel in pixels)
        print(f'{world_name}: {width}x{height}, boxes={len(boxes)}, occupied={occupied}')


if __name__ == '__main__':
    main()
