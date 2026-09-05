#!/usr/bin/env python3
"""Compare clear/obstacle planned-path CSVs geometrically."""

import argparse
import csv
import json
import math


def read_path(filename):
    with open(filename, newline='', encoding='utf-8') as stream:
        return [(float(row['x']), float(row['y'])) for row in csv.DictReader(stream)]


def distance(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])


def point_to_segment(point, start, end):
    dx = end[0] - start[0]
    dy = end[1] - start[1]
    length_squared = dx * dx + dy * dy
    if length_squared == 0.0:
        return distance(point, start)
    ratio = max(0.0, min(1.0, ((point[0] - start[0]) * dx +
                                (point[1] - start[1]) * dy) / length_squared))
    return distance(point, (start[0] + ratio * dx, start[1] + ratio * dy))


def point_to_path(point, path):
    if len(path) == 1:
        return distance(point, path[0])
    return min(point_to_segment(point, path[index - 1], path[index])
               for index in range(1, len(path)))


def path_length(path):
    return sum(distance(path[index - 1], path[index]) for index in range(1, len(path)))


def directed_deviation(source, target):
    return max(point_to_path(point, target) for point in source)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('clear_path_csv')
    parser.add_argument('obstacle_path_csv')
    parser.add_argument('--threshold', type=float, default=0.20)
    args = parser.parse_args()
    clear = read_path(args.clear_path_csv)
    obstacle = read_path(args.obstacle_path_csv)
    if len(clear) < 2 or len(obstacle) < 2:
        raise SystemExit('both files must contain at least two path poses')
    deviation = max(directed_deviation(clear, obstacle), directed_deviation(obstacle, clear))
    result = {
        'clear_path_length': path_length(clear),
        'obstacle_path_length': path_length(obstacle),
        'symmetric_geometry_deviation': deviation,
        'change_threshold': args.threshold,
        'meaningfully_changed': deviation > args.threshold,
    }
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['meaningfully_changed'] else 1


if __name__ == '__main__':
    raise SystemExit(main())
