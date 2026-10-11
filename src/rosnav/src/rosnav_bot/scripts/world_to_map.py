#!/usr/bin/env python3
"""
world_to_map.py -- build a map_server map directly from a Gazebo world.

The world SDF is an exact floor plan, so in simulation it gives a complete,
correct map without exploring first (like a CAD plan of a real warehouse).
AMCL localizing against a complete map is far more robust than against a
SLAM map that is still being built.

Only collision geometry that a robot can hit or its lidar can see is drawn:
everything whose height range overlaps [min_z, max_z] (default 0.0-0.45 m).
Floor markings (thinner than 2 cm), overhead structures (e.g. a catwalk
deck at 2.4 m) and <include>d models (people, props) are left out.

The map frame origin is placed at ``origin_xy`` in world coordinates
(multi_robot.launch.py uses robot1's spawn), so map = world - origin.

    world_to_map.py <world.sdf> <out_prefix> [--origin X Y] [--resolution R]
"""

import argparse
import math
import os
import xml.etree.ElementTree as ET


def _floats(text, n):
    v = [float(x) for x in (text or '').split()]
    return (v + [0.0] * n)[:n]


def world_shapes(world_path, min_z=0.0, max_z=0.45):
    root = ET.parse(os.path.expanduser(world_path)).getroot()
    shapes = []
    for model in root.iter('model'):
        if model.get('name') == 'ground_plane':
            continue
        mx, my, mz, _, _, myaw = _floats(model.findtext('pose'), 6)
        for link in model.findall('link'):
            lx, ly, lz, _, _, lyaw = _floats(link.findtext('pose'), 6)
            for col in link.findall('collision'):
                cx, cy, cz, _, _, cyaw = _floats(col.findtext('pose'), 6)
                geom = col.find('geometry')
                if geom is None:
                    continue
                yaw = myaw + lyaw + cyaw
                # Child poses are relative to the model frame (yaw only).
                ox, oy = lx + cx, ly + cy
                x = mx + ox * math.cos(myaw) - oy * math.sin(myaw)
                y = my + ox * math.sin(myaw) + oy * math.cos(myaw)
                z = mz + lz + cz
                box, cyl = geom.find('box'), geom.find('cylinder')
                if box is not None:
                    sx, sy, sz = _floats(box.findtext('size'), 3)
                    if sz < 0.02 or z + sz / 2 < min_z or z - sz / 2 > max_z:
                        continue
                    shapes.append(('box', x, y, yaw, sx, sy))
                elif cyl is not None:
                    r = float(cyl.findtext('radius', '0'))
                    length = float(cyl.findtext('length', '0'))
                    if length < 0.02 or z + length / 2 < min_z or z - length / 2 > max_z:
                        continue
                    shapes.append(('cyl', x, y, r))
    return shapes


def _extent(s):
    """Half extents (ex, ey) of a shape's axis-aligned bounding box."""
    if s[0] == 'box':
        _, _, _, yaw, sx, sy = s
        c, sn = abs(math.cos(yaw)), abs(math.sin(yaw))
        return sx / 2 * c + sy / 2 * sn, sx / 2 * sn + sy / 2 * c
    return s[3], s[3]


def rasterize(shapes, resolution=0.05, margin=0.5):
    xs, ys = [], []
    for s in shapes:
        ex, ey = _extent(s)
        xs += [s[1] - ex, s[1] + ex]
        ys += [s[2] - ey, s[2] + ey]
    min_x, max_x = min(xs) - margin, max(xs) + margin
    min_y, max_y = min(ys) - margin, max(ys) + margin
    w = int(math.ceil((max_x - min_x) / resolution))
    h = int(math.ceil((max_y - min_y) / resolution))
    occ = bytearray(w * h)
    for s in shapes:
        if s[0] == 'box':
            _, x, y, yaw, sx, sy = s
            c, sn = math.cos(yaw), math.sin(yaw)
            test = lambda px, py: (abs((px - x) * c + (py - y) * sn) <= sx / 2 and
                                   abs(-(px - x) * sn + (py - y) * c) <= sy / 2)
        else:
            _, x, y, r = s
            test = lambda px, py: math.hypot(px - x, py - y) <= r
        ex, ey = _extent(s)
        i0 = max(0, int((x - ex - min_x) / resolution))
        i1 = min(w - 1, int((x + ex - min_x) / resolution) + 1)
        j0 = max(0, int((y - ey - min_y) / resolution))
        j1 = min(h - 1, int((y + ey - min_y) / resolution) + 1)
        for j in range(j0, j1 + 1):
            py = min_y + (j + 0.5) * resolution
            for i in range(i0, i1 + 1):
                if test(min_x + (i + 0.5) * resolution, py):
                    occ[j * w + i] = 1
    return occ, w, h, min_x, min_y


def write_map(out_prefix, occ, w, h, min_x, min_y, resolution, origin_xy):
    os.makedirs(os.path.dirname(out_prefix) or '.', exist_ok=True)
    pgm = out_prefix + '.pgm'
    with open(pgm, 'wb') as f:
        f.write(f'P5\n{w} {h}\n255\n'.encode())
        # PGM rows go top to bottom; map rows go bottom (min_y) to top.
        for j in range(h - 1, -1, -1):
            f.write(bytes(0 if occ[j * w + i] else 254 for i in range(w)))
    yaml_path = out_prefix + '.yaml'
    with open(yaml_path, 'w') as f:
        f.write(f'image: {os.path.basename(pgm)}\n'
                f'mode: trinary\nresolution: {resolution}\n'
                f'origin: [{min_x - origin_xy[0]:.3f}, {min_y - origin_xy[1]:.3f}, 0.0]\n'
                f'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n')
    return yaml_path


def build(world_path, out_prefix, origin_xy=(0.0, 0.0), resolution=0.05):
    occ, w, h, min_x, min_y = rasterize(world_shapes(world_path), resolution)
    return write_map(out_prefix, occ, w, h, min_x, min_y, resolution, origin_xy)


def main():
    ap = argparse.ArgumentParser(description=__doc__.split('\n\n')[0])
    ap.add_argument('world')
    ap.add_argument('out_prefix')
    ap.add_argument('--origin', nargs=2, type=float, default=(0.0, 0.0))
    ap.add_argument('--resolution', type=float, default=0.05)
    a = ap.parse_args()
    print(build(a.world, a.out_prefix, tuple(a.origin), a.resolution))


if __name__ == '__main__':
    main()
