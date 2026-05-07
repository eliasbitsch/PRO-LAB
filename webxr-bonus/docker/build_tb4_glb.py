#!/usr/bin/env python3
"""Build a single GLB of TurtleBot4 from the URDF + DAE/STL meshes shipped
with the nav2_minimal_tb4_description / irobot_create_description packages.

Strategy: parse the xacro -> URDF, walk every <visual> mesh + transform, load
each mesh via trimesh, place it in the world per the parent link transform,
and export everything as one .glb.

Output: /home/ros/web/turtlebot4.glb
"""
from __future__ import annotations
import os
import re
import subprocess
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import trimesh

# --- locate packages ---
SHARE = Path('/opt/ros/jazzy/share')
DESC_PKG = SHARE / 'nav2_minimal_tb4_description'
XACRO = DESC_PKG / 'urdf' / 'standard' / 'turtlebot4.urdf.xacro'
OUTPUT = Path('/home/ros/web/turtlebot4.glb')


def xacro_to_urdf(xacro_path: Path) -> str:
    out = subprocess.run(
        ['ros2', 'run', 'xacro', 'xacro', str(xacro_path)],
        check=True, capture_output=True, text=True,
    )
    return out.stdout


def resolve_pkg_uri(uri: str) -> Path | None:
    m = re.match(r'package://([^/]+)/(.+)', uri)
    if not m:
        return None
    pkg, rel = m.group(1), m.group(2)
    return SHARE / pkg / rel


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def origin_to_T(origin_el):
    T = np.eye(4)
    if origin_el is None:
        return T
    xyz = [float(v) for v in (origin_el.get('xyz', '0 0 0').split())]
    rpy = [float(v) for v in (origin_el.get('rpy', '0 0 0').split())]
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def build_link_transforms(root: ET.Element) -> dict[str, np.ndarray]:
    """Compute world transform of every link by walking the joint tree."""
    joints = root.findall('joint')
    parent_of: dict[str, tuple[str, np.ndarray]] = {}  # child -> (parent, T_parent_child)
    for j in joints:
        parent = j.find('parent').get('link')
        child = j.find('child').get('link')
        T = origin_to_T(j.find('origin'))
        parent_of[child] = (parent, T)

    transforms: dict[str, np.ndarray] = {}

    def world_T(link: str) -> np.ndarray:
        if link in transforms:
            return transforms[link]
        if link not in parent_of:
            transforms[link] = np.eye(4)
            return transforms[link]
        parent, Tpc = parent_of[link]
        Twc = world_T(parent) @ Tpc
        transforms[link] = Twc
        return Twc

    for link_el in root.findall('link'):
        world_T(link_el.get('name'))
    return transforms


def main():
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    print(f'parsing {XACRO}')
    urdf_text = xacro_to_urdf(XACRO)
    root = ET.fromstring(urdf_text)
    transforms = build_link_transforms(root)

    scene = trimesh.Scene()
    skipped = []
    placed = 0
    for link_el in root.findall('link'):
        link = link_el.get('name')
        Twl = transforms.get(link, np.eye(4))
        for vis in link_el.findall('visual'):
            geom = vis.find('geometry')
            mesh_el = geom.find('mesh') if geom is not None else None
            if mesh_el is None:
                continue
            uri = mesh_el.get('filename', '')
            scale = mesh_el.get('scale', '1 1 1')
            sx, sy, sz = (float(v) for v in scale.split())
            mesh_path = resolve_pkg_uri(uri)
            if mesh_path is None or not mesh_path.exists():
                skipped.append((link, uri))
                continue
            try:
                m = trimesh.load(str(mesh_path), force='mesh', process=False)
            except Exception as e:
                skipped.append((link, f'{uri}: {e}'))
                continue
            if m.is_empty:
                skipped.append((link, f'{uri}: empty'))
                continue

            T_local = origin_to_T(vis.find('origin'))
            T_world = Twl @ T_local
            S = np.diag([sx, sy, sz, 1.0])
            T = T_world @ S

            # Apply material color if specified, or set default plastic
            mat_el = vis.find('material')
            color_rgba = None
            if mat_el is not None:
                col = mat_el.find('color')
                if col is not None:
                    rgba = [float(v) for v in col.get('rgba', '0.5 0.5 0.5 1').split()]
                    color_rgba = [int(rgba[0] * 255), int(rgba[1] * 255), int(rgba[2] * 255), int(rgba[3] * 255)]
            if color_rgba is not None and hasattr(m.visual, 'face_colors'):
                try:
                    m.visual.face_colors = color_rgba
                except Exception:
                    pass

            link_low = link.lower()
            wheel_prefix = 'WHEEL__' if ('wheel' in link_low or 'caster' in link_low) else ''
            scene.add_geometry(m, transform=T,
                               geom_name=f'{wheel_prefix}{link}__{Path(uri).name}')
            placed += 1

    if placed == 0:
        print('No meshes placed — aborting', file=sys.stderr)
        sys.exit(1)

    # URDF Z-up -> three.js Y-up. Apply a rotation to the scene root so the
    # GLB exports already aligned for our renderer.
    Y_UP = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
    scene.apply_transform(Y_UP)

    print(f'placed {placed} meshes, skipped {len(skipped)}')
    for s in skipped[:10]:
        print('  skip:', s)
    print(f'writing {OUTPUT}')
    scene.export(str(OUTPUT))
    print('done')


if __name__ == '__main__':
    main()
