#!/usr/bin/env python3
"""Build a single GLB of TurtleBot3 (Burger) from the URDF + STL/DAE meshes
shipped in https://github.com/ROBOTIS-GIT/turtlebot3.

Repo is cloned into /tmp/turtlebot3 by the Dockerfile. This script processes
the xacro -> URDF, walks every <visual> mesh + transform and exports a single
GLB at /home/ros/web/turtlebot3.glb.
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

TB3_REPO = Path('/tmp/turtlebot3')
TB3_PKG  = TB3_REPO / 'turtlebot3_description'
XACRO    = TB3_PKG / 'urdf' / 'turtlebot3_burger.urdf'  # Repo ships .urdf with xacro tags
SHARE    = Path('/opt/ros/jazzy/share')
OUTPUT   = Path('/home/ros/web/turtlebot3.glb')


def xacro_to_urdf(xacro_path: Path) -> str:
    """Run xacro with an AMENT overlay so $(find turtlebot3_description) resolves.
    Ament uses a resource_index file (not just symlinks) to enumerate packages,
    so we create that marker too.
    """
    prefix = Path('/tmp/tb3_ament_overlay')
    overlay = prefix / 'share'
    overlay.mkdir(parents=True, exist_ok=True)
    link = overlay / 'turtlebot3_description'
    if not link.exists():
        link.symlink_to(TB3_PKG)
    idx = overlay / 'ament_index' / 'resource_index' / 'packages'
    idx.mkdir(parents=True, exist_ok=True)
    (idx / 'turtlebot3_description').write_text('')

    env = os.environ.copy()
    env['AMENT_PREFIX_PATH'] = str(prefix) + ':' + env.get('AMENT_PREFIX_PATH', '')

    out = subprocess.run(
        ['xacro', str(xacro_path)],
        check=True, capture_output=True, text=True, env=env,
    )
    return out.stdout


def resolve_pkg_uri(uri: str) -> Path | None:
    m = re.match(r'package://([^/]+)/(.+)', uri)
    if not m:
        return None
    pkg, rel = m.group(1), m.group(2)
    if pkg == 'turtlebot3_description':
        return TB3_PKG / rel
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
    parent_of: dict[str, tuple[str, np.ndarray]] = {}
    for j in root.findall('joint'):
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


def collect_named_materials(root: ET.Element) -> dict[str, list[float]]:
    """Build a name -> [r,g,b,a] map from top-level <material> elements."""
    out: dict[str, list[float]] = {}
    for m in root.findall('material'):
        name = m.get('name')
        col = m.find('color')
        if name and col is not None:
            try:
                rgba = [float(v) for v in col.get('rgba', '0.5 0.5 0.5 1').split()]
                if len(rgba) == 4:
                    out[name] = rgba
            except ValueError:
                pass
    return out


def main():
    if not XACRO.exists():
        print(f'TB3 xacro missing at {XACRO} — repo not cloned?', file=sys.stderr)
        sys.exit(2)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    print(f'parsing {XACRO}')
    urdf_text = xacro_to_urdf(XACRO)
    root = ET.fromstring(urdf_text)
    transforms = build_link_transforms(root)
    named_materials = collect_named_materials(root)
    print(f'named materials: {list(named_materials.keys())}')

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

            mat_el = vis.find('material')
            color_rgba = None
            if mat_el is not None:
                col = mat_el.find('color')
                rgba_floats: list[float] | None = None
                if col is not None:
                    rgba_floats = [float(v) for v in col.get('rgba', '0.5 0.5 0.5 1').split()]
                else:
                    # Reference to a named material defined at URDF root
                    name = mat_el.get('name')
                    if name and name in named_materials:
                        rgba_floats = named_materials[name]
                if rgba_floats and len(rgba_floats) == 4:
                    color_rgba = [int(c * 255) for c in rgba_floats]
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
