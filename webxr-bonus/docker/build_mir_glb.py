#!/usr/bin/env python3
"""Build a single GLB of MiR 100 from the URDF + meshes in
https://github.com/DFKI-NI/mir_robot (mir_description package).

Repo is cloned into /tmp/mir_robot by the Dockerfile.
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

MIR_REPO = Path('/tmp/mir_robot')
MIR_PKG  = MIR_REPO / 'mir_description'
# DFKI-NI/mir_robot ships several xacros — use the 100 model.
XACRO_CANDIDATES = [
    MIR_PKG / 'urdf' / 'mir.urdf.xacro',
    MIR_PKG / 'urdf' / 'mir_100' / 'mir_100.urdf.xacro',
    MIR_PKG / 'urdf' / 'mir100.urdf.xacro',
]
SHARE  = Path('/opt/ros/jazzy/share')
OUTPUT = Path('/home/ros/web/mir100.glb')


def find_xacro() -> Path:
    for p in XACRO_CANDIDATES:
        if p.exists():
            return p
    # last-resort: any *.urdf.xacro under mir_description/urdf
    for p in MIR_PKG.glob('urdf/**/*.urdf.xacro'):
        return p
    raise FileNotFoundError('No mir xacro found')


def xacro_to_urdf(xacro_path: Path) -> str:
    prefix = Path('/tmp/mir_ament_overlay')
    overlay = prefix / 'share'
    overlay.mkdir(parents=True, exist_ok=True)
    link = overlay / 'mir_description'
    if not link.exists():
        link.symlink_to(MIR_PKG)
    idx = overlay / 'ament_index' / 'resource_index' / 'packages'
    idx.mkdir(parents=True, exist_ok=True)
    (idx / 'mir_description').write_text('')

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
    if pkg == 'mir_description':
        return MIR_PKG / rel
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
        p = j.find('parent'); c = j.find('child')
        if p is None or c is None: continue
        T = origin_to_T(j.find('origin'))
        parent_of[c.get('link')] = (p.get('link'), T)
    transforms: dict[str, np.ndarray] = {}
    def world_T(link: str) -> np.ndarray:
        if link in transforms: return transforms[link]
        if link not in parent_of:
            transforms[link] = np.eye(4); return transforms[link]
        parent, Tpc = parent_of[link]
        Twc = world_T(parent) @ Tpc
        transforms[link] = Twc
        return Twc
    for link_el in root.findall('link'):
        world_T(link_el.get('name'))
    return transforms


def primitive_mesh(geom_el):
    """Build a trimesh primitive from <cylinder>/<box>/<sphere>. Returns None on no-match."""
    cyl = geom_el.find('cylinder')
    if cyl is not None:
        r = float(cyl.get('radius', '0.05'))
        h = float(cyl.get('length', '0.1'))
        return trimesh.creation.cylinder(radius=r, height=h, sections=24)
    box = geom_el.find('box')
    if box is not None:
        sx, sy, sz = (float(v) for v in box.get('size', '0.1 0.1 0.1').split())
        return trimesh.creation.box(extents=(sx, sy, sz))
    sph = geom_el.find('sphere')
    if sph is not None:
        return trimesh.creation.icosphere(subdivisions=2,
                                          radius=float(sph.get('radius', '0.05')))
    return None


def main():
    xacro = find_xacro()
    print(f'parsing {xacro}')
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    urdf_text = xacro_to_urdf(xacro)
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
            if geom is None:
                continue
            mesh_el = geom.find('mesh')
            m = None
            sx = sy = sz = 1.0
            if mesh_el is not None:
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
            else:
                # Primitive geometry (cylinder/box/sphere)
                m = primitive_mesh(geom)
                if m is None:
                    continue
            if m is None or m.is_empty:
                continue
            T_local = origin_to_T(vis.find('origin'))
            T_world = Twl @ T_local
            S = np.diag([sx, sy, sz, 1.0])
            T = T_world @ S
            mat_el = vis.find('material')
            color_rgba = None
            if mat_el is not None:
                col = mat_el.find('color')
                if col is not None:
                    rgba = [float(v) for v in col.get('rgba', '0.5 0.5 0.5 1').split()]
                    color_rgba = [int(rgba[0]*255), int(rgba[1]*255), int(rgba[2]*255), int(rgba[3]*255)]
            if color_rgba is not None:
                try:
                    m.visual = trimesh.visual.color.ColorVisuals(mesh=m, face_colors=color_rgba)
                except Exception:
                    pass
            tag = mesh_el.get('filename', 'primitive') if mesh_el is not None else 'primitive'
            link_low = link.lower()
            wheel_prefix = 'WHEEL__' if ('wheel' in link_low or 'caster' in link_low) else ''
            scene.add_geometry(m, transform=T,
                               geom_name=f'{wheel_prefix}{link}__{Path(tag).name}')
            placed += 1

    # ── Cosmetic blue LED stripes (signature MiR look) ──
    # Two thin blue boxes along the long sides of the chassis at top edge.
    led_blue = [40, 130, 255, 255]
    body_len   = 0.85   # X
    body_width = 0.55   # Y
    led_h      = 0.025
    led_z      = 0.30   # near top of the body
    led_length = 0.40   # absolute strip length in meters (shorter than body)
    for y_side in (+body_width / 2 + 0.005, -body_width / 2 - 0.005):
        led = trimesh.creation.box(extents=(led_length, 0.004, led_h))
        led.visual = trimesh.visual.color.ColorVisuals(mesh=led, face_colors=led_blue)
        T = np.eye(4); T[:3, 3] = [0.0, y_side, led_z]
        scene.add_geometry(led, transform=T, geom_name=f'led_side_{y_side:+.3f}')
        placed += 1

    if placed == 0:
        print('No meshes placed — aborting', file=sys.stderr)
        sys.exit(1)

    Y_UP = trimesh.transformations.rotation_matrix(-np.pi/2, [1,0,0])
    scene.apply_transform(Y_UP)

    print(f'placed {placed} meshes, skipped {len(skipped)}')
    for s in skipped[:15]:
        print('  skip:', s)
    print(f'writing {OUTPUT}')
    scene.export(str(OUTPUT))
    print('done')


if __name__ == '__main__':
    main()
