#!/usr/bin/env python3
"""Build a Taurob Tracker GLB from the Sieglinde_2.0 repo
(https://github.com/TW-Robotics/Sieglinde_2.0).

The full tracker.urdf.xacro depends on internal taurob/hector packages we don't
ship, so this script bypasses xacro:
  * Loads tracker_chassis.dae directly (mesh that includes tracks)
  * Parses taurob_arm.urdf as plain URDF, walks its joint tree
  * Mounts the arm on top of the chassis at a hard-coded offset

Output: /home/ros/web/taurob.glb
"""
from __future__ import annotations
import math
import re
import sys
from pathlib import Path
from xml.etree import ElementTree as ET

import numpy as np
import trimesh

REPO    = Path('/tmp/sieglinde')
PKG     = REPO / 'sieglinde_new'
CHASSIS = PKG / 'meshes' / 'chassis_tracker' / 'tracker_chassis.dae'
ARM_URDF     = PKG / 'urdf' / 'arm' / 'taurob_arm.urdf'
GRIPPER_URDF = PKG / 'urdf' / 'gripper' / 'sieglindeGripper.urdf'
OUTPUT  = Path('/home/ros/web/taurob.glb')

# Where to mount the arm base on the chassis (chassis frame, Z-up before final flip).
# RPY taken from the (commented-out) dummyjoint in taurob_arm.urdf that connects
# arm_flange_link -> base_arm in the full tracker description.
ARM_MOUNT_XYZ = (-0.05, 0.0, 0.00)    # tuned mount height (chassis-top-ish)
# Stowed-pose orientation: arm points straight up from chassis (compact).
# This matches the (commented-out) dummyjoint in taurob_arm.urdf.
ARM_MOUNT_RPY = (-1.5708, 0.0, 3.141)

# Color overrides (regardless of what the URDF/DAE specifies)
CHASSIS_COLOR = [240, 240, 240, 255]   # white (body)
TRACK_COLOR   = [25, 25, 25, 255]      # black (tracks + wheels)
ARM_COLOR     = [25, 25, 25, 255]      # black (arm)

# In tracker_chassis.dae, these sub-mesh names are the body shell.
# Everything else in the file is tracks / wheels / sprockets.
BODY_SUBMESHES = {'Shape90_001-mesh', 'Cylinder-mesh'}

# 6 sprocket/idler discs (3 per track). The Shape641 series is the track
# chains — these are the actual disks inside the tracks, identified by
# their 0.22 × 0.03 × 0.22 m disk-like extents (thin in Y = axle direction).
WHEEL_SUBMESHES = {
    'Shape711_000-mesh',  # front left
    'Shape721_000-mesh',  # front right
    'Shape211_000-mesh',  # middle left
    'Shape231_000-mesh',  # middle right
    'Shape221_000-mesh',  # rear left
    'Shape241_000-mesh',  # rear right
}


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


_EVAL_NS = {'pi': math.pi, 'PI': math.pi}
_EXPR_RE = re.compile(r'\$\{([^}]+)\}')

def _expr_to_float(token: str) -> float:
    """Resolve xacro-style ${expr} or plain numeric token to float."""
    m = _EXPR_RE.fullmatch(token.strip())
    if m:
        return float(eval(m.group(1), {'__builtins__': {}}, _EVAL_NS))
    return float(token)


def origin_to_T(origin_el):
    T = np.eye(4)
    if origin_el is None:
        return T
    xyz = [_expr_to_float(v) for v in (origin_el.get('xyz', '0 0 0').split())]
    rpy = [_expr_to_float(v) for v in (origin_el.get('rpy', '0 0 0').split())]
    T[:3, :3] = rpy_to_matrix(rpy)
    T[:3, 3] = xyz
    return T


def resolve_pkg_uri(uri: str) -> Path | None:
    m = re.match(r'package://([^/]+)/(.+)', uri)
    if not m:
        return None
    pkg, rel = m.group(1), m.group(2)
    if pkg == 'sieglinde_new':
        return PKG / rel
    return None


def build_link_transforms(root: ET.Element) -> dict[str, np.ndarray]:
    parent_of: dict[str, tuple[str, np.ndarray]] = {}
    for j in root.findall('joint'):
        p = j.find('parent'); c = j.find('child')
        if p is None or c is None:
            continue
        parent_of[c.get('link')] = (p.get('link'), origin_to_T(j.find('origin')))
    transforms: dict[str, np.ndarray] = {}
    def world_T(link: str) -> np.ndarray:
        if link in transforms:
            return transforms[link]
        if link not in parent_of:
            transforms[link] = np.eye(4); return transforms[link]
        parent, Tpc = parent_of[link]
        Twc = world_T(parent) @ Tpc
        transforms[link] = Twc
        return Twc
    for link_el in root.findall('link'):
        world_T(link_el.get('name'))
    return transforms


def add_visuals(scene: trimesh.Scene, root: ET.Element, base_T: np.ndarray,
                prefix: str, mesh_scale: float = 1.0):
    transforms = build_link_transforms(root)
    placed = 0
    skipped = []
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
                continue
            T_local = origin_to_T(vis.find('origin'))
            # mesh_scale handles mm-authored STLs that the URDF doesn't scale.
            S = np.diag([sx * mesh_scale, sy * mesh_scale, sz * mesh_scale, 1.0])
            T = base_T @ Twl @ T_local @ S

            # Force arm color (replace visuals entirely so URDF/STL material loses)
            m.visual = trimesh.visual.color.ColorVisuals(
                mesh=m, face_colors=ARM_COLOR,
            )

            scene.add_geometry(m, transform=T,
                               geom_name=f'{prefix}_{link}__{Path(uri).name}')
            placed += 1
    return placed, skipped


def main():
    if not CHASSIS.exists() or not ARM_URDF.exists():
        print(f'Missing assets: chassis={CHASSIS.exists()} arm={ARM_URDF.exists()}',
              file=sys.stderr)
        sys.exit(2)
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)

    scene = trimesh.Scene()

    # Chassis (DAE in Z-up). Load as Scene to keep sub-meshes; resolve each
    # sub-mesh's graph transform (otherwise body/tracks land at wrong scales)
    # by walking the scene graph and pre-multiplying transforms into vertices.
    print(f'loading chassis {CHASSIS}')
    chassis_scene = trimesh.load(str(CHASSIS))
    if hasattr(chassis_scene, 'graph'):
        for node_name in chassis_scene.graph.nodes_geometry:
            T_node, geom_name = chassis_scene.graph[node_name]
            sub = chassis_scene.geometry[geom_name].copy()
            sub.apply_transform(T_node)

            if geom_name in BODY_SUBMESHES:
                color = CHASSIS_COLOR
                gname = f'chassis__{geom_name}'
                T_extra = None
            elif geom_name in WHEEL_SUBMESHES:
                color = TRACK_COLOR
                # Center the wheel vertices so rotation in three.js pivots
                # around the wheel center (instead of the chassis origin).
                centroid = sub.centroid.copy()
                sub.apply_translation(-centroid)
                T_extra = np.eye(4); T_extra[:3, 3] = centroid
                # +Y in DAE = bot's left side (these disks split cleanly here).
                side = 'left' if centroid[1] >= 0 else 'right'
                gname = f'WHEEL__{side}_{geom_name}'
            else:
                color = TRACK_COLOR
                gname = f'chassis__{geom_name}'
                T_extra = None

            sub.visual = trimesh.visual.color.ColorVisuals(mesh=sub, face_colors=color)
            if T_extra is not None:
                scene.add_geometry(sub, transform=T_extra, geom_name=gname)
            else:
                scene.add_geometry(sub, geom_name=gname)
    else:
        chassis_scene.visual = trimesh.visual.color.ColorVisuals(
            mesh=chassis_scene, face_colors=CHASSIS_COLOR,
        )
        scene.add_geometry(chassis_scene, geom_name='chassis_tracker')

    # Snapshot CHASSIS-ONLY bounds before adding the arm — the arm sticks
    # out way past the chassis and would skew the recentering offset.
    chassis_bounds = scene.bounds.copy() if scene.bounds is not None else None

    # Arm + gripper: merge both URDFs (gripper references arm_link_3 from arm).
    print(f'parsing arm urdf {ARM_URDF}')
    arm_root = ET.fromstring(ARM_URDF.read_text())
    if GRIPPER_URDF.exists():
        print(f'merging gripper urdf {GRIPPER_URDF}')
        gripper_root = ET.fromstring(GRIPPER_URDF.read_text())
        # Append all <link>/<joint> children from gripper into arm root so
        # build_link_transforms walks across both into a single tree.
        for child in list(gripper_root):
            if child.tag in ('link', 'joint'):
                arm_root.append(child)
    T_mount = np.eye(4)
    T_mount[:3, :3] = rpy_to_matrix(ARM_MOUNT_RPY)
    T_mount[:3, 3]  = ARM_MOUNT_XYZ
    placed, skipped = add_visuals(scene, arm_root, T_mount, prefix='arm')
    print(f'arm: placed {placed} meshes, skipped {len(skipped)}')
    for s in skipped[:8]:
        print('  skip:', s)

    # Recenter using CHASSIS-only XZ center (so SceneAnchor pivots about the
    # chassis center, not about the arm tip). Done in DAE Z-up frame, then
    # we Y-up flip the whole scene afterwards.
    if chassis_bounds is not None:
        cx = (chassis_bounds[0, 0] + chassis_bounds[1, 0]) / 2.0
        cy = (chassis_bounds[0, 1] + chassis_bounds[1, 1]) / 2.0  # Y in DAE = side
        T_center = np.eye(4)
        T_center[0, 3] = -cx
        T_center[1, 3] = -cy
        scene.apply_transform(T_center)

    # Z-up -> Y-up for three.js
    Y_UP = trimesh.transformations.rotation_matrix(-np.pi / 2, [1, 0, 0])
    scene.apply_transform(Y_UP)

    # Lift so bottom of tracks sits at y=0 (chassis DAE origin is at body
    # center vertically — without this the lower half ends up below floor).
    bounds = scene.bounds
    if bounds is not None and float(bounds[0, 1]) < 0:
        T_lift = np.eye(4)
        T_lift[1, 3] = -float(bounds[0, 1])
        scene.apply_transform(T_lift)

    print(f'writing {OUTPUT}')
    scene.export(str(OUTPUT))
    print('done')


if __name__ == '__main__':
    main()
