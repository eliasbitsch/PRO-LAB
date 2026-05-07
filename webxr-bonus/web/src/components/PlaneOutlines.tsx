/**
 * Thick blue outline of every detected plane (floor + tables + walls).
 * Uses InstancedMesh of cylinders per segment for guaranteed thick lines on
 * Quest (which ignores GL_LINE_WIDTH > 1).
 *
 * Side-effect: also populates the floor/obstacle registries used by the
 * ParticleCloud (cull) and LaserScan (raycast).
 */
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useFrame, useThree } from '@react-three/fiber';
import { useXR } from '@react-three/xr';
import {
  setFloorPolygons, setObstaclePolygons,
  type FloorPolygon, type ObstaclePolygon,
} from '@/lib/floor';
import { useStore } from '@/lib/store';

const FLOOR_HEIGHT_TOL = 0.10;
const MAX_SEGMENTS = 600;
const LINE_THICKNESS = 0.022;    // 22mm — chunky map-line
const LINE_COLOR = 0x1a44ff;     // dark blue
const FLOOR_LIFT = 0.008;        // lift slightly off the real floor so lines aren't z-fighting

const tmpObj = new THREE.Object3D();
const tmpA = new THREE.Vector3();
const tmpB = new THREE.Vector3();
const tmpDir = new THREE.Vector3();
const tmpQ = new THREE.Quaternion();
const Y_AXIS = new THREE.Vector3(0, 1, 0);

export function PlaneOutlines() {
  const session = useXR((s) => s.session);
  const enabled = useStore((s) => s.showFloorTiles);    // reuse existing toggle
  const { gl } = useThree();
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const lastBuild = useRef(0);

  // Cylinder along Y, length 1, radius LINE_THICKNESS. We scale Y per-segment.
  const geom = useMemo(() => {
    const g = new THREE.CylinderGeometry(LINE_THICKNESS, LINE_THICKNESS, 1, 6, 1);
    g.translate(0, 0.5, 0);
    return g;
  }, []);
  const mat = useMemo(() => new THREE.MeshBasicMaterial({
    color: LINE_COLOR,
    transparent: true,
    opacity: 0.92,
    depthWrite: false,
  }), []);

  useFrame((_state, _dt, xrFrame?: XRFrame) => {
    if (!enabled) {
      if (meshRef.current) meshRef.current.count = 0;
      return;
    }
    if (!session || !xrFrame || !meshRef.current) return;
    const refSpace = gl.xr.getReferenceSpace();
    if (!refSpace) return;

    const planes = (xrFrame as any).detectedPlanes as Set<XRPlane> | undefined;
    if (!planes) return;

    const t = performance.now();
    if (t - lastBuild.current < 700) return;
    lastBuild.current = t;

    type PD = {
      worldPolygon: { x: number; y: number; z: number }[];
      flat: { x: number; z: number }[];
      orient: 'horizontal' | 'vertical';
      y: number;
      label: string;
    };
    const all: PD[] = [];
    let lowestY = Infinity;
    const tmpV = new THREE.Vector3();
    const tmpM = new THREE.Matrix4();

    for (const plane of planes) {
      const orient = (plane as any).orientation as 'horizontal' | 'vertical';
      if (!orient) continue;
      const pose = xrFrame.getPose((plane as any).planeSpace, refSpace);
      if (!pose) continue;
      tmpM.fromArray(pose.transform.matrix);

      const worldPolygon: { x: number; y: number; z: number }[] = [];
      const flat: { x: number; z: number }[] = [];
      const polygon = (plane as any).polygon as Array<{ x: number; y: number; z: number }>;
      for (const v of polygon) {
        tmpV.set(v.x, v.y, v.z).applyMatrix4(tmpM);
        worldPolygon.push({ x: tmpV.x, y: tmpV.y, z: tmpV.z });
        flat.push({ x: tmpV.x, z: tmpV.z });
      }
      const y = pose.transform.position.y;
      if (orient === 'horizontal' && y < lowestY) lowestY = y;
      all.push({
        worldPolygon, flat, orient, y,
        label: String((plane as any).semanticLabel ?? '').toLowerCase(),
      });
    }

    // Identify floor planes (semantic-labeled or lowest horizontal)
    const labeled = all.filter((p) => p.orient === 'horizontal' && p.label === 'floor');
    const floorSet = labeled.length > 0
      ? new Set<PD>(labeled)
      : new Set<PD>(all.filter((p) => p.orient === 'horizontal' && Math.abs(p.y - lowestY) < FLOOR_HEIGHT_TOL));

    // Update shared registries for particle cull + laser scan
    const floors: FloorPolygon[] = [];
    const obstacles: ObstaclePolygon[] = [];
    for (const p of all) {
      if (floorSet.has(p)) floors.push({ polygon: p.flat, y: p.y });
      else obstacles.push({ polygon: p.flat });
    }
    setFloorPolygons(floors);
    setObstaclePolygons(obstacles);

    const floorY = (lowestY === Infinity ? 0 : lowestY) + FLOOR_LIFT;

    // ---- Collect all candidate segments tagged by source kind + plane id ----
    type Kind = 'floor' | 'wall' | 'table';
    type Seg = { ax: number; az: number; bx: number; bz: number; len: number; kind: Kind; planeIdx: number; hide: boolean };
    const candidates: Seg[] = [];
    all.forEach((p, planeIdx) => {
      const isFloor = floorSet.has(p);
      const kind: Kind = isFloor ? 'floor' : p.orient === 'vertical' ? 'wall' : 'table';
      const verts = p.flat;
      const n = verts.length;
      for (let i = 0; i < n; ++i) {
        const a = verts[i];
        const b = verts[(i + 1) % n];
        const dx = b.x - a.x, dz = b.z - a.z;
        const len = Math.hypot(dx, dz);
        if (len < 1e-5) continue;
        candidates.push({ ax: a.x, az: a.z, bx: b.x, bz: b.z, len, kind, planeIdx, hide: false });
      }
    });

    // ---- Resolve overlaps with TIGHT tolerances ----
    // RULE 1: same plane, colinear-overlap -> drop the shorter (collapses the
    //         degenerate front/back edges that vertical walls produce when
    //         projected to Y=0).
    // RULE 2: table edge overlaps wall edge -> hide BOTH (visual gap at contact).
    // RULE 3: table edge overlaps floor edge -> hide table (keep floor line).
    // Anything else (different planes of the same kind) is left alone.
    const ANGLE_DOT = 0.995;         // ~5.7° max angle deviation
    const PERP_TOL = 0.04;            // 4cm
    function overlaps(a: Seg, b: Seg): boolean {
      const adx = (a.bx - a.ax) / a.len, adz = (a.bz - a.az) / a.len;
      const bdx = (b.bx - b.ax) / b.len, bdz = (b.bz - b.az) / b.len;
      if (Math.abs(adx * bdx + adz * bdz) < ANGLE_DOT) return false;
      const mx = (a.ax + a.bx) / 2, mz = (a.az + a.bz) / 2;
      const dx = mx - b.ax, dz = mz - b.az;
      const perp = Math.abs(dx * (-bdz) + dz * bdx);
      if (perp > PERP_TOL) return false;
      const t1 = ((a.ax - b.ax) * bdx + (a.az - b.az) * bdz) / b.len;
      const t2 = ((a.bx - b.ax) * bdx + (a.bz - b.az) * bdz) / b.len;
      const tmin = Math.min(t1, t2);
      const tmax = Math.max(t1, t2);
      return tmax > 0.05 && tmin < 0.95;            // require >5% overlap
    }
    for (let i = 0; i < candidates.length; ++i) {
      const a = candidates[i];
      if (a.hide) continue;
      for (let j = 0; j < candidates.length; ++j) {
        if (j === i) continue;
        const b = candidates[j];
        if (b.hide) continue;
        if (!overlaps(a, b)) continue;
        const sameKind = a.kind === b.kind;
        const samePlane = a.planeIdx === b.planeIdx;
        if (sameKind && samePlane) {
          if (a.len < b.len) a.hide = true; else b.hide = true;
        } else if (sameKind) {
          // Different planes, same kind (two walls / two tables) — leave both.
          continue;
        } else if ((a.kind === 'table' && b.kind === 'wall') || (a.kind === 'wall' && b.kind === 'table')) {
          a.hide = true; b.hide = true;
        } else if (a.kind === 'table' && b.kind === 'floor') {
          a.hide = true;
        } else if (a.kind === 'floor' && b.kind === 'table') {
          b.hide = true;
        }
      }
    }
    const kept = candidates.filter((s) => !s.hide);

    // ---- Build InstancedMesh from the surviving segments ----
    let count = 0;
    for (const s of kept) {
      if (count >= MAX_SEGMENTS) break;
      tmpA.set(s.ax, floorY, s.az);
      tmpB.set(s.bx, floorY, s.bz);
      tmpDir.copy(tmpB).sub(tmpA);
      const len = tmpDir.length();
      tmpDir.divideScalar(len);
      tmpQ.setFromUnitVectors(Y_AXIS, tmpDir);
      tmpObj.position.copy(tmpA);
      tmpObj.quaternion.copy(tmpQ);
      tmpObj.scale.set(1, len, 1);
      tmpObj.updateMatrix();
      meshRef.current.setMatrixAt(count++, tmpObj.matrix);
    }
    meshRef.current.count = count;
    meshRef.current.instanceMatrix.needsUpdate = true;
  });

  return (
    <instancedMesh
      ref={meshRef}
      args={[geom, mat, MAX_SEGMENTS]}
      frustumCulled={false}
      renderOrder={-1}
    />
  );
}
