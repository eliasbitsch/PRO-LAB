/**
 * Cyan diamond floor tiles with Nav2-style costmap inflation gradient.
 *   cyan  = free   (>0.4m from any obstacle)
 *   yellow= caution(0.15..0.4m)
 *   red   = lethal (<0.15m)
 *
 * Sources for "obstacles": all detected horizontal planes that aren't part of
 * the floor (tables, couches, ...) AND all detected vertical planes (walls)
 * — projected to Y=0.
 */
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useFrame, useThree } from '@react-three/fiber';
import { useXR } from '@react-three/xr';
import {
  setFloorPolygons, setObstaclePolygons, distanceToObstacle, pointInPolygon,
  type FloorPolygon, type ObstaclePolygon,
} from '@/lib/floor';
import { useStore } from '@/lib/store';

const TILE_SIZE = 0.20;          // slightly larger -> fewer tiles
const MAX_TILES = 800;            // cap memory + draw call cost
const FLOOR_HEIGHT_TOL = 0.10;
const TILE_ROT_Y = Math.PI / 4;

const COLOR_FREE   = new THREE.Color(0x33ddff);
const COLOR_WARN   = new THREE.Color(0xffcc44);
const COLOR_LETHAL = new THREE.Color(0xff3344);

function inflationColor(d: number, out: THREE.Color) {
  if (d <= 0.15) return out.copy(COLOR_LETHAL);
  if (d >= 0.4)  return out.copy(COLOR_FREE);
  if (d <  0.275) {
    const t = (d - 0.15) / 0.125;
    return out.copy(COLOR_LETHAL).lerp(COLOR_WARN, t);
  }
  const t = (d - 0.275) / 0.125;
  return out.copy(COLOR_WARN).lerp(COLOR_FREE, t);
}

export function FloorTiles() {
  const session = useXR((s) => s.session);
  const enabled = useStore((s) => s.showFloorTiles);
  const { gl } = useThree();
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const lastBuild = useRef(0);

  const geom = useMemo(() => new THREE.BoxGeometry(TILE_SIZE * 0.78, 0.005, TILE_SIZE * 0.78), []);
  const mat = useMemo(() => new THREE.MeshBasicMaterial({
    color: 0xffffff,                  /* multiplied by per-instance color */
    transparent: true,
    opacity: 0.45,
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
    if (t - lastBuild.current < 500) return;
    lastBuild.current = t;

    type PD = { polygon: { x: number; z: number }[]; y: number; label: string; orient: 'horizontal' | 'vertical' };
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
      const polygon = ((plane as any).polygon as Array<{ x: number; y: number; z: number }>).map((v) => {
        tmpV.set(v.x, v.y, v.z).applyMatrix4(tmpM);
        return { x: tmpV.x, z: tmpV.z };
      });
      const y = pose.transform.position.y;
      if (orient === 'horizontal' && y < lowestY) lowestY = y;
      const label = String((plane as any).semanticLabel ?? '').toLowerCase();
      all.push({ polygon, y, label, orient });
    }

    const labeledFloors = all.filter((p) => p.orient === 'horizontal' && p.label === 'floor');
    const floors = labeledFloors.length > 0
      ? labeledFloors
      : all.filter((p) => p.orient === 'horizontal' && Math.abs(p.y - lowestY) < FLOOR_HEIGHT_TOL);
    const floorRefs = new Set(floors);

    // Obstacles: non-floor horizontal planes (tables) + ALL vertical planes (walls)
    const obstacles: ObstaclePolygon[] = all
      .filter((p) => !floorRefs.has(p))
      .map((p) => ({ polygon: p.polygon }));

    setFloorPolygons(floors.map((p) => ({ polygon: p.polygon, y: p.y } as FloorPolygon)));
    setObstaclePolygons(obstacles);

    // Build the tile grid
    const tmpObj = new THREE.Object3D();
    const tmpColor = new THREE.Color();
    let count = 0;

    for (const f of floors) {
      let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
      for (const v of f.polygon) {
        if (v.x < minX) minX = v.x; if (v.x > maxX) maxX = v.x;
        if (v.z < minZ) minZ = v.z; if (v.z > maxZ) maxZ = v.z;
      }
      const startX = Math.floor(minX / TILE_SIZE) * TILE_SIZE;
      const startZ = Math.floor(minZ / TILE_SIZE) * TILE_SIZE;
      for (let x = startX; x <= maxX; x += TILE_SIZE) {
        for (let z = startZ; z <= maxZ; z += TILE_SIZE) {
          const cx = x + TILE_SIZE / 2;
          const cz = z + TILE_SIZE / 2;
          if (!pointInPolygon(cx, cz, f.polygon)) continue;

          // Skip if directly under a non-floor obstacle (table, etc.)
          let blocked = false;
          for (const ex of obstacles) {
            if (pointInPolygon(cx, cz, ex.polygon)) { blocked = true; break; }
          }
          if (blocked) continue;

          const d = distanceToObstacle(cx, cz);
          inflationColor(d, tmpColor);

          tmpObj.position.set(cx, f.y + 0.003, cz);
          tmpObj.rotation.set(0, TILE_ROT_Y, 0);
          tmpObj.scale.set(1, 1, 1);
          tmpObj.updateMatrix();
          meshRef.current.setMatrixAt(count, tmpObj.matrix);
          meshRef.current.setColorAt(count, tmpColor);
          count++;
          if (count >= MAX_TILES) break;
        }
        if (count >= MAX_TILES) break;
      }
    }
    meshRef.current.count = count;
    meshRef.current.instanceMatrix.needsUpdate = true;
    if (meshRef.current.instanceColor) meshRef.current.instanceColor.needsUpdate = true;
  });

  return (
    <instancedMesh
      ref={meshRef}
      args={[geom, mat, MAX_TILES]}
      frustumCulled={false}
      renderOrder={-1}
    />
  );
}
