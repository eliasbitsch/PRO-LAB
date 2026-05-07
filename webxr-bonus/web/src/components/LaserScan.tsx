/**
 * 2D LiDAR-style scan visualization. Casts 360 rays at the robot's RPLIDAR
 * sensor height (~30cm above floor) against detected obstacle polygons (walls,
 * tables, ...) and renders red dots at each hit — same look as the /scan
 * topic in RViz.
 *
 * Two modes:
 *   1. Synthetic (default): rays are computed client-side from the room geometry
 *      detected via WebXR plane-detection. Works without any ROS data.
 *   2. ROS pass-through: would consume /scan from rosbridge directly. Not
 *      implemented here — the synthetic version is cleaner for the AR demo
 *      because it stays consistent with what the user is actually seeing.
 */
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';
import { useXR } from '@react-three/xr';
import { raycastObstacles, pointOnAnyFloor } from '@/lib/floor';
import { useStore } from '@/lib/store';

const NUM_RAYS = 180;            // 2° resolution — matches RPLIDAR low-rate mode
const SCAN_INTERVAL_MS = 120;    // ~8 Hz (RPLIDAR runs 5-15 Hz)
const SENSOR_HEIGHT = 0.30;
const MAX_DIST = 6.0;
const DOT_RADIUS = 0.012;

type SceneAnchorRefs = {
  anchor: THREE.Group | null;
  robot: THREE.Group | null;
};

export function LaserScan({ anchorRef }: { anchorRef: React.RefObject<THREE.Group | null> }) {
  const session = useXR((s) => s.session);
  const enabled = useStore((s) => s.showLaserScan);
  const meshRef = useRef<THREE.InstancedMesh>(null);
  const lastScan = useRef(0);

  const dotGeom = useMemo(() => new THREE.SphereGeometry(DOT_RADIUS, 6, 4), []);
  const dotMat = useMemo(() => new THREE.MeshBasicMaterial({
    color: 0xff2233,
    transparent: true,
    opacity: 0.9,
    depthWrite: false,
  }), []);

  const tmpObj = new THREE.Object3D();
  const sensorWorld = useMemo(() => new THREE.Vector3(), []);

  useFrame(() => {
    if (!enabled) {
      if (meshRef.current) meshRef.current.count = 0;
      return;
    }
    if (!session || !meshRef.current) return;
    const now = performance.now();
    if (now - lastScan.current < SCAN_INTERVAL_MS) return;
    lastScan.current = now;

    const anchor = anchorRef.current;
    if (!anchor) return;
    anchor.getWorldPosition(sensorWorld);
    const ox = sensorWorld.x;
    const oz = sensorWorld.z;

    if (!pointOnAnyFloor(ox, oz)) {
      meshRef.current.count = 0;
      return;
    }

    let dotCount = 0;
    for (let i = 0; i < NUM_RAYS; ++i) {
      const angle = (i / NUM_RAYS) * Math.PI * 2;
      const hit = raycastObstacles(ox, oz, Math.cos(angle), Math.sin(angle), MAX_DIST);
      if (!hit) continue;
      tmpObj.position.set(hit.x, sensorWorld.y + SENSOR_HEIGHT, hit.z);
      tmpObj.rotation.set(0, 0, 0);
      tmpObj.scale.set(1, 1, 1);
      tmpObj.updateMatrix();
      meshRef.current.setMatrixAt(dotCount++, tmpObj.matrix);
    }
    meshRef.current.count = dotCount;
    meshRef.current.instanceMatrix.needsUpdate = true;
  });

  return (
    <instancedMesh ref={meshRef} args={[dotGeom, dotMat, NUM_RAYS]} frustumCulled={false} />
  );
}
