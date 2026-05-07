import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useFrame } from '@react-three/fiber';
import { mergeGeometries } from 'three/examples/jsm/utils/BufferGeometryUtils.js';
import { useStore, type Particle } from '@/lib/store';
import { pointIsDrivable } from '@/lib/floor';

const MAX = 128;     // cap for Quest perf; covers the 80 sampler emits comfortably
const LERP_DURATION_MS = 1000;       // matches AMCL tick interval

// Demo set if /webxr/particles is empty — scattered in 3D for the volumetric
// look (XR demo polish). The PF itself is 2D, so for real /pf/pose particles
// the z is added cosmetically per-particle so the cloud reads as a 3D blob.
type DemoParticle = Particle & { z: number };
const DEMO_PARTICLES: DemoParticle[] = Array.from({ length: 10 }, (_, i) => {
  const angle = (i / 10) * Math.PI * 2;
  const radius = 0.25 + Math.random() * 0.15;
  return {
    x: Math.cos(angle) * radius,
    y: Math.sin(angle) * radius,
    z: Math.random() * 0.45,
    yaw: angle + Math.PI / 2 + (Math.random() - 0.5) * 0.4,
  };
});

// RViz-style arrow geometry: shaft cylinder + cone head, oriented along +X
function makeArrowGeometry() {
  const length = 0.18;
  const shaftR = 0.012;
  const headR = 0.028;
  const headH = 0.06;

  const shaft = new THREE.CylinderGeometry(shaftR, shaftR, length, 10);
  // CylinderGeometry is along Y. Rotate so shaft points along +X, base at origin.
  shaft.rotateZ(-Math.PI / 2);
  shaft.translate(length / 2, 0, 0);

  const head = new THREE.ConeGeometry(headR, headH, 14);
  head.rotateZ(-Math.PI / 2);
  head.translate(length + headH / 2, 0, 0);

  return mergeGeometries([shaft, head]);
}

const tmpObj = new THREE.Object3D();

export function ParticleCloud() {
  const ref = useRef<THREE.InstancedMesh>(null);
  const particles = useStore((s) => s.particles);
  const visible = useStore((s) => s.showParticles);
  const scale = useStore((s) => s.particleScale);

  const geom = useMemo(makeArrowGeometry, []);
  const mat = useMemo(() => new THREE.MeshStandardMaterial({
    color: 0xff3344,
    emissive: 0x661122,
    emissiveIntensity: 0.65,
    transparent: true,
    opacity: 0.92,
    metalness: 0.1,
    roughness: 0.45,
  }), []);

  // Per-particle phase offsets for the lerp jitter
  const phases = useMemo(() => Float32Array.from({ length: MAX }, () => Math.random() * Math.PI * 2), []);
  // Per-particle Z (height) offsets — applied to incoming 2D /pf/pose particles
  // so the cloud looks volumetric. Demo particles already carry their own z.
  const zOffsets = useMemo(() => Float32Array.from({ length: MAX }, () => Math.random() * 0.4), []);
  // Per-particle pitch (vertical tilt) for the arrows
  const pitches = useMemo(() => Float32Array.from({ length: MAX }, () => (Math.random() - 0.5) * 0.5), []);

  // Use demo set when no live data; AMCL pushes its 80 particles into store.
  const display = particles.length === 0 ? DEMO_PARTICLES : particles;

  // ---- Lerp state — particles morph from old positions to new each AMCL tick
  type Pose = { x: number; y: number; yaw: number };
  const shown = useMemo(() => Array.from({ length: MAX }, () => ({ x: 0, y: 0, yaw: 0 } as Pose)), []);
  const startBuf = useMemo(() => Array.from({ length: MAX }, () => ({ x: 0, y: 0, yaw: 0 } as Pose)), []);
  const targetBuf = useMemo(() => Array.from({ length: MAX }, () => ({ x: 0, y: 0, yaw: 0 } as Pose)), []);
  const lerpStart = useRef(0);
  const lastRef = useRef<typeof display | null>(null);

  if (display !== lastRef.current) {
    const n = Math.min(display.length, MAX);
    for (let i = 0; i < n; i++) {
      // Bootstrap shown[] on first run so initial particles ease in from same pos
      if (lastRef.current === null) {
        shown[i].x = display[i].x; shown[i].y = display[i].y; shown[i].yaw = display[i].yaw;
      }
      startBuf[i].x = shown[i].x; startBuf[i].y = shown[i].y; startBuf[i].yaw = shown[i].yaw;
      targetBuf[i].x = display[i].x; targetBuf[i].y = display[i].y; targetBuf[i].yaw = display[i].yaw;
    }
    lerpStart.current = performance.now();
    lastRef.current = display;
  }

  const localPos = useMemo(() => new THREE.Vector3(), []);
  const worldPos = useMemo(() => new THREE.Vector3(), []);

  useFrame((state) => {
    if (!ref.current) return;
    const tNow = state.clock.getElapsedTime();
    ref.current.updateWorldMatrix(true, false);
    const worldMat = ref.current.matrixWorld;

    // Lerp progress 0..1
    const raw = (performance.now() - lerpStart.current) / LERP_DURATION_MS;
    const tLerp = raw < 0 ? 0 : raw > 1 ? 1 : raw;
    const ease = tLerp * tLerp * (3 - 2 * tLerp);

    const max = Math.min(display.length, MAX);
    let count = 0;
    for (let i = 0; i < max; i++) {
      const s = startBuf[i];
      const e = targetBuf[i];
      // Lerp position, slerp yaw via shortest angular path
      const x = s.x + (e.x - s.x) * ease;
      const y = s.y + (e.y - s.y) * ease;
      const dy = Math.atan2(Math.sin(e.yaw - s.yaw), Math.cos(e.yaw - s.yaw));
      const yaw = s.yaw + dy * ease;
      shown[i].x = x; shown[i].y = y; shown[i].yaw = yaw;

      const phase = phases[i];
      const baseZ = (display[i] && 'z' in display[i] ? (display[i] as DemoParticle).z : zOffsets[i]) + 0.05;
      const jitterX = Math.sin(tNow * 1.6 + phase) * 0.012;
      const jitterY = Math.cos(tNow * 1.4 + phase * 1.3) * 0.012;
      const jitterZ = Math.sin(tNow * 0.9 + phase * 0.7) * 0.018;

      localPos.set(x + jitterX, baseZ + jitterZ, -(y + jitterY));
      worldPos.copy(localPos).applyMatrix4(worldMat);
      if (!pointIsDrivable(worldPos.x, worldPos.z)) continue;

      const pitch = pitches[i] + Math.sin(tNow * 1.2 + phase) * 0.03;
      tmpObj.position.copy(localPos);
      tmpObj.rotation.set(0, -yaw, pitch);
      tmpObj.scale.setScalar(scale);
      tmpObj.updateMatrix();
      ref.current.setMatrixAt(count++, tmpObj.matrix);
    }
    ref.current.count = count;
    ref.current.instanceMatrix.needsUpdate = true;
  });

  if (!visible) return null;
  return (
    <instancedMesh ref={ref} args={[geom, mat, MAX]} frustumCulled={false} />
  );
}
