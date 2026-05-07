import { useGLTF } from '@react-three/drei';
import { useFrame } from '@react-three/fiber';
import { useMemo } from 'react';
import * as THREE from 'three';
import { useStore, type RobotType } from '@/lib/store';

const GLB_URLS: Record<RobotType, string> = {
  tb3:    '/turtlebot3.glb',
  tb4:    '/turtlebot4.glb',
  mir100: '/mir100.glb',
  taurob: '/taurob.glb',
  neura:  '/neura_mav.glb',
};

// Per-robot uniform scale applied at render time. Used to match real-world
// dimensions when the source GLB isn't authored to scale.
// Neura MAV (large): 1.53 × 0.91 × 0.293 m → source GLB ~1.3× too big.
const RENDER_SCALE: Partial<Record<RobotType, number>> = {
  neura: 0.77,
};

export function Robot(props: React.ComponentProps<'group'>) {
  const robotType = useStore((s) => s.robotType);
  const s = RENDER_SCALE[robotType] ?? 1;
  return (
    <group {...props} scale={s}>
      <GlbModel url={GLB_URLS[robotType]} />
    </group>
  );
}

// Approximate wheel radius and half-wheelbase per robot family for visual
// wheel spin. The half-wheelbase governs how fast wheels counter-spin during
// in-place yaw (smaller bots → smaller half-base → less wheel speed for same w).
// Visual radius — slightly inflated above the physical radius so the spin
// looks plausible to the eye instead of "blurred frisbee" fast.
const WHEEL_RADIUS: Record<RobotType, number> = {
  tb3: 0.10, tb4: 0.10, mir100: 0.12, taurob: 0.12, neura: 0.12,
};
const HALF_WHEELBASE: Record<RobotType, number> = {
  tb3: 0.080, tb4: 0.115, mir100: 0.222, taurob: 0.180, neura: 0.300,
};

// Axle axis in the wheel mesh's LOCAL space, by source kind.
// - Cylinder primitives (trimesh.creation.cylinder) align along Z.
// - URDF STL wheels are typically authored with the axle along Y.
const PRIM_AXIS = new THREE.Vector3(0, 0, 1);
const STL_AXIS  = new THREE.Vector3(0, 1, 0);

function GlbModel({ url }: { url: string }) {
  const { scene } = useGLTF(url, true);

  // Lift so the model's lowest point sits at y=0 (the floor). The URDF-baked
  // GLBs already do this, but third-party GLBs like Neura have their origin
  // at the geometric center → half the bot would end up below the floor.
  useMemo(() => {
    const box = new THREE.Box3().setFromObject(scene);
    const minY = box.min.y;
    if (Number.isFinite(minY) && minY < -0.001) {
      scene.position.y -= minY;
    }
  }, [scene]);
  const robotType = useStore((s) => s.robotType);

  // Collect wheels with the axle axis + side (left/right/center) so we can
  // also spin them on in-place rotations (differential drive).
  const wheels = useMemo(() => {
    type W = { obj: THREE.Object3D; axis: THREE.Vector3; side: 1 | -1 | 0 };
    const list: W[] = [];
    scene.traverse((o) => {
      if (!o.name.startsWith('WHEEL__')) return;
      const isPrim = o.name.endsWith('__primitive');
      const lower = o.name.toLowerCase();
      const side: 1 | -1 | 0 =
        /(^|_)l(eft)?_/.test(lower) || lower.includes('left') ? 1 :
        /(^|_)r(ight)?_/.test(lower) || lower.includes('right') ? -1 : 0;
      list.push({ obj: o, axis: isPrim ? PRIM_AXIS : STL_AXIS, side });
    });
    return list;
  }, [scene]);

  useFrame((_, delta) => {
    if (wheels.length === 0) return;
    const dt = Math.min(delta, 0.05);
    const v = useStore.getState().driveVelXZ;
    const R = WHEEL_RADIUS[robotType];
    const halfBase = HALF_WHEELBASE[robotType];
    const lin = v.vx;                          // m/s forward
    const yawComp = v.w * halfBase;            // m/s side contribution
    if (Math.abs(lin) < 1e-4 && Math.abs(v.w) < 1e-4) return;
    for (const w of wheels) {
      // side = +1 left, -1 right, 0 center/caster (use linear only)
      const wheelLinear = lin + (w.side === 1 ? -yawComp : w.side === -1 ? +yawComp : 0);
      const omega = wheelLinear / R;
      w.obj.rotateOnAxis(w.axis, omega * dt);
    }
  });

  scene.traverse((o) => {
    const mesh = o as THREE.Mesh;
    if (!mesh.isMesh) return;
    mesh.castShadow = true;
    mesh.receiveShadow = true;

    // LED stripes → glow blue. Match by GLB node name (led_*).
    // CRITICAL: trimesh's GLB exporter may share one material across many meshes,
    // so we MUST clone the material before mutating, otherwise every mesh
    // sharing that material starts glowing too.
    const parentName = (mesh.parent as THREE.Object3D | null)?.name ?? '';
    if (/^led/i.test(mesh.name) || /^led/i.test(parentName)) {
      if (Array.isArray(mesh.material)) {
        mesh.material = mesh.material.map((m) => m.clone());
      } else if (mesh.material) {
        mesh.material = (mesh.material as THREE.Material).clone();
      }
      const mats = Array.isArray(mesh.material) ? mesh.material : [mesh.material];
      for (const m of mats) {
        const mm = m as THREE.MeshStandardMaterial;
        if (mm && 'emissive' in mm) {
          mm.color = new THREE.Color(0x0a4cff);
          mm.emissive = new THREE.Color(0x2a82ff);
          mm.emissiveIntensity = 5.0;   // bright enough for bloom, not blinding
          mm.toneMapped = false;
          mm.needsUpdate = true;
        }
      }
    }
  });
  return <primitive object={scene} />;
}


// Footprint radii used by RobotMover for waypoint-arrived check + LaserScan height.
export const ROBOT_INFO: Record<RobotType, { radius: number; sensorHeight: number; label: string }> = {
  tb3:    { radius: 0.10, sensorHeight: 0.21, label: 'TurtleBot 3' },
  tb4:    { radius: 0.18, sensorHeight: 0.31, label: 'TurtleBot 4' },
  mir100: { radius: 0.45, sensorHeight: 0.10, label: 'MiR 100' },
  taurob: { radius: 0.30, sensorHeight: 0.27, label: 'Taurob Tracker' },
  neura:  { radius: 0.45, sensorHeight: 0.20, label: 'Neura MAV' },
};
