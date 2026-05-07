import { Suspense, useEffect, useMemo, useRef } from 'react';
import { Canvas, useFrame, useThree } from '@react-three/fiber';
import {
  ContactShadows, Environment, MeshReflectorMaterial, OrbitControls,
} from '@react-three/drei';
import { XR, createXRStore, useXR, useXRInputSourceEvent } from '@react-three/xr';
import { EffectComposer, Bloom } from '@react-three/postprocessing';
import * as THREE from 'three';

import { Robot, ROBOT_INFO } from './Robot';
import { PoseArrow } from './PoseArrow';
import { ParticleCloud } from './ParticleCloud';
import { PlaneOutlines } from './PlaneOutlines';
import { LaserScan } from './LaserScan';
import { AmclRunner } from './AmclRunner';
import { useStore } from '@/lib/store';

export const xrStore = createXRStore({
  hand: {
    rayPointer: { rayModel: { color: 'cyan', maxLength: 5 } },
    teleportPointer: false,
    touchPointer: false,
  },
  controller: {
    rayPointer: { rayModel: { color: 'cyan', maxLength: 5 } },
    teleportPointer: false,
  },
  foveation: 1.0,
});

function FilterPose({
  which, color, sample,
}: {
  which: 'KF' | 'EKF' | 'PF';
  color: string;
  sample: { x: number; y: number; yaw: number } | null;
}) {
  const visible = useStore((s) => (
    which === 'KF' ? s.showKF : which === 'EKF' ? s.showEKF : s.showPF
  ));
  if (!sample || !visible) return null;
  return (
    <group position={[sample.x, 0.06, -sample.y]} rotation={[0, -sample.yaw, 0]}>
      <PoseArrow color={color} />
    </group>
  );
}

// ─────────────────────────────────────────────────────────────────────────────
// Keyboard input (desktop fallback): WASD + arrow keys
// ─────────────────────────────────────────────────────────────────────────────
function useKeyboard() {
  const keys = useRef<Record<string, boolean>>({});
  useEffect(() => {
    const dn = (e: KeyboardEvent) => { keys.current[e.code] = true; };
    const up = (e: KeyboardEvent) => { keys.current[e.code] = false; };
    // CRITICAL: keyup never fires if the window/tab loses focus while a key
    // is held → robot would drive forever. Clear all keys on blur/visibility.
    const clearAll = () => { keys.current = {}; };
    window.addEventListener('keydown', dn);
    window.addEventListener('keyup', up);
    window.addEventListener('blur', clearAll);
    document.addEventListener('visibilitychange', () => {
      if (document.hidden) clearAll();
    });
    return () => {
      window.removeEventListener('keydown', dn);
      window.removeEventListener('keyup', up);
      window.removeEventListener('blur', clearAll);
    };
  }, []);
  return keys;
}

function SceneAnchor({ children, anchorRef }: {
  children: React.ReactNode;
  anchorRef: React.RefObject<THREE.Group | null>;
}) {
  const ref = anchorRef;
  const robotRef = useRef<THREE.Group>(null);
  const hoverHaloRef = useRef<THREE.Mesh>(null);
  const targetMarkerRef = useRef<THREE.Group>(null);

  const robotType = useStore((s) => s.robotType);
  const controlMode = useStore((s) => s.controlMode);
  const waypoint = useStore((s) => s.waypoint);
  const setWaypoint = useStore((s) => s.setWaypoint);
  const setDriveVel = useStore((s) => s.setDriveVel);

  const grabSrc = useRef<XRInputSource | null>(null);
  const grabDistance = useRef(1.0);
  const grabOffset = useRef(new THREE.Vector3());

  const velY = useRef(0);
  const falling = useRef(false);
  const yaw = useRef(0); // anchor yaw (radians, around world Y)

  const keys = useKeyboard();

  // Helper: ray from XRSpace
  const m4 = useMemo(() => new THREE.Matrix4(), []);
  const FORWARD = useMemo(() => new THREE.Vector3(0, 0, -1), []);
  const tmpA = useMemo(() => new THREE.Vector3(), []);
  const tmpB = useMemo(() => new THREE.Vector3(), []);

  function rayFromSpace(
    xrFrame: XRFrame, refSpace: XRReferenceSpace, space: XRSpace,
    outOrigin: THREE.Vector3, outDir: THREE.Vector3,
  ): boolean {
    const pose = xrFrame.getPose(space, refSpace);
    if (!pose) return false;
    m4.fromArray(pose.transform.matrix);
    outOrigin.setFromMatrixPosition(m4);
    outDir.copy(FORWARD).transformDirection(m4);
    return true;
  }

  // Intersect ray with horizontal plane at y=yPlane. Returns true and writes out.
  function rayHitFloor(
    origin: THREE.Vector3, dir: THREE.Vector3, yPlane: number, out: THREE.Vector3,
  ): boolean {
    if (Math.abs(dir.y) < 1e-4) return false;
    const t = (yPlane - origin.y) / dir.y;
    if (t <= 0) return false;
    out.copy(origin).addScaledVector(dir, t);
    return true;
  }

  useXRInputSourceEvent('all', 'selectstart', (e) => {
    // Decide: grab robot OR set waypoint (if in waypoint mode and pointing at floor)
    falling.current = false;
    velY.current = 0;
    grabSrc.current = e.inputSource;
    grabDistance.current = -1;
  }, []);

  useXRInputSourceEvent('all', 'selectend', () => {
    if (!grabSrc.current) return;
    grabSrc.current = null;
    if (ref.current && ref.current.position.y > 0.001) {
      falling.current = true;
      velY.current = 0;
    }
  }, []);

  useFrame((state, delta, xrFrame?: XRFrame) => {
    try {
      const dt = Math.min(delta, 0.05);
      const gl = state.gl;
      const session = gl.xr.getSession();
      const refSpace = gl.xr.getReferenceSpace();
      const inXR = !!(xrFrame && session && refSpace);

      // Gravity drop
      if (falling.current && ref.current) {
        velY.current -= 9.81 * dt;
        ref.current.position.y += velY.current * dt;
        if (ref.current.position.y <= 0) {
          ref.current.position.y = 0;
          if (Math.abs(velY.current) > 0.4) velY.current = -velY.current * 0.3;
          else { velY.current = 0; falling.current = false; }
        }
      }

      // Grabbing — if ray hits robot, follow ray; otherwise treat as waypoint pinch
      let isGrabbing = false;
      if (inXR && grabSrc.current && ref.current && robotRef.current) {
        const src = grabSrc.current;
        if (rayFromSpace(xrFrame!, refSpace!, src.targetRaySpace, tmpA, tmpB)) {
          if (grabDistance.current < 0) {
            // First frame after selectstart: decide
            const ray = new THREE.Raycaster(tmpA, tmpB);
            const hits = ray.intersectObject(robotRef.current, true);
            if (hits.length > 0) {
              grabDistance.current = hits[0].distance;
              grabOffset.current.copy(ref.current.position).sub(hits[0].point);
            } else if (controlMode === 'waypoint') {
              // floor hit → set waypoint
              const floorHit = new THREE.Vector3();
              if (rayHitFloor(tmpA, tmpB, 0, floorHit)) {
                setWaypoint({ x: floorHit.x, z: floorHit.z });
              }
              grabSrc.current = null;
            } else {
              grabSrc.current = null;
            }
          }
          if (grabSrc.current) {
            isGrabbing = true;
            const target = tmpA.clone().addScaledVector(tmpB, grabDistance.current);
            ref.current.position.copy(target).add(grabOffset.current);
          }
        }
      }

      // ── Drive: waypoint or joystick ──
      if (!isGrabbing && !falling.current && ref.current) {
        const MAX_LIN = 0.6;     // m/s
        const MAX_ANG = 1.6;     // rad/s
        let vLin = 0;            // forward (+) / back (−)
        let vAng = 0;            // yaw rate
        // All bots are differential drive — no strafe.

        // Quest gamepad sticks: left = forward/back, right = turn (diff-drive convention)
        if (inXR && session && controlMode === 'joystick') {
          for (const src of session.inputSources) {
            const gp = (src as any).gamepad as Gamepad | undefined;
            if (!gp || !gp.axes) continue;
            const ax = gp.axes.length >= 4 ? gp.axes[2] : gp.axes[0];
            const ay = gp.axes.length >= 4 ? gp.axes[3] : gp.axes[1];
            const dz = (v: number) => Math.abs(v) < 0.15 ? 0 : v;
            if (src.handedness === 'left') {
              vLin += -dz(ay) * MAX_LIN;
              vAng += -dz(ax) * MAX_ANG;   // left-stick X also turns (some users prefer)
            } else if (src.handedness === 'right') {
              vAng += -dz(ax) * MAX_ANG;
              if (gp.axes.length < 4) vLin += -dz(ay) * MAX_LIN;
            } else {
              vLin += -dz(ay) * MAX_LIN;
              vAng += -dz(ax) * MAX_ANG;
            }
          }
        }

        // Keyboard: WASD where W/S = forward/back, A/D = turn (diff-drive).
        if (controlMode !== 'off') {
          const k = keys.current;
          if (k['KeyW'] || k['ArrowUp'])                   vLin += MAX_LIN;
          if (k['KeyS'] || k['ArrowDown'])                 vLin -= MAX_LIN;
          if (k['KeyA'] || k['ArrowLeft']  || k['KeyQ'])   vAng += MAX_ANG;
          if (k['KeyD'] || k['ArrowRight'] || k['KeyE'])   vAng -= MAX_ANG;
        }

        // Waypoint seek (overrides joystick if active)
        if (controlMode === 'waypoint' && waypoint) {
          const dx = waypoint.x - ref.current.position.x;
          const dz = waypoint.z - ref.current.position.z;
          const dist = Math.hypot(dx, dz);
          const arriveR = ROBOT_INFO[robotType].radius * 0.6 + 0.05;
          if (dist < arriveR) {
            setWaypoint(null);
            vLin = 0; vAng = 0;
          } else {
            const desiredYaw = Math.atan2(-dz, dx);
            let dy = desiredYaw - yaw.current;
            while (dy >  Math.PI) dy -= 2 * Math.PI;
            while (dy < -Math.PI) dy += 2 * Math.PI;
            vAng = THREE.MathUtils.clamp(dy * 2.5, -MAX_ANG, MAX_ANG);
            const align = Math.max(0, Math.cos(dy));
            vLin = Math.min(MAX_LIN, dist * 1.2) * align;
          }
        }

        // Apply (diff-drive — only forward + yaw)
        if (vLin || vAng) {
          yaw.current += vAng * dt;
          const fwdX = Math.cos(yaw.current);
          const fwdZ = -Math.sin(yaw.current);
          ref.current.position.x += fwdX * vLin * dt;
          ref.current.position.z += fwdZ * vLin * dt;
          ref.current.rotation.y = yaw.current;
        }

        setDriveVel({ vx: vLin, vz: 0, w: vAng });
      }

      // Hover halo
      if (hoverHaloRef.current && robotRef.current) {
        let hovering = false;
        if (inXR && session) {
          for (const src of session.inputSources) {
            if (rayFromSpace(xrFrame!, refSpace!, src.targetRaySpace, tmpA, tmpB)) {
              const ray = new THREE.Raycaster(tmpA, tmpB);
              if (ray.intersectObject(robotRef.current, true).length > 0) {
                hovering = true; break;
              }
            }
          }
        }
        const targetOpacity = grabSrc.current ? 1.0 : hovering ? 0.55 : 0.0;
        const mat = hoverHaloRef.current.material as THREE.MeshBasicMaterial;
        mat.opacity = THREE.MathUtils.lerp(mat.opacity, targetOpacity, 0.18);
        hoverHaloRef.current.visible = mat.opacity > 0.01;
      }

      // Waypoint marker pulse
      if (targetMarkerRef.current) {
        targetMarkerRef.current.visible = !!waypoint;
        if (waypoint) {
          targetMarkerRef.current.position.set(waypoint.x, 0.005, waypoint.z);
          const t = state.clock.elapsedTime;
          const s = 1 + Math.sin(t * 4) * 0.15;
          targetMarkerRef.current.scale.set(s, 1, s);
        }
      }
    } catch (err) {
      console.warn('[xr useFrame]', err);
    }
  });

  return (
    <>
      <group ref={ref as React.Ref<THREE.Group>} position={[0, 0, -1.5]}>
        <group ref={robotRef}>{children}</group>
        <mesh ref={hoverHaloRef} rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.005, 0]}>
          <ringGeometry args={[0.21, 0.27, 64]} />
          <meshBasicMaterial color="#5cf" transparent opacity={0} side={THREE.DoubleSide} />
        </mesh>
      </group>
      {/* Waypoint marker — sits at world coords, NOT inside anchor */}
      <group ref={targetMarkerRef} visible={false}>
        <mesh rotation={[-Math.PI / 2, 0, 0]}>
          <ringGeometry args={[0.18, 0.24, 32]} />
          <meshBasicMaterial color="#22ff88" transparent opacity={0.85} side={THREE.DoubleSide} />
        </mesh>
        <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0.001, 0]}>
          <circleGeometry args={[0.05, 16]} />
          <meshBasicMaterial color="#22ff88" />
        </mesh>
      </group>
    </>
  );
}

function SceneBackground() {
  const session = useXR((s) => s.session);
  const isAR = session && (session.environmentBlendMode === 'additive' ||
                           session.environmentBlendMode === 'alpha-blend');
  if (isAR) return null;
  return (
    <>
      <color attach="background" args={['#1a2030']} />
      <fog attach="fog" args={['#1a2030', 12, 28]} />
    </>
  );
}

function NonXR({ children }: { children: React.ReactNode }) {
  const session = useXR((s) => s.session);
  if (session) return null;
  return <>{children}</>;
}

function HandOpacityPatcher() {
  const session = useXR((s) => s.session);
  const { scene } = useThree();
  const patched = useMemo(() => new WeakSet<THREE.Material>(), []);
  useFrame(() => {
    if (!session) return;
    scene.traverse((obj) => {
      const sm = obj as THREE.SkinnedMesh;
      if (!(sm as any).isSkinnedMesh || !sm.material) return;
      const apply = (m: THREE.Material) => {
        if (patched.has(m)) return;
        const mm = m as THREE.MeshStandardMaterial;
        mm.transparent = true;
        mm.opacity = 0.10;
        mm.depthWrite = false;
        mm.side = THREE.DoubleSide;
        mm.needsUpdate = true;
        patched.add(m);
      };
      if (Array.isArray(sm.material)) sm.material.forEach(apply);
      else apply(sm.material);
    });
  });
  return null;
}

function ARFloorReflector() {
  const inXR = useXR((s) => s.session !== undefined);
  if (inXR) return null;
  return (
    <mesh rotation={[-Math.PI / 2, 0, 0]} position={[0, 0, 0]} receiveShadow>
      <planeGeometry args={[20, 20]} />
      <MeshReflectorMaterial
        mirror={0.5} blur={[300, 100]} resolution={1024} mixBlur={1} mixStrength={50}
        roughness={1} depthScale={1.2} minDepthThreshold={0.4} maxDepthThreshold={1.4}
        color="#0d0f12" metalness={0.6}
      />
    </mesh>
  );
}

function SceneLights() {
  return (
    <>
      <ambientLight intensity={0.25} />
      <hemisphereLight intensity={0.45} color="#ffffff" groundColor="#445566" />
      <directionalLight position={[4, 8, 3]} intensity={1.4} />
      <directionalLight position={[-3, 4, -2]} intensity={0.55} color="#88aaff" />
      <pointLight position={[1.5, 1.0, 1.5]} intensity={0.35} color="#ffd6a8" />
    </>
  );
}

export default function Scene() {
  const kf = useStore((s) => s.kf);
  const ekf = useStore((s) => s.ekf);
  const pf = useStore((s) => s.pf);
  const anchorRef = useRef<THREE.Group>(null);

  return (
    <Canvas
      shadows={false}
      camera={{ position: [0, 1.6, 2.4], fov: 65, near: 0.05, far: 200 }}
      dpr={[1, 1.5]}
      gl={{
        antialias: true, alpha: true,
        toneMapping: THREE.NoToneMapping,
        powerPreference: 'high-performance',
      }}
      onCreated={({ gl }) => {
        gl.xr.setReferenceSpaceType('local-floor');
        gl.xr.setFramebufferScaleFactor(0.7);
      }}
    >
      <XR store={xrStore}>
        <SceneBackground />
        <SceneLights />
        <PlaneOutlines />
        <HandOpacityPatcher />

        <NonXR>
          <Suspense fallback={null}>
            <Environment preset="warehouse" environmentIntensity={0.45} background={false} />
          </Suspense>
          <ARFloorReflector />
          {/* Bloom for glowing emissive materials (LED stripes etc). NonXR only —
              postprocessing is heavy on Quest. */}
          <EffectComposer>
            <Bloom
              intensity={0.7}
              luminanceThreshold={0.9}
              luminanceSmoothing={0.3}
              mipmapBlur
            />
          </EffectComposer>
          {/* Desktop: orbit + zoom around the robot. Disabled in XR. */}
          <OrbitControls
            target={[0, 0.25, -1.5]}
            enablePan
            enableZoom
            enableRotate
            minDistance={0.4}
            maxDistance={12}
            maxPolarAngle={Math.PI * 0.495}
            zoomSpeed={0.9}
            rotateSpeed={0.8}
          />
        </NonXR>

        <SceneAnchor anchorRef={anchorRef}>
          <Suspense fallback={null}>
            <Robot />
          </Suspense>
          <FilterPose which="KF" color="#4dabff" sample={kf} />
          <FilterPose which="EKF" color="#6dd47e" sample={ekf} />
          <FilterPose which="PF" color="#ff5566" sample={pf} />
          <ParticleCloud />
          <ContactShadows
            position={[0, 0.001, 0]}
            opacity={0.55} blur={2.6} far={1.2} resolution={256}
            color="#000000" frames={1}
          />
        </SceneAnchor>

        <LaserScan anchorRef={anchorRef} />
        <AmclRunner anchorRef={anchorRef} />
      </XR>
    </Canvas>
  );
}
