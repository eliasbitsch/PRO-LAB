/**
 * Drives the AMCL particle filter: once per second, reads the robot's world
 * pose from the SceneAnchor ref, runs amcl.tick(), and pushes the new particle
 * set into the store. ParticleCloud handles the per-frame lerp.
 */
import { useEffect } from 'react';
import * as THREE from 'three';
import { useThree } from '@react-three/fiber';
import { useXR } from '@react-three/xr';
import { tick as amclTick } from '@/lib/amcl';
import { useStore, type Particle } from '@/lib/store';

const TICK_INTERVAL_MS = 1000;

export function AmclRunner({ anchorRef }: { anchorRef: React.RefObject<THREE.Group | null> }) {
  const session = useXR((s) => s.session);
  const { gl } = useThree();

  useEffect(() => {
    const tmpPos = new THREE.Vector3();
    const tmpEuler = new THREE.Euler(0, 0, 0, 'YXZ');
    const id = window.setInterval(() => {
      if (!session) return;
      const anchor = anchorRef.current;
      if (!anchor) return;
      anchor.getWorldPosition(tmpPos);
      tmpEuler.setFromQuaternion(anchor.getWorldQuaternion(new THREE.Quaternion()));
      const robotYaw = tmpEuler.y;

      // Robot pose in world XZ. Note: amcl operates on (x, y=z, yaw).
      const result = amclTick(tmpPos.x, tmpPos.z, robotYaw);
      // Convert to store Particle format. Note: store.Particle.y is the ROS y
      // (which we treat as world Z negated for visualization), so we emit raw
      // world XZ; ParticleCloud converts to local. To keep ParticleCloud's
      // existing transform (p.x -> local x, -p.y -> local z) we need to turn
      // world (X, Z) into the (x, y) in robot-local space by inverting the
      // anchor matrix.
      const inv = new THREE.Matrix4().copy(anchor.matrixWorld).invert();
      const v = new THREE.Vector3();
      const local: Particle[] = result.map((p) => {
        v.set(p.x, 0, p.y).applyMatrix4(inv);
        return { x: v.x, y: -v.z, yaw: p.yaw };
      });
      useStore.getState().setParticles(local);
    }, TICK_INTERVAL_MS);

    return () => window.clearInterval(id);
  }, [session, anchorRef, gl]);

  return null;
}
