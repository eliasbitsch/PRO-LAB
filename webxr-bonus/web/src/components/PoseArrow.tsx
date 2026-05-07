import * as THREE from 'three';
import { useMemo } from 'react';

type Props = {
  color: string;
  length?: number;
  emissiveIntensity?: number;
};

export function PoseArrow({ color, length = 0.55, emissiveIntensity = 0.4 }: Props) {
  const mat = useMemo(() => new THREE.MeshStandardMaterial({
    color,
    emissive: new THREE.Color(color).multiplyScalar(0.6),
    emissiveIntensity,
    metalness: 0.4,
    roughness: 0.3,
  }), [color, emissiveIntensity]);

  return (
    <group rotation={[0, 0, -Math.PI / 2]}>
      {/* shaft along +Y after rotation -> +X in world */}
      <mesh position={[0, length / 2, 0]} material={mat} castShadow>
        <cylinderGeometry args={[0.012, 0.012, length, 12]} />
      </mesh>
      <mesh position={[0, length + 0.06, 0]} material={mat} castShadow>
        <coneGeometry args={[0.04, 0.12, 16]} />
      </mesh>
    </group>
  );
}
