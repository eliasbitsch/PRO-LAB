/**
 * Meta-style ghost-hands. Reads XRHand joint poses each frame and renders:
 *   - 25 joint spheres per hand (slightly larger toward the wrist)
 *   - 28 bone cylinders connecting joints
 * with a translucent fresnel-glow material — opaque at grazing angles, see-
 * through in the center, just like Meta's system UI hands.
 */
import { useMemo, useRef } from 'react';
import * as THREE from 'three';
import { useFrame, useThree } from '@react-three/fiber';
import { useXR } from '@react-three/xr';

const JOINTS: XRHandJoint[] = [
  'wrist',
  'thumb-metacarpal', 'thumb-phalanx-proximal', 'thumb-phalanx-distal', 'thumb-tip',
  'index-finger-metacarpal', 'index-finger-phalanx-proximal', 'index-finger-phalanx-intermediate', 'index-finger-phalanx-distal', 'index-finger-tip',
  'middle-finger-metacarpal', 'middle-finger-phalanx-proximal', 'middle-finger-phalanx-intermediate', 'middle-finger-phalanx-distal', 'middle-finger-tip',
  'ring-finger-metacarpal', 'ring-finger-phalanx-proximal', 'ring-finger-phalanx-intermediate', 'ring-finger-phalanx-distal', 'ring-finger-tip',
  'pinky-finger-metacarpal', 'pinky-finger-phalanx-proximal', 'pinky-finger-phalanx-intermediate', 'pinky-finger-phalanx-distal', 'pinky-finger-tip',
];

const BONES: [number, number][] = [
  [0, 1], [1, 2], [2, 3], [3, 4],
  [0, 5], [5, 6], [6, 7], [7, 8], [8, 9],
  [0, 10], [10, 11], [11, 12], [12, 13], [13, 14],
  [0, 15], [15, 16], [16, 17], [17, 18], [18, 19],
  [0, 20], [20, 21], [21, 22], [22, 23], [23, 24],
  [5, 10], [10, 15], [15, 20],
];

const MAX_HANDS = 2;
const JOINTS_PER_HAND = JOINTS.length;
const BONES_PER_HAND = BONES.length;
const TOTAL_JOINTS = MAX_HANDS * JOINTS_PER_HAND;
const TOTAL_BONES  = MAX_HANDS * BONES_PER_HAND;

// Fresnel-glow ghost material. MeshStandardMaterial + shader injection.
function makeGhostMaterial() {
  const m = new THREE.MeshStandardMaterial({
    color: 0xffffff,
    transparent: true,
    opacity: 1.0,
    depthWrite: false,
    side: THREE.FrontSide,
    roughness: 0.45,
    metalness: 0.0,
  });
  m.onBeforeCompile = (shader) => {
    shader.vertexShader = shader.vertexShader
      .replace('#include <common>', `
        #include <common>
        varying vec3 vWorldPos;
        varying vec3 vWorldNormal;
      `)
      .replace('#include <begin_vertex>', `
        #include <begin_vertex>
        vWorldPos = (modelMatrix * vec4(position, 1.0)).xyz;
        vWorldNormal = normalize(mat3(modelMatrix) * normal);
      `);
    shader.fragmentShader = shader.fragmentShader
      .replace('#include <common>', `
        #include <common>
        varying vec3 vWorldPos;
        varying vec3 vWorldNormal;
      `)
      .replace('#include <opaque_fragment>', `
        vec3 viewDir = normalize(cameraPosition - vWorldPos);
        float fresnel = pow(1.0 - abs(dot(viewDir, vWorldNormal)), 2.5);
        // Boost emissive at grazing angles (rim glow)
        outgoingLight += vec3(fresnel * 1.4);
        // Alpha: see-through center (~0.18), opaque edges (~0.95)
        diffuseColor.a = mix(0.18, 0.95, fresnel);
        gl_FragColor = vec4(outgoingLight, diffuseColor.a);
      `);
  };
  return m;
}

const tmpObj = new THREE.Object3D();
const tmpMat = new THREE.Matrix4();
const tmpVec = new THREE.Vector3();
const tmpVec2 = new THREE.Vector3();
const Y_AXIS = new THREE.Vector3(0, 1, 0);

export function MetaStyleHands() {
  const session = useXR((s) => s.session);
  const { gl } = useThree();
  const jointRef = useRef<THREE.InstancedMesh>(null);
  const boneRef = useRef<THREE.InstancedMesh>(null);

  // Slim joint sphere — natural hand proportions, the fresnel glow does the
  // "feeling solid" work.
  const jointGeo = useMemo(() => new THREE.SphereGeometry(0.007, 14, 10), []);
  // Slim bone — thin so it reads as a finger, not a sausage.
  const boneGeo = useMemo(() => {
    const g = new THREE.CylinderGeometry(0.004, 0.004, 1, 12, 1);
    g.translate(0, 0.5, 0);
    return g;
  }, []);

  const ghostMat = useMemo(makeGhostMaterial, []);

  useFrame((_state, _dt, xrFrame?: XRFrame) => {
    const refSpace = gl.xr.getReferenceSpace();
    const jm = jointRef.current;
    const bm = boneRef.current;
    if (!session || !xrFrame || !refSpace || !jm || !bm) {
      if (jm) jm.count = 0;
      if (bm) bm.count = 0;
      return;
    }

    const positions: (THREE.Vector3 | null)[] = new Array(TOTAL_JOINTS).fill(null);
    let jointIdx = 0;
    let handSlot = 0;

    for (const src of session.inputSources) {
      if (!src.hand || handSlot >= MAX_HANDS) continue;
      for (let i = 0; i < JOINTS_PER_HAND; ++i) {
        const joint = (src.hand as any).get(JOINTS[i]) as XRJointSpace | undefined;
        if (!joint) { jointIdx++; continue; }
        const pose = xrFrame.getJointPose ? xrFrame.getJointPose(joint, refSpace) : null;
        if (!pose) { jointIdx++; continue; }
        tmpMat.fromArray(pose.transform.matrix);
        const p = new THREE.Vector3().setFromMatrixPosition(tmpMat);
        positions[handSlot * JOINTS_PER_HAND + i] = p;
        // Joint radius from WebXR ranges ~5–14mm; we keep a smaller base sphere
        // and only mildly scale by it so finger tips stay slim.
        const r = pose.radius ?? 0.007;
        tmpObj.position.copy(p);
        tmpObj.scale.setScalar(0.7 + (r / 0.007) * 0.3);
        tmpObj.rotation.set(0, 0, 0);
        tmpObj.updateMatrix();
        jm.setMatrixAt(jointIdx, tmpObj.matrix);
        jointIdx++;
      }
      handSlot++;
    }
    jm.count = jointIdx;
    jm.instanceMatrix.needsUpdate = true;

    let boneIdx = 0;
    for (let h = 0; h < handSlot; ++h) {
      for (const [a, b] of BONES) {
        const pa = positions[h * JOINTS_PER_HAND + a];
        const pb = positions[h * JOINTS_PER_HAND + b];
        if (!pa || !pb) continue;
        tmpVec.copy(pa);
        tmpVec2.copy(pb).sub(pa);
        const len = tmpVec2.length();
        if (len < 1e-5) continue;
        const dir = tmpVec2.clone().normalize();
        const q = new THREE.Quaternion().setFromUnitVectors(Y_AXIS, dir);
        tmpObj.position.copy(tmpVec);
        tmpObj.quaternion.copy(q);
        tmpObj.scale.set(1, len, 1);
        tmpObj.updateMatrix();
        bm.setMatrixAt(boneIdx, tmpObj.matrix);
        boneIdx++;
      }
    }
    bm.count = boneIdx;
    bm.instanceMatrix.needsUpdate = true;
  });

  return (
    <group>
      <instancedMesh ref={jointRef} args={[jointGeo, ghostMat, TOTAL_JOINTS]} frustumCulled={false} renderOrder={999} />
      <instancedMesh ref={boneRef}  args={[boneGeo,  ghostMat, TOTAL_BONES]}  frustumCulled={false} renderOrder={999} />
    </group>
  );
}
