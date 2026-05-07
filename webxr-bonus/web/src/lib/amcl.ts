/**
 * AMCL-style particle filter that uses Quest's detected planes (walls + tables)
 * as the map.
 *
 *   1. Init: scatter N particles across the detected floor polygon.
 *   2. Predict: small Gaussian noise on each particle (no odometry available).
 *   3. Update: each particle simulates a 36-ray scan from its hypothetical pose
 *              and is weighted by similarity to the *real* scan from the robot.
 *   4. Resample: systematic resampling.
 *   5. Inject: small post-resample noise (regularization) to avoid degeneracy.
 *
 * The "real scan" comes from the actual robot pose (the AR scene-anchor's
 * world position) using the same raycastObstacles primitive — so the filter
 * has a perfectly consistent simulated sensor.
 */
import {
  raycastObstacles, pointIsDrivable, getFloorPolygons,
} from './floor';

export type Particle = { x: number; y: number; yaw: number };

const NUM_PARTICLES = 80;
const NUM_SCAN_RAYS = 36;            // 10° resolution
const SCAN_MAX_DIST = 6.0;
const SCAN_SIGMA = 0.20;              // m  — sensor stdev for likelihood
const PREDICT_NOISE_XY = 0.04;        // m  per tick
const PREDICT_NOISE_YAW = 0.06;       // rad per tick
const REGULARIZATION_XY = 0.025;      // m  injected after resample
const REGULARIZATION_YAW = 0.04;
const NEFF_THRESHOLD = NUM_PARTICLES / 2;

let particles: Particle[] = [];

function randn(): number {
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

function bboxOfFloors(): { minX: number; maxX: number; minZ: number; maxZ: number } | null {
  const floors = getFloorPolygons();
  if (floors.length === 0) return null;
  let minX = Infinity, maxX = -Infinity, minZ = Infinity, maxZ = -Infinity;
  for (const f of floors) for (const v of f.polygon) {
    if (v.x < minX) minX = v.x; if (v.x > maxX) maxX = v.x;
    if (v.z < minZ) minZ = v.z; if (v.z > maxZ) maxZ = v.z;
  }
  return { minX, maxX, minZ, maxZ };
}

function init(): void {
  particles = [];
  const bb = bboxOfFloors();
  if (!bb) return;
  let attempts = 0;
  while (particles.length < NUM_PARTICLES && attempts < 5000) {
    attempts++;
    const x = bb.minX + Math.random() * (bb.maxX - bb.minX);
    const z = bb.minZ + Math.random() * (bb.maxZ - bb.minZ);
    if (!pointIsDrivable(x, z)) continue;
    particles.push({ x, y: z, yaw: Math.random() * Math.PI * 2 });
  }
}

function simulateScan(x: number, z: number, yaw: number): Float32Array {
  const out = new Float32Array(NUM_SCAN_RAYS);
  for (let i = 0; i < NUM_SCAN_RAYS; ++i) {
    const a = yaw + (i / NUM_SCAN_RAYS) * Math.PI * 2;
    const hit = raycastObstacles(x, z, Math.cos(a), Math.sin(a), SCAN_MAX_DIST);
    out[i] = hit ? hit.dist : SCAN_MAX_DIST;
  }
  return out;
}

function predict(): void {
  for (const p of particles) {
    p.x += randn() * PREDICT_NOISE_XY;
    p.y += randn() * PREDICT_NOISE_XY;
    p.yaw += randn() * PREDICT_NOISE_YAW;
  }
}

function weights(realScan: Float32Array): Float32Array {
  const w = new Float32Array(particles.length);
  const denom = 2 * SCAN_SIGMA * SCAN_SIGMA * NUM_SCAN_RAYS;
  for (let i = 0; i < particles.length; ++i) {
    const p = particles[i];
    if (!pointIsDrivable(p.x, p.y)) { w[i] = 0; continue; }
    const expected = simulateScan(p.x, p.y, p.yaw);
    let err = 0;
    for (let j = 0; j < NUM_SCAN_RAYS; ++j) {
      const d = expected[j] - realScan[j];
      err += d * d;
    }
    w[i] = Math.exp(-err / denom);
  }
  return w;
}

function resample(w: Float32Array): boolean {
  let sum = 0;
  for (const v of w) sum += v;
  if (sum < 1e-12) return false;                // total degeneracy → caller reinitializes
  for (let i = 0; i < w.length; ++i) w[i] /= sum;

  // Effective sample size — only resample when degeneration is significant
  let nEff = 0;
  for (const v of w) nEff += v * v;
  nEff = 1 / nEff;
  if (nEff > NEFF_THRESHOLD) return true;

  // Systematic resampling
  const newP: Particle[] = new Array(NUM_PARTICLES);
  const r = Math.random() / NUM_PARTICLES;
  let c = w[0];
  let i = 0;
  for (let m = 0; m < NUM_PARTICLES; ++m) {
    const u = r + m / NUM_PARTICLES;
    while (u > c && i < NUM_PARTICLES - 1) { i++; c += w[i]; }
    const src = particles[i];
    newP[m] = {
      x: src.x + randn() * REGULARIZATION_XY,
      y: src.y + randn() * REGULARIZATION_XY,
      yaw: src.yaw + randn() * REGULARIZATION_YAW,
    };
  }
  particles = newP;
  return true;
}

/** Run one PF step with the given true robot pose. Returns the current set
 *  of particles in (x_world, z_world, yaw) form. */
export function tick(robotX: number, robotZ: number, robotYaw: number): Particle[] {
  if (particles.length === 0) init();
  if (particles.length === 0) return [];        // no floor detected yet

  predict();
  const realScan = simulateScan(robotX, robotZ, robotYaw);
  const w = weights(realScan);
  if (!resample(w)) {
    init();
    return particles.map((p) => ({ ...p }));
  }
  return particles.map((p) => ({ ...p }));
}

export function reset(): void { particles = []; }
