/**
 * Shared world geometry registry.
 *   - floorPolygons: drivable surfaces (used for tile + particle culling)
 *   - obstaclePolygons: any other detected horizontal/vertical plane projected to
 *                       Y=0 (tables, couches, walls). Used for inflation gradient
 *                       and laser-scan ray casts.
 */
export type Vec2 = { x: number; z: number };
export type FloorPolygon = { polygon: Vec2[]; y: number };
export type ObstaclePolygon = { polygon: Vec2[] };

let floors: FloorPolygon[] = [];
let obstacles: ObstaclePolygon[] = [];

export function setFloorPolygons(p: FloorPolygon[]) { floors = p; }
export function getFloorPolygons() { return floors; }
export function setObstaclePolygons(p: ObstaclePolygon[]) { obstacles = p; }
export function getObstaclePolygons() { return obstacles; }

export function pointInPolygon(x: number, z: number, poly: Vec2[]): boolean {
  let inside = false;
  for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
    const xi = poly[i].x, zi = poly[i].z;
    const xj = poly[j].x, zj = poly[j].z;
    if (((zi > z) !== (zj > z)) && (x < (xj - xi) * (z - zi) / (zj - zi) + xi)) {
      inside = !inside;
    }
  }
  return inside;
}

export function pointOnAnyFloor(x: number, z: number): boolean {
  if (floors.length === 0) return true;
  return floors.some((fp) => pointInPolygon(x, z, fp.polygon));
}

export function pointInAnyObstacle(x: number, z: number): boolean {
  for (const o of obstacles) {
    if (pointInPolygon(x, z, o.polygon)) return true;
  }
  return false;
}

/** Drivable = on a floor polygon AND not inside any obstacle (table/wall) footprint. */
export function pointIsDrivable(x: number, z: number): boolean {
  return pointOnAnyFloor(x, z) && !pointInAnyObstacle(x, z);
}

/** Squared distance from (px,pz) to segment (a,b). */
function distSqToSegment(px: number, pz: number, ax: number, az: number, bx: number, bz: number): number {
  const abx = bx - ax;
  const abz = bz - az;
  const apx = px - ax;
  const apz = pz - az;
  const lenSq = abx * abx + abz * abz;
  let t = lenSq > 0 ? (apx * abx + apz * abz) / lenSq : 0;
  if (t < 0) t = 0; else if (t > 1) t = 1;
  const cx = ax + t * abx;
  const cz = az + t * abz;
  const dx = px - cx;
  const dz = pz - cz;
  return dx * dx + dz * dz;
}

/** Min distance (m) from (x,z) to any obstacle polygon edge. Inf if no obstacles. */
export function distanceToObstacle(x: number, z: number): number {
  let minSq = Infinity;
  for (const o of obstacles) {
    const poly = o.polygon;
    if (pointInPolygon(x, z, poly)) return 0; // inside obstacle
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const d2 = distSqToSegment(x, z, poly[i].x, poly[i].z, poly[j].x, poly[j].z);
      if (d2 < minSq) minSq = d2;
    }
  }
  return Math.sqrt(minSq);
}

/** Ray-segment intersection. Returns t along ray (>=0), or Infinity if no hit. */
export function raySegmentT(
  ox: number, oz: number, dx: number, dz: number,
  ax: number, az: number, bx: number, bz: number,
): number {
  const sx = bx - ax;
  const sz = bz - az;
  const denom = dx * sz - dz * sx;
  if (Math.abs(denom) < 1e-9) return Infinity;
  const ux = ax - ox;
  const uz = az - oz;
  const t = (ux * sz - uz * sx) / denom;            // along ray
  const u = (ux * dz - uz * dx) / denom;            // along segment
  if (t < 0 || u < 0 || u > 1) return Infinity;
  return t;
}

/** Cast a ray from (ox,oz) in direction (dx,dz) (unit). Returns hit (x,z) or null. */
export function raycastObstacles(
  ox: number, oz: number, dx: number, dz: number, maxDist = 8,
): { x: number; z: number; dist: number } | null {
  let bestT = maxDist;
  for (const o of obstacles) {
    const poly = o.polygon;
    for (let i = 0, j = poly.length - 1; i < poly.length; j = i++) {
      const t = raySegmentT(ox, oz, dx, dz, poly[i].x, poly[i].z, poly[j].x, poly[j].z);
      if (t < bestT) bestT = t;
    }
  }
  if (bestT >= maxDist) return null;
  return { x: ox + dx * bestT, z: oz + dz * bestT, dist: bestT };
}
