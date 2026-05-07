/**
 * Client-side demo data generator. Runs at 30Hz; if no real ROS message has
 * arrived for >2s, it synthesizes a moving robot with KF/EKF/PF estimates and
 * a Gaussian particle cloud — no Gazebo required.
 *
 * The moment real /pf/pose etc. start flowing, the demo pauses and the real
 * data takes over.
 */
import { useStore, type Particle } from './store';

const STALE_MS = 2000;
const TICK_MS = 33;
const NUM_PARTICLES = 80;

let timerId: number | null = null;

function gauss() {
  // Box-Muller approximation for ~N(0,1)
  let u = 0, v = 0;
  while (u === 0) u = Math.random();
  while (v === 0) v = Math.random();
  return Math.sqrt(-2 * Math.log(u)) * Math.cos(2 * Math.PI * v);
}

export function startDemoLoop() {
  if (timerId !== null) return;

  const t0 = performance.now();
  timerId = window.setInterval(() => {
    const s = useStore.getState();
    const stale = Date.now() - s.lastRealDataMs > STALE_MS;
    if (!s.demoMode || !stale) return;

    const t = (performance.now() - t0) / 1000;

    // Robot trajectory: gentle figure-8 around origin in robot frame
    const r = 0.45;
    const x  = r * Math.sin(t * 0.6);
    const y  = r * Math.sin(t * 0.6) * Math.cos(t * 0.6);
    // Heading along trajectory tangent
    const dx = r * 0.6 * Math.cos(t * 0.6);
    const dy = r * 0.6 * (Math.cos(2 * t * 0.6));
    const yaw = Math.atan2(dy, dx);

    // Filters with slight drift relative to ground truth
    const kfDrift = 0.02 * Math.sin(t * 0.3);
    const ekfDrift = 0.012 * Math.cos(t * 0.4);
    const pfDrift = 0.005 * Math.sin(t * 0.7);

    s.setKF({  x: x + kfDrift,   y: y + kfDrift * 0.7, yaw: yaw + 0.04, covXY: 0.04, covYaw: 0.05 });
    s.setEKF({ x: x + ekfDrift,  y: y + ekfDrift,      yaw: yaw + 0.02, covXY: 0.02, covYaw: 0.03 });
    s.setPF({  x: x + pfDrift,   y: y + pfDrift,       yaw: yaw,        covXY: 0.012, covYaw: 0.02 });

    // Particle cloud — gaussian around the PF estimate
    const std = 0.06 + 0.02 * Math.sin(t * 0.5);
    const stdYaw = 0.18;
    const particles: Particle[] = [];
    for (let i = 0; i < NUM_PARTICLES; ++i) {
      particles.push({
        x: x + gauss() * std,
        y: y + gauss() * std,
        yaw: yaw + gauss() * stdYaw,
      });
    }
    s.setParticles(particles);

    // setKF/setEKF/setPF/setParticles updated lastRealDataMs as a side-effect.
    // Reset it manually so we keep generating in demo mode.
    useStore.setState({ lastRealDataMs: 0 });
  }, TICK_MS);
}

export function stopDemoLoop() {
  if (timerId !== null) {
    clearInterval(timerId);
    timerId = null;
  }
}
