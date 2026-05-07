import { create } from 'zustand';

export type PoseSample = {
  x: number; y: number; yaw: number; covXY?: number; covYaw?: number;
};

export type Particle = { x: number; y: number; yaw: number };

export type RobotType = 'tb3' | 'tb4' | 'mir100' | 'taurob' | 'neura';

export type ControlMode = 'off' | 'joystick' | 'waypoint';

type State = {
  connected: boolean;
  rosUrl: string;
  kf: PoseSample | null;
  ekf: PoseSample | null;
  pf: PoseSample | null;
  particles: Particle[];
  showKF: boolean;
  showEKF: boolean;
  showPF: boolean;
  showParticles: boolean;
  particleScale: number;
  scenePlaced: boolean;
  showFloorTiles: boolean;
  showLaserScan: boolean;
  lastRealDataMs: number;
  demoMode: boolean;

  // Robot switcher
  robotType: RobotType;

  // Movement
  controlMode: ControlMode;
  // Driving target in WORLD space (XZ plane). When set, robot drives toward it.
  waypoint: { x: number; z: number } | null;
  // True velocity in world coords (m/s). Updated by RobotMover, read by anyone needing it.
  driveVelXZ: { vx: number; vz: number; w: number };

  // Demo flow
  demoActive: boolean;
  demoStep: number;            // -1 = idle, 0..N-1 = current step
  demoCue: string;             // current on-screen text

  setConnected: (v: boolean) => void;
  setKF: (p: PoseSample) => void;
  setEKF: (p: PoseSample) => void;
  setPF: (p: PoseSample) => void;
  setParticles: (p: Particle[]) => void;
  setRosUrl: (s: string) => void;
  setShow: (which: 'KF' | 'EKF' | 'PF' | 'Particles' | 'FloorTiles' | 'LaserScan', v: boolean) => void;
  setParticleScale: (v: number) => void;
  setScenePlaced: (v: boolean) => void;
  setDemoMode: (v: boolean) => void;
  setRobotType: (t: RobotType) => void;
  setControlMode: (m: ControlMode) => void;
  setWaypoint: (w: { x: number; z: number } | null) => void;
  setDriveVel: (v: { vx: number; vz: number; w: number }) => void;
  setDemoActive: (v: boolean) => void;
  setDemoStep: (n: number) => void;
  setDemoCue: (s: string) => void;
};

export const useStore = create<State>((set) => ({
  connected: false,
  rosUrl: typeof location !== 'undefined'
    ? (location.protocol === 'https:'
        ? `wss://${location.host}/ws`
        : `ws://${location.hostname}:9090`)
    : 'ws://localhost:9090',
  kf: null, ekf: null, pf: null, particles: [],
  showKF: true, showEKF: true, showPF: true, showParticles: true,
  particleScale: 1.0,
  scenePlaced: false,
  showFloorTiles: false,
  showLaserScan: false,
  lastRealDataMs: 0,
  demoMode: true,

  robotType: 'tb4',
  controlMode: 'joystick',
  waypoint: null,
  driveVelXZ: { vx: 0, vz: 0, w: 0 },

  demoActive: false,
  demoStep: -1,
  demoCue: '',

  setConnected: (v) => set({ connected: v }),
  setKF: (p) => set({ kf: p, lastRealDataMs: Date.now() }),
  setEKF: (p) => set({ ekf: p, lastRealDataMs: Date.now() }),
  setPF: (p) => set({ pf: p, lastRealDataMs: Date.now() }),
  setParticles: (p) => set({ particles: p, lastRealDataMs: Date.now() }),
  setRosUrl: (s) => set({ rosUrl: s }),
  setShow: (which, v) => set(() => ({
    ...(which === 'KF' ? { showKF: v } : {}),
    ...(which === 'EKF' ? { showEKF: v } : {}),
    ...(which === 'PF' ? { showPF: v } : {}),
    ...(which === 'Particles' ? { showParticles: v } : {}),
    ...(which === 'FloorTiles' ? { showFloorTiles: v } : {}),
    ...(which === 'LaserScan' ? { showLaserScan: v } : {}),
  })),
  setParticleScale: (v) => set({ particleScale: v }),
  setScenePlaced: (v) => set({ scenePlaced: v }),
  setDemoMode: (v) => set({ demoMode: v }),
  setRobotType: (t) => set({ robotType: t }),
  setControlMode: (m) => set({ controlMode: m, waypoint: null }),
  setWaypoint: (w) => set({ waypoint: w }),
  setDriveVel: (v) => set({ driveVelXZ: v }),
  setDemoActive: (v) => set({ demoActive: v }),
  setDemoStep: (n) => set({ demoStep: n }),
  setDemoCue: (s) => set({ demoCue: s }),
}));
