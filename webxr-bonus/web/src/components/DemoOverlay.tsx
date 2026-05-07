import { useEffect, useRef, useState } from 'react';
import { Button } from '@/components/ui/button';
import { Card } from '@/components/ui/card';
import { Play, Square, SkipForward } from 'lucide-react';
import { useStore, type RobotType } from '@/lib/store';

/**
 * Demo flow with MP3 narration.
 *
 * Drop your audio files into `webxr-bonus/web/public/audio/`:
 *   - intro.mp3
 *   - step1_tb4.mp3, step2_tb3.mp3, step3_mir.mp3, step4_taurob.mp3
 *   - step5_filters.mp3, step6_drive.mp3, outro.mp3
 *
 * Each step:
 *  - shows headline + subtitle on screen
 *  - plays an MP3 (optional — silent fallback if missing)
 *  - can switch the active robot or toggle a feature
 *  - advances on audio `ended` OR after `maxMs` (whichever first), or via Skip button
 */

type Step = {
  audio: string;          // public path
  headline: string;
  subtitle: string;
  robot?: RobotType;
  enableLaser?: boolean;
  maxMs: number;          // hard cap if audio is missing/short
};

const STEPS: Step[] = [
  {
    audio: '/audio/intro.mp3',
    headline: 'Probabilistic Robotics — Live Demo',
    subtitle: 'Kalman · Extended Kalman · Particle Filter — visualized in mixed reality.',
    maxMs: 7000,
  },
  {
    audio: '/audio/step1_tb4.mp3',
    headline: 'TurtleBot 4',
    subtitle: 'Differential drive on a Create-3 base. RPLIDAR + OAK-D camera.',
    robot: 'tb4',
    maxMs: 7000,
  },
  {
    audio: '/audio/step2_tb3.mp3',
    headline: 'TurtleBot 3 (Burger)',
    subtitle: 'Compact ROS classroom platform. LDS-01 360° lidar.',
    robot: 'tb3',
    maxMs: 7000,
  },
  {
    audio: '/audio/step3_mir.mp3',
    headline: 'MiR 100',
    subtitle: 'Industrial AGV. Two SICK safety lasers, 100 kg payload.',
    robot: 'mir100',
    maxMs: 7000,
  },
  {
    audio: '/audio/step4_taurob.mp3',
    headline: 'Taurob Tracker',
    subtitle: 'Tracked UGV with 4-DOF manipulator — inspection & EOD.',
    robot: 'taurob',
    maxMs: 8000,
  },
  {
    audio: '/audio/step5_filters.mp3',
    headline: 'Three filters, one ground truth',
    subtitle: 'Blue=KF · Green=EKF · Red=PF. Watch the cloud converge to the pose.',
    enableLaser: true,
    maxMs: 9000,
  },
  {
    audio: '/audio/step6_drive.mp3',
    headline: 'Drive it yourself',
    subtitle: 'Left stick = move · Right stick = turn · Pinch on floor = waypoint.',
    maxMs: 9000,
  },
  {
    audio: '/audio/outro.mp3',
    headline: 'Filters running.',
    subtitle: 'HUD lets you toggle filters, change robot, switch control mode.',
    maxMs: 5000,
  },
];

export function DemoOverlay() {
  const demoActive = useStore((s) => s.demoActive);
  const demoStep = useStore((s) => s.demoStep);
  const demoCue = useStore((s) => s.demoCue);
  const setDemoActive = useStore((s) => s.setDemoActive);
  const setDemoStep = useStore((s) => s.setDemoStep);
  const setDemoCue = useStore((s) => s.setDemoCue);
  const setRobotType = useStore((s) => s.setRobotType);
  const setShow = useStore((s) => s.setShow);

  const audioRef = useRef<HTMLAudioElement | null>(null);
  const timerRef = useRef<number | null>(null);
  const [paused, setPaused] = useState(false);

  // Fire current step
  useEffect(() => {
    if (!demoActive) return;
    if (demoStep < 0 || demoStep >= STEPS.length) {
      // finished
      setDemoActive(false);
      setDemoCue('');
      setDemoStep(-1);
      return;
    }
    const step = STEPS[demoStep];
    setDemoCue(`${step.headline}\n${step.subtitle}`);
    if (step.robot) setRobotType(step.robot);
    if (step.enableLaser) setShow('LaserScan', true);

    // audio
    if (audioRef.current) {
      audioRef.current.pause();
      audioRef.current.src = '';
    }
    const a = new Audio(step.audio);
    a.preload = 'auto';
    a.volume = 0.95;
    audioRef.current = a;

    let advanced = false;
    const advance = () => {
      if (advanced) return;
      advanced = true;
      setDemoStep(demoStep + 1);
    };
    a.addEventListener('ended', advance);
    a.addEventListener('error', () => { /* missing mp3 — fall through to maxMs */ });
    a.play().catch(() => { /* autoplay blocked: rely on maxMs */ });

    timerRef.current = window.setTimeout(advance, step.maxMs);

    return () => {
      if (timerRef.current) window.clearTimeout(timerRef.current);
      a.pause();
      a.removeEventListener('ended', advance);
    };
  // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [demoActive, demoStep]);

  const start = () => {
    setPaused(false);
    setDemoActive(true);
    setDemoStep(0);
  };
  const stop = () => {
    if (audioRef.current) audioRef.current.pause();
    if (timerRef.current) window.clearTimeout(timerRef.current);
    setDemoActive(false);
    setDemoStep(-1);
    setDemoCue('');
  };
  const next = () => {
    if (audioRef.current) audioRef.current.pause();
    if (timerRef.current) window.clearTimeout(timerRef.current);
    setDemoStep(demoStep + 1);
  };
  const togglePause = () => {
    const a = audioRef.current;
    if (!a) return;
    if (paused) { a.play().catch(() => {}); setPaused(false); }
    else { a.pause(); setPaused(true); }
  };

  return (
    <>
      {/* Floating Start button when idle */}
      {!demoActive && (
        <div className="pointer-events-auto absolute left-1/2 -translate-x-1/2 top-20 z-30">
          <Button
            size="lg"
            onClick={start}
            className="h-14 px-8 text-base bg-emerald-500 hover:bg-emerald-400 text-black font-semibold"
          >
            <Play className="h-5 w-5 mr-2" />
            Start Demo
          </Button>
        </div>
      )}

      {/* Step cue card during demo */}
      {demoActive && demoCue && (
        <div className="pointer-events-none absolute left-1/2 -translate-x-1/2 top-16 z-30 w-[min(720px,90vw)]">
          <Card className="px-6 py-4 bg-black/70 backdrop-blur border-emerald-500/30">
            <div className="flex items-baseline gap-3">
              <span className="text-emerald-400 font-mono text-xs tracking-wider">
                STEP {demoStep + 1}/{STEPS.length}
              </span>
              <span className="text-xs text-muted-foreground">
                {STEPS[demoStep]?.robot ?? ''}
              </span>
            </div>
            <div className="mt-1 text-2xl font-semibold text-white leading-tight">
              {STEPS[demoStep]?.headline}
            </div>
            <div className="mt-1 text-sm text-zinc-300">
              {STEPS[demoStep]?.subtitle}
            </div>
            {/* Progress bar */}
            <div className="mt-3 h-1 w-full bg-white/10 rounded overflow-hidden">
              <div
                key={demoStep}
                className="h-full bg-emerald-400"
                style={{
                  animation: `demoProgress ${STEPS[demoStep]?.maxMs ?? 6000}ms linear forwards`,
                }}
              />
            </div>
          </Card>
        </div>
      )}

      {/* Demo controls (right side, only while active) */}
      {demoActive && (
        <div className="pointer-events-auto absolute left-4 top-20 z-30 flex flex-col gap-2">
          <Button size="sm" variant="outline" onClick={togglePause}>
            {paused ? <Play className="h-3.5 w-3.5" /> : <Square className="h-3.5 w-3.5" />}
            {paused ? 'Resume' : 'Pause'}
          </Button>
          <Button size="sm" variant="outline" onClick={next}>
            <SkipForward className="h-3.5 w-3.5" /> Next
          </Button>
          <Button size="sm" variant="outline" onClick={stop} className="border-rose-500/50 text-rose-300 hover:bg-rose-500/10">Stop</Button>
        </div>
      )}

      {/* keyframes */}
      <style>{`
        @keyframes demoProgress {
          from { width: 0%; }
          to { width: 100%; }
        }
      `}</style>
    </>
  );
}
