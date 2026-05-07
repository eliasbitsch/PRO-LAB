import { Card, CardHeader, CardTitle, CardContent } from '@/components/ui/card';
import { Switch } from '@/components/ui/switch';
import { Slider } from '@/components/ui/slider';
import { Button } from '@/components/ui/button';
import { useStore, type RobotType, type ControlMode } from '@/lib/store';
import { connectRos } from '@/lib/ros';
import { xrStore } from './Scene';
import { ROBOT_INFO } from './Robot';
import { Activity, BoxSelect, Plug, Smartphone, Bot, Gamepad2, Hand, Power } from 'lucide-react';
import { useEffect, useState } from 'react';

function Dot({ color }: { color: string }) {
  return <span className="inline-block h-2.5 w-2.5 rounded-full" style={{ background: color }} />;
}

const ROBOTS: { id: RobotType; short: string }[] = [
  { id: 'tb3',    short: 'TB3' },
  { id: 'tb4',    short: 'TB4' },
  { id: 'mir100', short: 'MiR' },
  { id: 'taurob', short: 'Taurob' },
  { id: 'neura',  short: 'Neura' },
];

const MODES: { id: ControlMode; label: string; icon: any }[] = [
  { id: 'off',      label: 'Off',      icon: Power },
  { id: 'joystick', label: 'Joystick', icon: Gamepad2 },
  { id: 'waypoint', label: 'Waypoint', icon: Hand },
];

export function HUD() {
  const connected = useStore((s) => s.connected);
  const rosUrl = useStore((s) => s.rosUrl);
  const showKF = useStore((s) => s.showKF);
  const showEKF = useStore((s) => s.showEKF);
  const showPF = useStore((s) => s.showPF);
  const showParticles = useStore((s) => s.showParticles);
  const setShow = useStore((s) => s.setShow);
  const particleScale = useStore((s) => s.particleScale);
  const setParticleScale = useStore((s) => s.setParticleScale);
  const particles = useStore((s) => s.particles.length);
  const showFloorTiles = useStore((s) => s.showFloorTiles);
  const showLaserScan = useStore((s) => s.showLaserScan);
  const robotType = useStore((s) => s.robotType);
  const setRobotType = useStore((s) => s.setRobotType);
  const controlMode = useStore((s) => s.controlMode);
  const setControlMode = useStore((s) => s.setControlMode);
  const waypoint = useStore((s) => s.waypoint);
  const setWaypoint = useStore((s) => s.setWaypoint);

  const [arSupported, setArSupported] = useState(false);
  const [vrSupported, setVrSupported] = useState(false);
  const xrAvailable = typeof navigator !== 'undefined' && 'xr' in navigator;

  useEffect(() => {
    if (xrAvailable) {
      // @ts-ignore
      navigator.xr.isSessionSupported('immersive-ar').then(setArSupported).catch(() => setArSupported(false));
      // @ts-ignore
      navigator.xr.isSessionSupported('immersive-vr').then(setVrSupported).catch(() => setVrSupported(false));
    }
    connectRos(rosUrl);
  // eslint-disable-next-line
  }, []);

  return (
    <div className="pointer-events-none absolute inset-0 p-3 sm:p-5 flex flex-col gap-3">
      {/* Top bar */}
      <div className="pointer-events-auto flex items-center gap-3 justify-between">
        <Card className="px-3 py-2">
          <div className="flex items-center gap-2 text-xs">
            <Plug className={`h-3.5 w-3.5 ${connected ? 'text-emerald-400' : 'text-rose-400'}`} />
            <span className="text-muted-foreground">rosbridge</span>
            <span className="font-mono text-[11px]">{rosUrl}</span>
          </div>
        </Card>

        <div className="flex items-center gap-2">
          <Button
            variant="outline" size="sm"
            onClick={async () => {
              try { await xrStore.enterAR(); }
              catch (e: any) { alert('AR session failed: ' + (e?.message ?? e)); }
            }}
            disabled={!arSupported}
            className="pointer-events-auto"
          >
            <Smartphone className="h-4 w-4" /> Enter AR
          </Button>
          <Button
            variant="outline" size="sm"
            onClick={() => xrStore.enterVR().catch((e: any) => alert('VR session failed: ' + (e?.message ?? e)))}
            disabled={!vrSupported}
            className="pointer-events-auto"
          >
            <BoxSelect className="h-4 w-4" /> VR
          </Button>
          {!xrAvailable && (
            <span className="text-xs text-amber-400 ml-1">WebXR not in this browser</span>
          )}
        </div>
      </div>

      {/* Right side panel */}
      <div className="pointer-events-auto self-end ml-auto w-full max-w-xs space-y-3">
        {/* Robot switcher */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Bot className="h-4 w-4" /> Robot — {ROBOT_INFO[robotType].label}
            </CardTitle>
          </CardHeader>
          <CardContent>
            <div className="grid grid-cols-5 gap-1">
              {ROBOTS.map((r) => (
                <Button
                  key={r.id}
                  size="sm"
                  variant={robotType === r.id ? 'default' : 'outline'}
                  onClick={() => setRobotType(r.id)}
                  className="h-8 text-[10px] px-1"
                >
                  {r.short}
                </Button>
              ))}
            </div>
          </CardContent>
        </Card>

        {/* Control */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Gamepad2 className="h-4 w-4" /> Control
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-2">
            <div className="grid grid-cols-3 gap-1.5">
              {MODES.map((m) => (
                <Button
                  key={m.id}
                  size="sm"
                  variant={controlMode === m.id ? 'default' : 'outline'}
                  onClick={() => setControlMode(m.id)}
                  className="h-8 text-[11px]"
                >
                  <m.icon className="h-3 w-3 mr-1" /> {m.label}
                </Button>
              ))}
            </div>
            <div className="text-[10px] text-muted-foreground leading-snug">
              {controlMode === 'joystick' && 'Quest: L-stick move · R-stick turn. Desktop: WASD + Q/E.'}
              {controlMode === 'waypoint' && (waypoint
                ? `Driving to (${waypoint.x.toFixed(2)}, ${waypoint.z.toFixed(2)})`
                : 'Pinch on the floor to set a target.')}
              {controlMode === 'off' && 'Movement disabled. Use the grab gesture to reposition.'}
            </div>
          </CardContent>
        </Card>

        {/* Filters */}
        <Card>
          <CardHeader>
            <CardTitle className="flex items-center gap-2">
              <Activity className="h-4 w-4" /> Filters
            </CardTitle>
          </CardHeader>
          <CardContent className="space-y-3">
            <Row label="Kalman" color="#4dabff" checked={showKF} onCheck={(v) => setShow('KF', v)} />
            <Row label="Extended KF" color="#6dd47e" checked={showEKF} onCheck={(v) => setShow('EKF', v)} />
            <Row label="Particle" color="#ff5566" checked={showPF} onCheck={(v) => setShow('PF', v)} />
            <hr className="border-border/40" />
            <Row label="Particles" color="#ff3344" checked={showParticles} onCheck={(v) => setShow('Particles', v)} />
            <div className="space-y-1">
              <div className="flex items-center justify-between text-xs text-muted-foreground">
                <span>arrow size</span>
                <span className="font-mono">{particleScale.toFixed(2)}×</span>
              </div>
              <Slider
                value={[particleScale]} min={0.3} max={2.5} step={0.05}
                onValueChange={(v) => setParticleScale(v[0])}
              />
            </div>
            <hr className="border-border/40" />
            <Row label="Room outline" color="#1a44ff" checked={showFloorTiles} onCheck={(v) => setShow('FloorTiles', v)} />
            <Row label="Laser scan (heavy)" color="#ff2233" checked={showLaserScan} onCheck={(v) => setShow('LaserScan', v)} />
            <div className="text-[10px] text-muted-foreground font-mono pt-1">
              particles: {particles} · AMCL @ 1Hz
            </div>
          </CardContent>
        </Card>
      </div>

      {/* Hint bottom */}
      <div className="pointer-events-none mt-auto text-center text-xs text-muted-foreground space-y-0.5">
        <div>controller: aim ray at robot · trigger to grab</div>
        <div>hand: pinch on robot to grab · pinch on floor (waypoint mode) to drive</div>
      </div>
    </div>
  );
}

function Row({
  label, color, checked, onCheck,
}: { label: string; color: string; checked: boolean; onCheck: (v: boolean) => void }) {
  return (
    <div className="flex items-center justify-between">
      <div className="flex items-center gap-2 text-sm">
        <Dot color={color} /> <span>{label}</span>
      </div>
      <Switch checked={checked} onCheckedChange={onCheck} />
    </div>
  );
}
