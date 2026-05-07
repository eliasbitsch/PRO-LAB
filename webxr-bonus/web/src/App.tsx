import Scene from '@/components/Scene';
import { HUD } from '@/components/HUD';
import { DemoOverlay } from '@/components/DemoOverlay';

export default function App() {
  return (
    <div className="relative h-full w-full">
      <Scene />
      <HUD />
      <DemoOverlay />
    </div>
  );
}
