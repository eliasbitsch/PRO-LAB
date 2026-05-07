import React from 'react';
import ReactDOM from 'react-dom/client';
import App from './App';
import './index.css';

// Inject extra optional features into every WebXR session so plane-detection,
// hand-tracking, mesh-detection, hit-test and anchors are available without
// having to thread session init options through @react-three/xr.
if (typeof navigator !== 'undefined' && (navigator as any).xr) {
  const xr = (navigator as any).xr as XRSystem;
  const orig = xr.requestSession.bind(xr);
  // Only ask for what we actually use. Quest may decline a session if too
  // many features are requested or one isn't supported in the current build.
  const EXTRAS = [
    'plane-detection',
    'hand-tracking',
    'local-floor',
  ];
  (xr as any).requestSession = (mode: XRSessionMode, init?: XRSessionInit) => {
    const merged: XRSessionInit = {
      ...(init ?? {}),
      optionalFeatures: Array.from(
        new Set([...(init?.optionalFeatures ?? []), ...EXTRAS])
      ),
    };
    return orig(mode, merged);
  };
}

ReactDOM.createRoot(document.getElementById('root')!).render(
  <React.StrictMode>
    <App />
  </React.StrictMode>
);
