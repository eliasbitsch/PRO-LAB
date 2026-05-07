# PRO-LAB WebXR Bonus

**Komplett separates Bonus-Projekt** — touched keinen Filter-Code.

## Stack

- **Frontend**: Vite + React 18 + TypeScript + Tailwind + shadcn/ui
- **3D**: Three.js + @react-three/fiber + @react-three/drei + @react-three/xr
- **State**: Zustand
- **ROS**: roslibjs ↔ rosbridge_websocket
- **Container**: Multi-stage (Node-Builder → Nginx-Runtime)
- **HTTPS**: self-signed cert (WebXR braucht secure context)

## Was es macht

- **Immersive AR** (Quest 3 / Vision Pro / Android Chrome) — Roboter im echten Raum
- **Distance-Grab**: Controller-Trigger auf Roboter halten + Hand bewegen → Roboter folgt
- **Particle Cloud**: rote Pfeile (sampled aus `/pf/pose` Kovarianz, ~80 Pfeile)
- **Filter-Schätzungen**: KF (blau) / EKF (grün) / PF (rot) als 3 distinct Arrows
- **Schatten** (DirectionalLight + ContactShadows)
- **HDRI Reflexionen** (Environment preset "city")
- **Toggle Panel** (shadcn/ui): Filter ein/aus, Partikel-Größe-Slider

## Architektur

```
prolab_jazzy (Hauptcontainer)        prolab_webxr (Bonus, separat)
─────────────────────────            ─────────────────────────────
gz sim, Nav2, AMCL                   particle_sampler.py
                                     ├── /pf/pose (subscribe)
pf_node ─/pf/pose──────────────────▶ └── /webxr/particles (publish)
kf_node ─/kf/pose──────────────────▶ rosbridge_websocket :9090
ekf_node ─/ekf/pose─────────────────▶ nginx :8080 (serves dist/)
                                          │
                                          ▼
                                     Browser/Headset → WebXR
```

ROS-Code bleibt unangetastet. Particle-Sampling rein im Bonus-Container.

## Modell

Das App lädt automatisch `/turtlebot4.glb` falls vorhanden, sonst Procedural-Fallback (sehr passable TB4-Silhouette: Create-3-Base + Tower + RPLIDAR + OAK-D).

GLB-Modell selbst hinzufügen (optional):
1. Lade ein `turtlebot4.glb` von z.B.
   - https://github.com/Open-Robotics/open-robotics-models
   - https://sketchfab.com/3d-models (Suche "turtlebot4" → Download as glTF)
   - Eigene Konvertierung aus den `nav2_minimal_tb4_description` STLs via Blender
2. Lege es nach `web/public/turtlebot4.glb`
3. Container neu bauen

## Starten

```bash
# Hauptsetup muss laufen (gz sim + filter nodes)
cd PRO-LAB/docker && docker compose up -d
docker exec prolab_jazzy bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ws/install/setup.bash && ros2 launch pro_lab_filters all_in_one.launch.py" &
docker exec prolab_jazzy bash -lc "source /opt/ros/jazzy/setup.bash && source /home/ros/ws/install/setup.bash && ros2 launch pro_lab_filters filters.launch.py" &

# Bonus-Container
cd PRO-LAB/webxr-bonus/docker && docker compose up -d --build

# WSL-IP rauskriegen
ip addr show eth0 | grep inet
```

Im Browser auf `https://<wsl-ip>:8080` (Zertifikat-Warnung akzeptieren).

**Quest 3:** Browser auf gleichem Netz öffnen → "Enter AR" Button → Robot erscheint im Raum.

## Dev-Mode (ohne Container, schneller Iterieren)

```bash
cd web/
npm install
npm run dev
# Vite serviert auf http://localhost:5173 (HTTP) oder mit --https für WebXR-Tests
```

Dann braucht's noch rosbridge auf Port 9090 — entweder:
- Bonus-Container nur für rosbridge laufen lassen (`docker compose up -d`), oder
- Lokal via `ros2 launch rosbridge_server rosbridge_websocket_launch.xml`
