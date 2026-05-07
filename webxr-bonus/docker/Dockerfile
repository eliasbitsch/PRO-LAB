# Multi-stage: Node builds the Vite app, then ROS+nginx runtime serves it.
FROM node:20-alpine AS web-builder
WORKDIR /web
COPY web/package.json ./
RUN npm install --no-audit --no-fund
COPY web/ ./

# Optional GLB compression for assets dropped into web/public/.
# Drop the raw NEURA MAV at: webxr-bonus/web/public/neura_mav_raw.glb
# Build will compress it (Draco mesh + WebP textures + dedupe) into neura_mav.glb.
RUN if [ -f /web/public/neura_mav_raw.glb ]; then \
      echo "Compressing NEURA MAV GLB..." && \
      npm install -g @gltf-transform/cli@^4 && \
      apk add --no-cache python3 make g++ libwebp-tools && \
      gltf-transform optimize /web/public/neura_mav_raw.glb /web/public/neura_mav.glb \
        --compress draco --texture-compress webp --texture-size 1024 \
      && ls -la /web/public/neura_mav*.glb \
      && rm /web/public/neura_mav_raw.glb ; \
    else \
      echo "No neura_mav_raw.glb found — skipping compression"; \
    fi

RUN npm run build

FROM ros:jazzy-ros-base
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -y --no-install-recommends \
    git \
    python3-numpy \
    python3-pip \
    python3-yaml \
    nginx \
    openssl \
    assimp-utils \
    ros-jazzy-rosbridge-server \
    ros-jazzy-rosbridge-suite \
    ros-jazzy-nav2-minimal-tb4-description \
    ros-jazzy-turtlebot4-description \
    ros-jazzy-irobot-create-description \
    ros-jazzy-xacro \
    && rm -rf /var/lib/apt/lists/*

# Build robot GLBs from URDF + meshes at image-build time so the static web
# payload includes all real models.
RUN pip install --break-system-packages --no-cache-dir trimesh==4.5.2 \
    pycollada==0.8 numpy lxml scipy

# HTTPS cert (WebXR requires secure context)
RUN mkdir -p /etc/nginx/ssl && \
    openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
        -subj "/CN=prolab-webxr" \
        -keyout /etc/nginx/ssl/server.key \
        -out /etc/nginx/ssl/server.crt 2>/dev/null

COPY docker/nginx.conf /etc/nginx/nginx.conf
COPY docker/entrypoint.sh /entrypoint.sh
COPY docker/build_tb4_glb.py    /tmp/build_tb4_glb.py
COPY docker/build_tb3_glb.py    /tmp/build_tb3_glb.py
COPY docker/build_mir_glb.py    /tmp/build_mir_glb.py
COPY docker/build_taurob_glb.py /tmp/build_taurob_glb.py
COPY bridge /home/ros/bridge
COPY --from=web-builder /web/dist /home/ros/web

# Clone third-party robot repos for URDFs/meshes
RUN git clone --depth 1 https://github.com/ROBOTIS-GIT/turtlebot3.git /tmp/turtlebot3 || \
    echo 'TB3 clone failed'
RUN git clone --depth 1 https://github.com/DFKI-NI/mir_robot.git /tmp/mir_robot || \
    echo 'MiR clone failed'
RUN git clone --depth 1 https://github.com/TW-Robotics/Sieglinde_2.0.git /tmp/sieglinde || \
    echo 'Sieglinde clone failed'

# Bake GLBs (each is best-effort; on failure a procedural fallback is rendered)
RUN bash -c 'source /opt/ros/jazzy/setup.bash && python3 /tmp/build_tb4_glb.py' || \
    echo 'TB4 GLB build failed — procedural fallback will be used'
RUN bash -c 'source /opt/ros/jazzy/setup.bash && python3 /tmp/build_tb3_glb.py' || \
    echo 'TB3 GLB build failed — procedural fallback will be used'
RUN bash -c 'source /opt/ros/jazzy/setup.bash && python3 /tmp/build_mir_glb.py' || \
    echo 'MiR GLB build failed — procedural fallback will be used'
RUN bash -c 'source /opt/ros/jazzy/setup.bash && python3 /tmp/build_taurob_glb.py' || \
    echo 'Taurob GLB build failed — procedural fallback will be used'

RUN chmod +x /entrypoint.sh && chown -R www-data:www-data /home/ros/web

EXPOSE 8080 9090
ENTRYPOINT ["/entrypoint.sh"]
