#!/usr/bin/env bash
# Run Gazebo Harmonic natively in WSL2 with the nav2 warehouse world.
# Assets (world + robot models) must already be extracted to ~/gz_assets.
# See: scripts/extract_gz_assets.sh
set -euo pipefail

ASSETS="${GZ_ASSETS:-$HOME/gz_assets}"

if [ ! -d "$ASSETS/nav2_minimal_tb4_sim" ]; then
  echo "Missing $ASSETS/nav2_minimal_tb4_sim — run scripts/extract_gz_assets.sh first." >&2
  exit 1
fi

export GZ_PARTITION=prolab
export GZ_IP=127.0.0.1

# WSL2 GPU: route OpenGL through Mesa's D3D12 backend and pick NVIDIA adapter.
# Default in WSLg is llvmpipe (CPU). Without this, Gazebo renders on CPU.
export GALLIUM_DRIVER=d3d12
export MESA_D3D12_DEFAULT_ADAPTER_NAME=NVIDIA
export LIBGL_ALWAYS_SOFTWARE=0
export GZ_SIM_RESOURCE_PATH="$ASSETS:\
$ASSETS/nav2_minimal_tb4_sim/models:\
$ASSETS/nav2_minimal_tb4_sim/worlds:\
$ASSETS/nav2_minimal_tb4_description/meshes:\
$ASSETS/nav2_minimal_tb4_description/urdf:\
$ASSETS/turtlebot4_description:\
$ASSETS/irobot_create_description"

WORLD="${1:-$ASSETS/nav2_minimal_tb4_sim/worlds/warehouse.sdf}"

echo "Launching gz sim with:"
echo "  WORLD=$WORLD"
echo "  GZ_PARTITION=$GZ_PARTITION"
echo "  GZ_SIM_RESOURCE_PATH=$GZ_SIM_RESOURCE_PATH"

HEADLESS="${HEADLESS:-1}"
if [ "$HEADLESS" = "1" ]; then
  echo "  HEADLESS=1 (server only — set HEADLESS=0 to also start the GUI)"
  exec gz sim -s -r -v 4 "$WORLD"
else
  exec gz sim -r -v 4 "$WORLD"
fi
