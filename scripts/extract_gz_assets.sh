#!/usr/bin/env bash
# Extract warehouse world + TurtleBot4 models from the container image
# out to the WSL native filesystem so gz sim can load them natively.
set -euo pipefail

IMAGE="${IMAGE:-prolab:jazzy}"
DEST="${DEST:-$HOME/gz_assets}"

mkdir -p "$DEST"

echo "Extracting assets from $IMAGE to $DEST ..."

docker run --rm "$IMAGE" tar -C /opt/ros/jazzy/share -cf - \
  nav2_minimal_tb4_sim \
  nav2_minimal_tb4_description \
  turtlebot4_description \
  irobot_create_description \
  | tar -C "$DEST" -xf -

echo "Done. Contents:"
ls "$DEST"
