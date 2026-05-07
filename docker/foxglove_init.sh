#!/bin/sh
# Patch Lichtblick's index.html so the default layout is the PRO-LAB layout.
# Runs every container start (idempotent: replaces the placeholder each time).
set -e

LAYOUT_FILE="/layouts/prolab_layout.json"
INDEX="/src/index.html"

if [ -f "$LAYOUT_FILE" ] && [ -f "$INDEX" ]; then
  # Pull the original (or already-patched) head + tail around the
  # LICHTBLICK_SUITE_DEFAULT_LAYOUT assignment, splice in the layout JSON.
  awk -v layout_file="$LAYOUT_FILE" '
    BEGIN {
      while ((getline line < layout_file) > 0) layout = layout line
      close(layout_file)
    }
    {
      gsub(/globalThis\.LICHTBLICK_SUITE_DEFAULT_LAYOUT[[:space:]]*=[[:space:]]*[^;]+;/,
           "globalThis.LICHTBLICK_SUITE_DEFAULT_LAYOUT = " layout ";")
      print
    }
  ' "$INDEX" > "$INDEX.new" && mv "$INDEX.new" "$INDEX"

  # Also serve the layout file directly (fallback for ?layoutUrl=...).
  cp "$LAYOUT_FILE" /src/prolab_layout.json
  echo "[foxglove_init] patched $INDEX with layout from $LAYOUT_FILE"
fi

exec /bin/sh /entrypoint.sh "$@"
