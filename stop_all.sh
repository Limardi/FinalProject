#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Tear down everything start_all.sh brought up: YOLO node, rosbridge, and the
# Unity localization stack. Run from WSL:  ./stop_all.sh
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
COMPOSE="$SCRIPT_DIR/pros_app/docker/compose"

if command -v docker-compose &>/dev/null; then
    DC="docker-compose"
elif docker compose version &>/dev/null; then
    DC="docker compose"
else
    echo "ERROR: Docker Compose is not installed." >&2
    exit 1
fi

echo "==> Stopping YOLO node..."
docker rm -f yolo_node &>/dev/null

echo "==> Bringing down rosbridge + Unity localization stack..."
$DC -f "$COMPOSE/docker-compose_rosbridge_server.yml"   down --timeout 0
$DC -f "$COMPOSE/docker-compose_navigation_unity.yml"   down --timeout 0
$DC -f "$COMPOSE/docker-compose_localization_unity.yml" down --timeout 0
$DC -f "$COMPOSE/docker-compose_robot_unity.yml"        down --timeout 0

echo "==> All down."
