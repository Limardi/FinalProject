#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# Clear the AMCL "cache" between runs WITHOUT tearing down rosbridge or YOLO.
#
# AMCL applies its initial_pose (0,0,0) only when the process starts, and its
# recovery is disabled (recovery_alpha_fast/slow = 0 in mapper_params.yaml), so
# a stale/desynced pose never self-corrects. Symptom: the robot used to do the
# task but now drives off to nowhere / can't reach the bridge — because its idea
# of where it is no longer matches Unity.
#
# Run this whenever you reset / replay the Unity scene (car back at its spawn),
# BEFORE you (re)start robot_control:
#
#   1. In Unity: stop + play again so the car is at its start pose.
#   2. ./reset_loc.sh          <-- restarts AMCL clean at origin, aligned.
#   3. restart robot_control in its terminal.
#
# Run from WSL:  ./reset_loc.sh
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

echo "==> Restarting localization stack FRESH (clears stale AMCL pose)..."
$DC -f "$COMPOSE/docker-compose_robot_unity.yml"        up -d --force-recreate
$DC -f "$COMPOSE/docker-compose_localization_unity.yml" up -d --force-recreate
$DC -f "$COMPOSE/docker-compose_navigation_unity.yml"   up -d --force-recreate

echo "==> Done. AMCL re-initialized at (0,0,0). Make sure the Unity car is at"
echo "    its spawn pose, then (re)start robot_control."
echo
echo "    Verify pose is flowing (should print poses near 0,0 once Unity moves):"
echo "        docker exec \$(docker ps --filter ancestor=ghcr.io/screamlab/pros_jetson_driver_image:0.1.0 -q | head -1) \\"
echo "            bash -lc 'source /opt/ros/*/setup.bash; ros2 topic echo /amcl_pose --once'"
