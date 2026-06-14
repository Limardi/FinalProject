#!/bin/bash
# ─────────────────────────────────────────────────────────────────────────────
# One-shot launcher for the Unity robotics stack.
#
# Brings up — all DETACHED, so they keep running while you restart the
# robot_control node as many times as you want:
#   1. Unity localization stack (robot_unity + localization_unity + navigation_unity)
#   2. rosbridge server (port 9090 — what Unity connects to)
#   3. YOLO detection node (yolo_example_pkg yolo_node)
#
# It does NOT open the robot_control container — run that yourself in a separate
# terminal (see the printout at the end). Tear everything down with ./stop_all.sh
#
# Run from WSL:  ./start_all.sh
# ─────────────────────────────────────────────────────────────────────────────

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROS_APP="$SCRIPT_DIR/pros_app"
YOLO_DIR="$SCRIPT_DIR/ros2_yolo_integration"
COMPOSE="$PROS_APP/docker/compose"

# ── Pick the docker compose command ──────────────────────────────────────────
if command -v docker-compose &>/dev/null; then
    DC="docker-compose"
elif docker compose version &>/dev/null; then
    DC="docker compose"
else
    echo "ERROR: Docker Compose is not installed." >&2
    exit 1
fi

# ── 1 + 2: infra via docker compose (naturally detached, stays up) ───────────
# IMPORTANT: --force-recreate on the localization trio. AMCL only applies its
# initial_pose (0,0,0) when the PROCESS starts, and its recovery is disabled
# (recovery_alpha_fast/slow = 0 in mapper_params.yaml) so it can never re-home a
# stale estimate on its own. If we leave AMCL up across runs, its pose drifts /
# desyncs from Unity's spawn and navigation silently breaks ("can't do task2
# anymore"). Force-recreating guarantees a clean origin pose every launch.
echo "==> Bringing up Unity localization stack (FRESH — clears stale AMCL pose)..."
$DC -f "$COMPOSE/docker-compose_robot_unity.yml"        up -d --force-recreate
$DC -f "$COMPOSE/docker-compose_localization_unity.yml" up -d --force-recreate
$DC -f "$COMPOSE/docker-compose_navigation_unity.yml"   up -d --force-recreate

echo "==> Bringing up rosbridge server (detached)..."
$DC -f "$COMPOSE/docker-compose_rosbridge_server.yml"   up -d

# ── 3: YOLO node ─────────────────────────────────────────────────────────────
# yolo_activate.sh normally opens an interactive shell where you build + run by
# hand. Here we do the same steps non-interactively in a detached container.
# Uses yolo_pkg (our package). Menu is auto-answered "1" (detect+publish, no
# screenshots). The node now publishes /yolo/target_info so auto_task can see
# detections.
echo "==> Starting YOLO node (detached container 'yolo_node')..."
docker rm -f yolo_node &>/dev/null

YOLO_IMAGE="registry.screamtrumpet.csie.ncku.edu.tw/screamlab/pros_cameraapi:0.0.2"
YOLO_CMD='cd /workspaces \
  && source /opt/ros/*/setup.bash \
  && colcon build \
  && source ./install/setup.bash \
  && echo 1 | ros2 run yolo_pkg yolo_detection_node'

run_yolo() {
    docker run -d --name yolo_node \
        --network compose_my_bridge_network \
        -e PYTHONUNBUFFERED=1 \
        -e YOLO_FORCE_CPU="${YOLO_FORCE_CPU:-1}" \
        "$@" \
        --env-file "$YOLO_DIR/.env" \
        -v "$YOLO_DIR/src:/workspaces/src" \
        -v "$YOLO_DIR/screenshots:/workspaces/screenshots" \
        -v "$YOLO_DIR/fps_screenshots:/workspaces/fps_screenshots" \
        "$YOLO_IMAGE" \
        bash -lc "$YOLO_CMD"
}

# Default to CPU because detached CUDA startup can hang while loading the model.
# To try GPU explicitly: YOLO_FORCE_CPU=0 ./start_all.sh
if [ "${YOLO_FORCE_CPU:-1}" = "1" ]; then
    run_yolo
elif ! run_yolo --gpus all; then
    echo "    GPU launch failed - retrying YOLO without GPU..."
    docker rm -f yolo_node &>/dev/null
    YOLO_FORCE_CPU=1 run_yolo
fi

# ── Done ─────────────────────────────────────────────────────────────────────
cat <<EOF

==> Infra is up (detached). Nothing here will die when you restart the node.

    Watch YOLO build/run:   docker logs -f yolo_node
    Watch localization:     docker logs -f \$(docker ps --filter name=localization -q)

    Next, in a SEPARATE terminal, start the robot_control node:
        cd "$SCRIPT_DIR/pros_car" && ./car_control.sh
        # then inside the container:
        colcon build --packages-select pros_car_py
        source install/setup.bash
        ros2 run pros_car_py robot_control

    Stop everything:        "$SCRIPT_DIR/stop_all.sh"
EOF
