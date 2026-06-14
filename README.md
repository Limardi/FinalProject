# Final Project — Unity ROS2 Robotics Stack

Monorepo containing the full competition stack:

| Folder | Purpose |
|--------|---------|
| `pros_app/` | SLAM, localization, Nav2, Docker compose |
| `pros_car/` | Car control, auto task, arm control |
| `ros2_yolo_integration/` | YOLO object detection |

## Quick start

See **[agents.md](agents.md)** for the full tutorial (WSL, Docker, Unity, Foxglove).

```bash
# From WSL, in this folder:
./start_all.sh          # localization + rosbridge + YOLO
./stop_all.sh           # tear down
./reset_loc.sh          # reset AMCL only
```

**Unity simulator** is not included (too large for GitHub). Download from the link in `agents.md` and extract next to this repo as `pros_twin_unity_tsai_run/`.

## Layout

- Unity runs on **Windows** (native, not WSL)
- ROS2 / Docker runs in **WSL Ubuntu 22.04**
- Rosbridge: `ws://localhost:9090`
