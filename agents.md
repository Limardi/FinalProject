# UNITY Virtual Environment Car — Automatic Control Practical Tutorial

## RELATED RESOURCES

- Unity Virtual Environment: [https://drive.google.com/drive/folders/13juV_QX70JGf63GbHUtsG4nQGBDRMlNY?usp=sharing](https://drive.google.com/drive/folders/13juV_QX70JGf63GbHUtsG4nQGBDRMlNY?usp=sharing)
- VSCode: [https://code.visualstudio.com/download](https://code.visualstudio.com/download)
- Foxglove: [https://foxglove.dev/download](https://foxglove.dev/download)
- Docker: [https://docs.docker.com/desktop/setup/install/windows-install/](https://docs.docker.com/desktop/setup/install/windows-install/)
- WSL Installation Guide: [https://learn.microsoft.com/windows/wsl](https://learn.microsoft.com/windows/wsl)
- Complex environment apps built by others (SLAM, NAV2, etc.): [https://github.com/asd56585452/pros_app](https://github.com/asd56585452/pros_app)
- YOLO: [https://github.com/asd56585452/ros2_yolo_integration](https://github.com/asd56585452/ros2_yolo_integration)
- Custom car control program (arm, wheels, etc.): [https://github.com/asd56585452/pros_car](https://github.com/asd56585452/pros_car)

---

## Tutorial Contents

- Environment check and system tutorial (no code changes needed to complete the practical)
- Introduction to URDF
- Unity Topic Publish information
- FINAL project direction suggestions

---

## Environment Check & System Tutorial

### Environment Setup

#### WSL & UBUNTU 22.04

Open PowerShell on Windows **as Administrator** and run the following command to install Ubuntu 22.04 directly:

```powershell
wsl --install -d Ubuntu-22.04
```

After installation, the system will ask you to create a **Ubuntu username and password** — follow the prompts to complete setup.
Refer to the official Microsoft WSL installation guide for additional configuration.

#### VS CODE

Download and install VSCode from the official website, then open VSCode and install the **Remote Development** extension pack.

#### DOCKER

1. Go to the Docker official website and download **Docker Desktop** for Windows.
2. During installation, make sure **"Use WSL 2 based engine"** is checked.
3. After installation, launch Docker Desktop from Windows.
4. Confirm Docker is **open and running correctly** before proceeding.

#### Environment Setup Steps

Assuming WSL Ubuntu 22.04, VSCode, and Docker are already installed on Windows and Docker is running:

1. Connect VSCode to WSL using the Remote Explorer
2. Navigate to `home/{user}` (the username you set when configuring WSL)
3. Open a Terminal and create the `workspace/pros` folder
4. Clone the repositories:

```bash
mkdir workspace && cd workspace
mkdir pros && cd pros
git clone https://github.com/asd56585452/pros_app
git clone https://github.com/asd56585452/pros_car
git clone https://github.com/asd56585452/ros2_yolo_integration
```

---

## Practical Tutorial — SLAM

### Starting SLAM

Open a new Terminal and enter:

```bash
cd ./workspace/pros/pros_app/
python3 ./control.py -s
# Select the number for slam_unity.sh
```

### UNITY

Launch the Unity virtual environment on Windows and set Car ⇒ Mode to **AI** so that ROS2 controls the car.

### FOXGLOVE

1. Open Foxglove
2. Click **Open connection**
3. Select **Rosbridge**, set URL to `ws://localhost:9090`

After connecting, configure the following:

- Set **Display frame** to `map`
- In the Transform panel, hide all then re-enable: `map`, `base_footprint`, `camera`, `laser`, `arm_ik_base`
- In the Topics panel, enable: `/camera/color/camera_info`, `/camera/image/compressed`, `/map`, `/scan`, `/plan`

### Keyboard Driving to Build the Map

1. Go back to VSCode and open a new Terminal
2. Start pros_car:

```bash
cd ~/workspace/pros/pros_car
./car_control.sh
r
ros2 run pros_car_py robot_control
```

> Wait for the lightning bolt icon ⚡ to appear — this confirms the Docker container is active.

1. After successful startup, you will see the **Main Menu**
2. Select **Control Vehicle**
3. Use `w`, `s`, `e`, `r` to drive the car and build the map (you will see the Foxglove map fill in as you drive)


| Key | Action                    |
| --- | ------------------------- |
| `w` | Forward                   |
| `s` | Reverse                   |
| `e` | Turn Left                 |
| `r` | Turn Right                |
| `z` | Stop                      |
| `c` | Save current camera frame |
| `q` | Quit and return to menu   |


### Localization & Path Planning

1. Return to the terminal running pros_app
2. Press `b` to go back to the script selection menu
3. Enter the number for `./store_map.sh` (saves the scanned map)
4. After saving successfully, press `b` again to return to the menu
5. Press `d` to shut down the previous processes
6. Enter the number for `./localization_unity.sh`

### Starting NAVIGATION

1. Return to Foxglove
2. Right-click on the map to set `/initialpose` and `/goal_pose` — you will see the planned path appear
3. If the topic names are incorrect, fix them in the **Publish** panel at the bottom
4. Return to the pros_car terminal:
  - Select **Auto Navigation**
  - Select **manual_auto_nav**
  - The car will autonomously navigate to the target position

### Important Notes

1. It is normal for the car to collide with the bridge or other obstacles during navigation, because LiDAR can only detect taller obstacles.
2. If no plan (`/plan`) can be found, manually drive the car away from walls and obstacles first — this prevents Nav2 from mistakenly thinking the car is inside a wall.

---

## Practical Tutorial — YOLO

### Image Recognition

1. Keep `localization_unity.sh` running
2. Open a new terminal and enter the ros2_yolo_integration directory:

```bash
cd ~/workspace/pros/ros2_yolo_integration
./yolo_activate.sh
r
ros2 run yolo_example_pkg yolo_node
```

1. Open another new terminal for pros_car:

```bash
cd ~/workspace/pros/pros_car
./car_control.sh
r
ros2 run pros_car_py robot_control
```

1. Select **Auto Navigation** → **manual_auto_nav** and press Enter
  - If values are printing in the terminal and the car is moving autonomously to find the ball, it is working correctly

### Foxglove Image Setup

1. Open Foxglove → Open connection → Rosbridge → `ws://localhost:9090`
2. Click the add panel icon in the top-right corner and add an **Image** panel
3. Click the left panel, change the topic to `/yolo/detection/compressed` and set Calibration to **None**
4. You will now see the YOLO detection result in the image view

### Collecting YOLO Training Data

1. Return to the pros_car Main Menu
2. Select **Control Vehicle**
3. Drive around using `w`, `s`, `e`, `r`
4. Press `c` to save the current camera frame to `pros_car/src/pros_car_py/images`

---

## Practical Tutorial — Automatic Gripper

### Manual Gripper Control

1. Keep `localization_unity.sh` running
2. In Unity, set Arm ⇒ Mode to **AI** so ROS2 controls the arm
3. Start pros_car (same commands as above)
4. Select **Manual Arm Control**
5. Choose which axis to control (0, 1, or 2):


| Key | Action                  |
| --- | ----------------------- |
| `i` | Increase angle          |
| `k` | Decrease angle          |
| `b` | Return to default angle |
| `q` | Return to menu          |


### Automatic Gripper Operation

1. Keep `localization_unity.sh` running
2. Drive the car to a position in front of the bear
3. Switch to **Automatic Arm Mode** ⇒ `auto_arm_human`
4. In Foxglove, use `click_point` to click on the bear's position
5. Return to VSCode and press `g` to automatically grasp the object at the click_point

---

## URDF

### URDF Introduction

**Definition:** URDF (Unified Robot Description Format) is an XML-based language used to describe a robot's physical structure and kinematic model.

**Core elements:**

- **Link:** Describes the rigid body parts of the robot (e.g. chassis, wheels, arm links). Can define Visual, Collision, and Inertial parameters.
- **Joint:** Describes the connection and motion type between two Links (e.g. fixed, revolute, continuous).

### Role of URDF in Our System

- **Building the TF Tree (Transform Tree):** URDF tells the system the relative positions of each part of the robot.
- **Visualization and alignment:** In Foxglove, coordinate frames such as `map`, `base_footprint`, `camera`, `laser`, and `arm_ik_base` are correctly aligned thanks to the URDF structure.
- **Foundation for SLAM and Navigation:** Nav2 and SLAM algorithms must know the precise distance and angle between the LiDAR scan points and the robot's center (`base_footprint`) in order to build maps and avoid obstacles correctly.
- **Automatic arm grasping:** The system needs to know the object's position relative to the arm's position to automatically calculate the arm's grasp angles.

### Current URDF Structure

- **File location:** `pros_app/docker/compose/demo/v6_unity.urdf`
- **Base (chassis):** The system uses `base_footprint` as the origin reference point.
- **Sensors:** The LiDAR (`laser`) and camera (`camera`) are fixed to specific positions on the chassis via Fixed Joints.
- **Arm:** The arm base position `arm_ik_base` is described via a Fixed Joint.

### URDF Workflow in ROS2

1. **Node startup:** When `./localization_unity.sh` or `./slam_unity.sh` is run, the system also starts `docker-compose_robot_unity.yml`.
2. **Robot State Publisher:** This built-in ROS2 node reads the URDF file.
3. **TF broadcast:** After computing all Link positions, it broadcasts them globally on the `/tf_static` topic, making the robot's structure available to all other ROS2 nodes.

### Key URDF Links Summary

#### Sensors

- `camera` — Camera reference frame, derived from `camera_1`.
- `camera_1` — Physical camera model (Link).
- `camera_optical_frame` — Camera optical frame used for image processing (Z-axis pointing forward).

#### Wheels

Four independent wheel Links connected to the chassis via Continuous Joints:

- `new_wheel_v1_1` — Wheel 1
- `new_wheel_v1__1__1` — Wheel 2
- `new_wheel_v1__2__1` — Wheel 3
- `new_wheel_v1__2___1__1` — Wheel 4

#### Arm & Gripper

- `arm_ik_base` — IK (Inverse Kinematics) calculation reference point for the arm, fixed to motor `new_motor_v2_1`.
- `big_U_self_v2_1` — Arm segment 0 (rotation center for axis 0), co-located with `arm_ik_base`.
- `big_U_self_v2__1__1` — Arm segment 1 (rotation center for axis 1).
- `grap2_v2_1` — One side of the gripper (rotation center), connected to `grap_base_v1_1`.
- `grap2_v2_Mirror__1` — The mirrored opposite side of the gripper.

#### Body & Others

- `base_footprint` — The robot's ground projection origin; the root of the entire TF Tree and the reference for navigation and localization.
- `base_link` — The actual chassis center, offset from `base_footprint`.
- `upper_blue_v1_1` / `upper_silver_v1_1` — Blue and silver body panels on top of the chassis.

---

## TOPIC — Unity and ROS2 Topics

### Published (Unity → ROS2)


| Topic                       | Type                          | Description                                                               |
| --------------------------- | ----------------------------- | ------------------------------------------------------------------------- |
| `/scan_tmp`                 | `sensor_msgs/LaserScan`       | LiDAR scan range and intensity data, values clamped to 0.15m–16.0m        |
| `/camera/image/compressed`  | `sensor_msgs/CompressedImage` | RGB camera feed, compressed in JPG format                                 |
| `/camera/image/camera_info` | `sensor_msgs/CameraInfo`      | RGB camera intrinsic matrix (K, D, R, P distortion and projection params) |
| `/camera/depth/compressed`  | `sensor_msgs/CompressedImage` | Depth map; single-channel RFloat data encoded via EXR (ZIP) then Base64   |
| `/camera/depth/camera_info` | `sensor_msgs/CameraInfo`      | Depth camera intrinsic matrix (same as RGB camera)                        |


### Subscribed (ROS2 → Unity)


| Topic                | Type                                   | Description                               |
| -------------------- | -------------------------------------- | ----------------------------------------- |
| `/car_C_front_wheel` | `std_msgs/Float32MultiArray`           | Target RPM for 2 front wheels (2 floats)  |
| `/car_C_rear_wheel`  | `std_msgs/Float32MultiArray`           | Target RPM for 2 rear wheels (2 floats)   |
| `/robot_arm`         | `trajectory_msgs/JointTrajectoryPoint` | Target rotation angles for each arm joint |


### YOLO Topics


| Topic                          | Description                                                                                                                                |
| ------------------------------ | ------------------------------------------------------------------------------------------------------------------------------------------ |
| `/out/compressed`              | Raw camera image                                                                                                                           |
| `/yolo/detection/compressed`   | Processed image with detection bounding boxes                                                                                              |
| `/yolo/target_info`            | Index0 (found): 1 = target detected; Index1 (distance): target depth in meters; Index2 (delta_x): pixel offset of target from image center |
| `/camera/x_multi_depth_values` | Depth values at equally spaced horizontal points (default: 20 points)                                                                      |


> The number of split points can be adjusted via `self.x_num_splits` in `ros2_yolo_integration/src/yolo_example_pkg/yolo_example_pkg/object_detect.py`.

### pros_car Topics

All centralized in `ros_communicator.py` — covers wheel control, arm control, etc. Explore it yourself.

### Foxglove Topics

All active topics on the ROS2 network can be viewed in Foxglove. Use **Raw Messages** to inspect the raw data format of any topic.

---

## FINAL Project Direction Suggestions

### Map Randomization

This year's map is **fully randomized** — the bridge, bears, and road paths will all be randomized each run.

Path randomization uses a combination of **Wave Function Collapse (WFC)** and **A** algorithm:

- A ensures a valid path always exists from the start to the door
- WFC fills in the remaining map tiles by collapsing tiles with the fewest options and propagating constraints

**Current version has 2 maps:**

- **Final Project:** Randomized paths, bridge placement, and bear positions
- **Racing2026:** Fixed roads and bridge position, but bear placements are randomized

---

### Vision & Perception

#### Visual Servoing

- **Concept:** Directly use YOLO-detected object features (e.g. ball center pixel and bounding box area) to compute an error, then output `cmd_vel` (forward speed / turn rate) via a PID controller — bypassing complex global coordinate transforms entirely.
- **Pros:** Straightforward to implement; extremely precise and fast when close to the target (e.g. within 30cm before grasping); not affected by SLAM cumulative drift.
- **Cons:** No global awareness — can get stuck behind obstacles (no avoidance); if the target leaves the camera frame, the system may crash.

#### Semantic Segmentation

- **Concept:** Upgrade from YOLO's rectangular bounding boxes to pixel-level masks (e.g. YOLOv8-Seg).
- **Pros:** Gives the true shape and boundary of objects — very useful for computing object centroids, determining grasp points on irregular shapes, or distinguishing drivable surfaces from walls.
- **Cons:** Higher computational cost; more complex output data structures requiring additional image processing.

#### 3D Bounding Box Detection / 6D Pose Estimation

- **Concept:** Not only detect where an object is, but also compute its 3D rotation (Roll, Pitch, Yaw).
- **Pros:** Critical for arm grasping — lets the system know the object's orientation so it can decide whether to grasp horizontally or vertically.
- **Cons:** Requires depth image data; model training on specific features; higher integration complexity in ROS2.

---

### Navigation & SLAM

#### Visual SLAM

- **Concept:** Replace or supplement 2D LiDAR with an RGB-D depth camera for spatial mapping (e.g. RTAB-Map or ORB-SLAM3).
- **Pros:** Builds colored 3D point cloud maps; better localization in feature-rich environments.
- **Cons:** Extremely CPU/GPU intensive; can fail in low-feature environments (plain white walls) or under heavy lighting changes.

#### Dynamic SLAM

- **Concept:** Combine YOLO or optical flow to filter out moving objects (e.g. people, other vehicles) from LiDAR or camera data while building the map.
- **Pros:** Produces an extremely clean global map without "ghost walls" left by moving objects.
- **Cons:** Longer processing pipeline; requires very high real-time performance to avoid map update delays.

#### 3D Navigation / OctoMap

- **Concept:** Upgrade the 2D Costmap to a 3D octree map (OctoMap), taking height information into account.
- **Pros:** Can avoid overhanging obstacles (e.g. table edges, protruding branches) — especially important for robots with arms that may hit objects the 2D LiDAR cannot see.
- **Cons:** Significantly higher computational load; Nav2 defaults to 2D navigation and may require MoveIt2 for pure 3D path planning.

---

### Control, Decision-Making & System Integration

#### Dynamic Obstacle Avoidance

- **Concept:** Upgrade Nav2's Local Planner using algorithms like **TEB Local Planner** or **MPPI (Model Predictive Path Integral)**.
- **Pros:** Can predict moving obstacle trajectories and navigate around them proactively; results in very smooth driving behavior.
- **Cons:** Parameter tuning is extremely painful; poorly tuned parameters easily cause the "Freezing Robot Problem" (car oscillates in place indefinitely).

#### Advanced Driving Model — Kinematic / Dynamic

- **Concept:** Fully adopt ROS standard `cmd_vel` (linear and angular velocity) instead of directly commanding individual wheel RPMs. Add an EKF (Extended Kalman Filter) at the low level to fuse IMU and odometry data.
- **Pros:** Greatly improves Blind Navigation accuracy (dead-reckoning without LiDAR); ensures smooth, noise-free odometry input to Nav2.
- **Cons:** Requires handling complex math matrices and system noise covariance configuration.

#### Active Exploration / Frontier Exploration

- **Concept:** Use algorithms like `explore_lite` to automatically identify unknown boundary regions (Frontiers) on the map and navigate toward them.
- **Pros:** True full autonomy — from the moment the car is placed, it can simultaneously build the map, run YOLO to detect targets, and navigate to grasp them automatically. Very impressive demo.
- **Cons:** May get stuck in a loop in narrow corners due to blind spots in the path planning algorithm.

---

## Scoring

**Login:** Username `test`, Password `TEST1234`, select **EASY** mode

### Task 1 — 30 pts

- **Locate & Observe (10 pts)** — Must wait **5 seconds!!**
- **Recovery (20 pts)**

### Task 2 — 30 pts

- **Ascent (10 pts)**
- **Descent (10 pts)**
- **Recovery (10 pts)** — Must retrieve the bear on the bridge and return it to the start

### Task 3 — 40 pts

- **Locate & Observe (10 pts)**
- **Unlock (20 pts)**
- **Clear (10 pts)** — Must retrieve the bear near the door

---

## Changelog

### 4/20 Update — Unity Environment Linux Version Released

Added Linux version `pros_twin_unity_linux_V3.zip`. After extracting, run:

```bash
chmod +x pros_twin_unity_tsai_run_linux.x86_64
./pros_twin_unity_tsai_run_linux.x86_64 -force-vulkan
```

**Notes:**

- Navigation programs (pros_app, pros_car, yolo) must run in a Linux environment (Docker), so non-Linux OS users need to use WSL or a VM.
- First-time use requires waiting for Docker images to pull — may take a while on slow networks.
- The Unity environment must run on the native OS (not in a VM) due to Vulkan graphics dependency. On Windows: navigation programs run in WSL, Unity runs on Windows.

### ROS2_YOLO_INTEGRATION — RTX 50-Series GPU Support

If `yolo_activate.sh` throws a CUDA version error, use the new script instead:

```bash
./yolo_activate_cu128.sh
```

This is expected behavior for RTX 50-series GPUs.

### PROS_APP Update

- **SLAM LiDAR position fix:** The previous version of `./localization_unity.sh` published incorrect LiDAR positions, causing localization issues like jitter/oscillation — now fixed.
- `./localization_unity.sh`, `./slam_unity.sh`, and `./rosbridge_server.sh` now include **Foxglove Bridge** connection. You can now use Foxglove WebSocket for display (lower latency) while Unity connects separately via Rosbridge — giving you access to more complete topic data such as wheel RPM.

### 4/27 Update — PROS_APP Update

- Modified Nav2 parameters to make path planning more reliable.

### SLAM Supplementary Notes

- When setting `initialpose`, the **direction must also be correct** in addition to the position and topic name — the blue arrow must point toward the front of the car (the gripper direction).
- The bridge position in the Final Project scene is randomized each time, so SLAM localization may be inaccurate. It is recommended to test with the **Racing2026** scene first (bridge position is fixed).

### V4.1 Scene Update

- Reduced Unity virtual LiDAR publish frequency.
- Racing2026 updated to more closely match the physical competition environment.
- Wheel and gripper ROSBridge reception changed from passive to a buffered queue processed from the main thread.
- Linux and Windows V4 had a scoring bug — update to V4.1 to fix.

