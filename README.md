# SCARA Card Manipulator — ROS2 Stack

> **Educational Use Only**
> This project was developed as a course deliverable for the SAU Engineering Robotics class, 2026.
> It is released under an educational license. Use, reproduction, or adaptation is permitted for
> non-commercial educational and academic purposes only. See [LICENSE](#license) for full terms.

A ROS2 Jazzy–based pick-and-place system built around the **Dobot M1 Pro SCARA robot arm**,
targeting automated Skip-Bo card sorting via classical computer vision. The entire stack runs inside
Docker on Windows 11 + WSL2 and integrates a custom C++ TCP robot driver, OpenCV-based card
recognition, and a full motion coordination node.

---

## Table of Contents

1. [System Overview](#1-system-overview)
2. [Hardware Requirements](#2-hardware-requirements)
3. [Software Prerequisites](#3-software-prerequisites)
4. [Repository Structure](#4-repository-structure)
5. [Installation — Phase 1: Host Setup](#5-installation--phase-1-host-setup)
6. [Installation — Phase 2: Docker Image Build](#6-installation--phase-2-docker-image-build)
7. [Installation — Phase 3: ROS2 Workspace Build](#7-installation--phase-3-ros2-workspace-build)
8. [Camera Setup & Calibration](#8-camera-setup--calibration)
9. [Card Template Capture](#9-card-template-capture)
10. [Running the Full Stack](#10-running-the-full-stack)
11. [Package Reference](#11-package-reference)
12. [ROS2 Topic & Service Reference](#12-ros2-topic--service-reference)
13. [Configuration Reference](#13-configuration-reference)
14. [Troubleshooting](#14-troubleshooting)
15. [Architecture Notes](#15-architecture-notes)
16. [License](#16-license)

---

## 1. System Overview

This stack implements an automated card-sorting pipeline with three operating layers:

**Robot Driver Layer** — A custom C++ ROS2 node (`m1pro_bringup`) communicates with the Dobot M1 Pro
over three raw TCP sockets: dashboard (port 29999), motion commands (port 30003), and a 1440-byte
binary real-time feedback stream (port 30004). It exposes 70 ROS2 services covering all robot
primitives and streams `/joint_states` at a configurable rate.

**Vision Layer** — Two parallel vision pipelines are provided:

- `m1pro_skipbo_vision` — The primary pipeline. A classical OpenCV node segments white card
  rectangles on a dark table surface using HSV thresholding, perspective-rectifies each card to a
  fixed portrait view, classifies color (green/red/purple) via HSV pixel counting, and classifies
  card number (1–12) via multi-scale normalized template matching with glare-robust preprocessing.
  Watershed-based overlap splitting handles partially overlapping cards.

- `m1pro_vision` — A secondary YOLO + OpenVINO inference pipeline using Ultralytics YOLOv8 for
  general-purpose object detection via `vision_msgs/Detection2DArray`. Designed to be swapped in
  when a trained custom YOLO model is available.

**Motion Coordination Layer** — `m1pro_motion` implements a three-state machine (SCANNING →
SORTING → RETURNING_HOME) that consumes vision pick targets, commands the robot to each card
location, actuates the vacuum gripper via ToolDO services, and deposits each card into its
color-coded bin before returning to the scan home position.

```
┌──────────────────────────────────────────────────────────────┐
│                        Docker Container                       │
│                                                              │
│  ┌─────────────────┐   /joint_states    ┌────────────────┐  │
│  │  m1pro_bringup  │◄──────────────────►│  m1pro_moveit  │  │
│  │   (C++ driver)  │   ROS2 services    │  (MoveIt2 cfg) │  │
│  └────────┬────────┘                    └────────────────┘  │
│           │ TCP 29999/30003/30004                            │
│           ▼                                                   │
│    Dobot M1 Pro                                              │
│                                                              │
│  ┌──────────────┐  /camera/image_raw  ┌──────────────────┐  │
│  │ m1pro_camera │────────────────────►│m1pro_skipbo_vision│  │
│  │  (usb_cam)   │                     │  (OpenCV node)   │  │
│  └──────────────┘                     └────────┬─────────┘  │
│                                                │             │
│                              /skipbo/pick_targets            │
│                              /skipbo/pick_target_labels      │
│                                                ▼             │
│                                    ┌────────────────────┐   │
│                                    │    m1pro_motion    │   │
│                                    │  (sorting node)    │   │
│                                    └────────────────────┘   │
└──────────────────────────────────────────────────────────────┘
```

---

## 2. Hardware Requirements

| Component | Specification |
|-----------|--------------|
| Robot | Dobot M1 Pro SCARA — **firmware V1.7.0.6** (see critical note below) |
| End effector | UFactory AS1001 vacuum gripper (custom HD15 → M12 12-pin adapter cable) |
| Camera | eMeet C960 USB webcam (1280×720, 30Hz, MJPEG) |
| Host PC | Windows 11 with WSL2 — Ubuntu 24.04; ≥16 GB RAM recommended |
| Network | Dedicated Ethernet NIC for robot (static IP on `192.168.1.x` subnet) |
| Storage | ≥10 GB free on the drive hosting the workspace |

> ⚠️ **Critical — Firmware Version:** This stack was developed and tested against firmware
> **V1.7.0.6 only.** V2.x and V4.x firmware use an incompatible TCP packet format and a different
> 1440-byte binary feedback structure. The driver **will not connect** on other firmware versions.
> Verify the firmware version in DobotStudio Pro before proceeding. Do not upgrade to V2.x or V4.x.

---

## 3. Software Prerequisites

Install these on your Windows host before starting:

- **Docker Desktop for Windows** — with WSL2 backend enabled
  (Settings → Resources → WSL Integration → enable Ubuntu-24.04)
- **WSL2** with Ubuntu 24.04 (`wsl --install -d Ubuntu-24.04`)
- **DobotStudio Pro** (for firmware verification and initial robot connectivity test)
- **usbipd-win** — required to pass USB camera through to WSL2/Docker
  (`winget install usbipd` in PowerShell as Administrator)

No additional software is required on the host. All ROS2 dependencies are installed inside Docker.

---

## 4. Repository Structure

```
scara_card_manipulator_ros/
├── docker/
│   ├── Dockerfile              # Ubuntu 24.04 + ROS2 Jazzy + OpenVINO + OpenCV contrib
│   ├── docker-compose.yml      # Host networking, volume mounts, device passthrough
│   └── entrypoint.sh           # NumPy ABI check + workspace sourcing at container start
├── src/
│   ├── m1pro_bringup/          # C++ TCP driver — 70 ROS2 services, joint state publisher
│   │   ├── include/            # commander.h (CR5Commander), robot.h, tcp_socket.h
│   │   ├── src/                # main.cpp, robot.cpp, tcp_socket.cpp
│   │   ├── msg/                # RobotStatus.msg, ToolVectorActual.msg
│   │   ├── srv/                # 70 .srv definitions (MovJ, JointMovJ, ToolDO, etc.)
│   │   └── launch/bringup.launch.py
│   ├── m1pro_description/      # URDF robot model + STL meshes
│   │   ├── urdf/m1pro_description.urdf
│   │   └── meshes/             # base_link, Link1–4 STL files
│   ├── m1pro_moveit/           # MoveIt2 config (SRDF, OMPL, KDL, joint limits)
│   │   ├── config/             # kinematics.yaml, ompl_planning.yaml, joint_limits.yaml
│   │   └── launch/move_group.launch.py
│   ├── m1pro_camera/           # usb_cam wrapper + calibration scripts
│   │   ├── config/
│   │   │   ├── camera_params.yaml            # C960 at 1280×720, 30Hz, mjpeg2rgb
│   │   │   ├── aruco_params.yaml             # ArUco marker publisher config
│   │   │   └── calibration/c960_calibration.yaml
│   │   ├── scripts/
│   │   │   ├── calibrate_camera.py           # Affine transform calculation
│   │   │   ├── charuco_camera_to_base_node.py # ChArUco extrinsic calibration node
│   │   │   ├── affine_calibration_gui.py     # GUI-assisted calibration
│   │   │   ├── click_calibration_points.py   # Click-to-calibrate point picker
│   │   │   ├── pixel_to_world_affine.py      # Pixel→world coordinate transform
│   │   │   └── camera_info_service_proxy.py  # CameraInfo service proxy
│   │   └── launch/
│   │       ├── camera.launch.py              # Main camera bringup
│   │       ├── aruco_marker_publisher.launch.py
│   │       └── static_camera_transform.launch.py
│   ├── m1pro_skipbo_vision/    # Classical OpenCV Skip-Bo card detection (PRIMARY)
│   │   ├── m1pro_skipbo_vision/
│   │   │   ├── skipbo_vision_node.py         # Main vision node
│   │   │   ├── template_capture_node.py      # Template image capture utility
│   │   │   └── card_world_publisher.py       # Camera→world coordinate publisher
│   │   ├── config/skipbo_params.yaml
│   │   └── launch/
│   │       ├── skipbo_vision.launch.py
│   │       └── template_capture.launch.py
│   ├── m1pro_vision/           # YOLO + OpenVINO vision node (secondary / general-purpose)
│   │   ├── m1pro_vision/vision_node.py
│   │   ├── config/vision_params.yaml
│   │   └── launch/vision.launch.py
│   └── m1pro_motion/           # Pick-and-place coordinator (state machine)
│       ├── m1pro_motion/dobot_color_sorting_node.py
│       └── launch/
│           ├── color_sorting.launch.py
│           └── test_color_sorter.launch.py
├── config/                     # Calibration output files (populated during setup)
├── models/
│   └── card-templates/templates/  # Number template PNGs (1.png – 12.png)
├── install/                    # Colcon build output (volume-mounted, persisted on host)
└── build/                      # Colcon build artifacts (volume-mounted)
```

---

## 5. Installation — Phase 1: Host Setup

### 5.1 Install WSL2 and Ubuntu 24.04

Open PowerShell **as Administrator**:

```powershell
# Install WSL2 with Ubuntu 24.04
wsl --install -d Ubuntu-24.04
```

Reboot when prompted. After reboot, open Ubuntu 24.04 and create a user account. Then verify:

```bash
# In WSL2
wsl -l -v
```

Expected:
```
  NAME            STATE           VERSION
* Ubuntu-24.04    Running         2
```

### 5.2 Install Docker Desktop

Download and install Docker Desktop for Windows from [docker.com](https://docker.com). After
installation:

1. Open Docker Desktop → Settings → Resources → WSL Integration
2. Enable integration for **Ubuntu-24.04**
3. Apply and Restart Docker Desktop

Verify from WSL2:

```bash
docker --version
docker compose version
```

Expected: Docker version 26.x or later; Docker Compose version v2.x or later.

### 5.3 Install usbipd-win (USB Camera Passthrough)

In PowerShell as Administrator:

```powershell
winget install usbipd
```

### 5.4 Configure the Robot Network Interface

The Dobot M1 Pro communicates over Ethernet at `192.168.1.6`. Configure a static IP on the NIC
connected to the robot:

- **Windows:** Control Panel → Network Adapters → your robot NIC → Properties → IPv4
- Set IP: `192.168.1.100` (any `.1.x` address other than `.6`)
- Subnet: `255.255.255.0`
- Gateway: leave blank

Verify connectivity from WSL2 (robot must be powered on and in Remote Mode):

```bash
ping 192.168.1.6
```

Expected: replies with < 2ms latency.

> 📝 **Note:** The default robot IP in the bringup launch file is `10.12.1.224` (VLAN). Update
> the `robot_ip` argument at launch time to match your network (depending on if connection is via direct Ethernet or over VLAN): `robot_ip:=192.168.1.6`

### 5.5 Create the Workspace

In WSL2:

```bash
# Create workspace on D: drive (or any drive with ≥10 GB free)
mkdir -p /mnt/d/robotics_main/ros2_dobot_ws
cd /mnt/d/robotics_main/ros2_dobot_ws
mkdir -p src docker config models install build
```

### 5.6 Clone This Repository

```bash
cd /mnt/d/robotics_main/ros2_dobot_ws

# Clone the repository
git clone https://github.com/SethRobertson71/scara_card_manipulator_ros.git .
```

The mesh files in `src/m1pro_description/meshes/` are included in the repository. Verify they are
present:

```bash
ls src/m1pro_description/meshes/
# Expected: base_link.STL  Link1.STL  Link2.STL  Link3.STL  Link4.STL
```

### 5.7 Phase 1 Verification

```bash
# ── Phase 1 verification ──────────────────────────────────────
wsl -l -v                                          # Ubuntu-24.04 at VERSION 2
docker --version                                   # Docker 26.x or later
docker compose version                             # v2.x or later
ping 192.168.1.6 -c 3                             # < 2ms replies
ls /mnt/d/robotics_main/ros2_dobot_ws/src/m1pro_bringup/CMakeLists.txt  # file exists
```

---

## 6. Installation — Phase 2: Docker Image Build

> ⚠️ **Critical — DOCKER_BUILDKIT=0:** This must be set before **every** `docker compose build`
> command. BuildKit's network isolation silently blocks `apt` from reaching `packages.ros.org`
> inside WSL2, causing ROS2 packages to appear to install but actually fail. This is the single
> most common setup failure.

### 6.1 Attach the USB Camera to WSL2

Before building (or any time the camera is reconnected), run in PowerShell as Administrator:

```powershell
# List connected USB devices
usbipd list

# Attach the camera (replace <BUSID> with the bus ID shown for your C960)
usbipd attach --wsl --busid <BUSID>
```

The bus ID looks like `1-4` or `2-3`. Find the eMeet C960 in the list. Re-run this command each
time the camera is unplugged or the host reboots.

### 6.2 Build the Docker Image

```bash
# In WSL2, from the workspace root
cd /mnt/d/robotics_main/ros2_dobot_ws

export DOCKER_BUILDKIT=0
docker compose -f docker/docker-compose.yml build --no-cache
```

The build takes 10–20 minutes on first run. It installs:

- Ubuntu 24.04 base system packages (cmake, v4l-utils, build-essential, libopencl, etc.)
- ROS2 Jazzy base + project-specific packages (cv_bridge, usb_cam, image_pipeline, moveit, tf2,
  rmw-cyclonedds-cpp, vision_msgs)
- Python ML stack: OpenVINO (CPU/iGPU inference), NumPy 1.26.x, SciPy, `opencv-contrib-python-headless==4.10.0.84`, Ultralytics YOLOv8 8.3.145, transforms3d, pyserial

> 📝 **Note — OpenCV packaging:** `libopencv-dev` and `python3-opencv` are intentionally **not**
> installed via apt. They ship OpenCV 4.6.x without the contrib module (no ArUco/ChArUco support).
> The Dockerfile installs `opencv-contrib-python-headless==4.10.0.84` via pip exclusively, which
> provides full ArUco/ChArUco calibration capability. Do not add apt OpenCV packages alongside
> this — they conflict on the `cv2` namespace.

### 6.3 Verify the Build

```bash
docker run --rm ros2_dobot:latest ros2 --version
```

Expected:
```
ros2 cli package version jazzy
```

---

## 7. Installation — Phase 3: ROS2 Workspace Build

### 7.1 Start the Container

```bash
cd /mnt/d/robotics_main/ros2_dobot_ws
docker compose -f docker/docker-compose.yml up -d
docker exec -it ros2_dobot bash
```

You are now inside the container at `/ros2_ws`. The `src/`, `install/`, `build/`, `config/`, and
`models/` directories are live-mounted from your host drive — edits made on the host are
immediately visible inside the container without rebuilding the image.

### 7.2 Build All ROS2 Packages

Inside the container:

```bash
# Inside container at /ros2_ws
cd /ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --symlink-install 2>&1 | tee /tmp/build.log
source install/setup.bash
```

Expected output: all packages finish with `[build] Summary: X packages finished`. No `[ERROR]`
lines. The build compiles the C++ driver (`m1pro_bringup`) and installs all Python packages.
Typical build time: 3–8 minutes.

> ⚠️ **Critical — Header-only changes:** After modifying any `.h` file in `m1pro_bringup/include/`,
> perform a clean rebuild of that package:
> ```bash
> rm -rf build/m1pro_bringup install/m1pro_bringup
> colcon build --packages-select m1pro_bringup
> ```
> Colcon's dependency tracking does not reliably detect header-only changes.

### 7.3 Verify the Build

```bash
# ── Phase 3 verification ──────────────────────────────────────
ros2 pkg list | grep m1pro
```

Expected:
```
m1pro_bringup
m1pro_camera
m1pro_description
m1pro_motion
m1pro_moveit
m1pro_skipbo_vision
m1pro_vision
```

### 7.4 Test the Robot Driver

The robot must be powered on and in Remote Mode before launching the driver.

```bash
# Terminal 1 (inside container)
source /ros2_ws/install/setup.bash
ros2 launch m1pro_bringup bringup.launch.py robot_ip:=192.168.1.6
```

Expected console output:
```
[m1pro_driver]: Connecting to Dobot M1 Pro at 192.168.1.6
[m1pro_driver]: CR5Commander initialized — connecting to robot...
[m1pro_driver]: M1Pro driver ready — 20 services registered
[m1pro_driver]: M1Pro driver running at 10.0 Hz
```

Open a second terminal into the container:

```bash
# Terminal 2
docker exec -it ros2_dobot bash
source /ros2_ws/install/setup.bash

# Confirm joint states streaming
ros2 topic echo /joint_states --once

# Enable the robot
ros2 service call /bringup/srv/EnableRobot m1pro_bringup/srv/EnableRobot "{args: []}"

# Test a small safe joint move (values in degrees)
ros2 service call /bringup/srv/JointMovJ m1pro_bringup/srv/JointMovJ \
  "{j1: 5.0, j2: 0.0, j3: 0.0, j4: 0.0, param_value: []}"
```

> ⚠️ **Critical:** `JointMovJ` values are in **degrees**, not radians. Clear the workspace of
> all objects and personnel before any motion command.

---

## 8. Camera Setup & Calibration

### 8.1 Launch the Camera Node

```bash
# Inside container
source /ros2_ws/install/setup.bash
ros2 launch m1pro_camera camera.launch.py
```

Expected output:
```
[usb_cam]: Starting usb_cam node...
[usb_cam]: This device supports: MJPEG
[usb_cam]: Starting streaming at 1280x720 @ 30.00 fps
```

Verify topics:

```bash
ros2 topic list | grep camera
# Expected: /camera/image_raw  /camera/camera_info  /camera/image_rect  /camera/image_rect_cropped

ros2 topic hz /camera/image_raw
# Expected: ~30Hz
```

If `/dev/video0` is not the C960, edit `src/m1pro_camera/config/camera_params.yaml`, change
`video_device` to the correct path (e.g. `/dev/video2`), then:

```bash
colcon build --packages-select m1pro_camera
source install/setup.bash
```

The camera launch also applies forced manual exposure settings for consistent card imaging:

```
auto_exposure=1 (manual mode)
exposure_time_absolute=75
exposure_dynamic_framerate=0
```

### 8.2 Intrinsic Calibration (Checkerboard)

A pre-captured calibration file for the C960 is included at
`src/m1pro_camera/config/calibration/c960_calibration.yaml`. If you need to recalibrate (e.g.
different camera, different focal length setting):

Print the supplied checkerboard target (`checkerboard_9x6_30mm.pdf` — 9×6 inner corners, 30mm
squares). Then inside the container:

```bash
# Camera must be running in a separate terminal first
ros2 run camera_calibration cameracalibrator \
  --size 9x6 \
  --square 0.030 \
  --ros-args -r image:=/camera/image_raw -r camera:=/camera
```

Move the target through the camera field of view in varied poses until the calibration UI reports
enough data, then click **Calibrate** and **Save**. Copy the resulting `ost.yaml` to:

```bash
cp /tmp/calibrationdata/ost.yaml \
   /ros2_ws/src/m1pro_camera/config/calibration/c960_calibration.yaml
colcon build --packages-select m1pro_camera
```

### 8.3 Extrinsic Calibration (Camera-to-Robot-Base)

The ChArUco-based extrinsic calibration node estimates the rigid transform from the camera frame to
the robot base frame.

Print the supplied ChArUco target (`charuco_7x5_35mm.pdf` — 7×5 squares, 35mm square size,
DICT_5X5_50). Place it flat on the robot workspace table.

```bash
# Camera must be running. Then:
source /ros2_ws/install/setup.bash
python3 /ros2_ws/src/m1pro_camera/scripts/charuco_camera_to_base_node.py
```

The node detects the ChArUco board, estimates its pose, inverts to camera→base, and publishes
the static transform on TF (parent: `base_link`, child: `camera_color_optical_frame`).

For an alternative affine pixel-to-robot calibration (fewer prerequisites, manual point
correspondence):

```bash
# Edit pixel_points and robot_points in the script to match your setup
python3 /ros2_ws/src/m1pro_camera/scripts/calibrate_camera.py
```

The static camera transform is published by:

```bash
ros2 launch m1pro_camera static_camera_transform.launch.py
```

---

## 9. Card Template Capture

The `m1pro_skipbo_vision` node uses normalized template matching against reference digit images.
Templates must be captured once and stored at `/ros2_ws/models/card-templates/templates/`.

### 9.1 Create the Template Directory

```bash
mkdir -p /ros2_ws/models/card-templates/templates
```

### 9.2 Capture Templates for Each Card Number

With the camera running, place each card (numbers 1–12) in the camera field of view and run the
template capture node:

```bash
# Capture templates for card number 1 (repeat for 2..12)
source /ros2_ws/install/setup.bash
ros2 launch m1pro_skipbo_vision template_capture.launch.py \
  target_number:=1 \
  required_samples:=4 \
  output_dir:=/ros2_ws/models/card-templates/templates
```

The node automatically captures 4 samples per card and exits. Repeat for `target_number:=2`
through `target_number:=12`. After completion, verify:

```bash
ls /ros2_ws/models/card-templates/templates/
# Expected: 1.png  2.png  3.png  4.png  5.png  6.png  7.png  8.png  9.png  10.png  11.png  12.png
```

> 📝 **Note:** Templates should be captured under the same lighting conditions used during
> operation. The camera's manual exposure is fixed at `exposure_time_absolute=75`; do not
> change ambient lighting between template capture and sorting runs.

---

## 10. Running the Full Stack

### 10.1 Full Launch Order

Each component runs in its own terminal. In every terminal, start by attaching to the container and
sourcing the workspace:

```bash
docker exec -it ros2_dobot bash
source /ros2_ws/install/setup.bash
```

**Terminal 1 — Robot driver (bringup only, no MoveIt2):**

```bash
ros2 launch m1pro_bringup bringup.launch.py robot_ip:=192.168.1.6
```

**Terminal 2 — Camera:**

```bash
ros2 launch m1pro_camera camera.launch.py
```

**Terminal 3 — Skip-Bo Vision:**

```bash
ros2 launch m1pro_skipbo_vision skipbo_vision.launch.py \
  image_topic:=/camera/image_raw \
  template_dir:=/ros2_ws/models/card-templates/templates
```

**Terminal 4 — Static Camera Transform (required for world-frame pose publishing):**

```bash
ros2 launch m1pro_camera static_camera_transform.launch.py
```

**Terminal 5 — Color Sorting Motion Node:**

```bash
ros2 launch m1pro_motion color_sorting.launch.py
```

### 10.2 System Health Check

From any terminal inside the container:

```bash
# 1. Joint states streaming
ros2 topic hz /joint_states
# Expected: ~10Hz

# 2. Camera publishing
ros2 topic hz /camera/image_raw
# Expected: ~30Hz

# 3. Skip-Bo detections publishing
ros2 topic hz /skipbo/detections
# Expected: ~10Hz

# 4. Pick targets being published
ros2 topic echo /skipbo/pick_targets --once

# 5. Annotated vision feed
ros2 topic hz /camera/skipbo_annotated
# Expected: ~10Hz
```

### 10.3 MoveIt2 (Optional — Motion Planning Only)

MoveIt2 is configured but the system uses Dobot onboard kinematics for execution. If you want to
use MoveIt2 motion planning:

```bash
# Replaces Terminal 1 — includes driver + robot_state_publisher + move_group
ros2 launch m1pro_moveit move_group.launch.py robot_ip:=192.168.1.6
```

Wait for:
```
[move_group]: All is well! Everyone is happy! You can start planning now!
```

> ⚠️ **Note:** Do not run both `bringup.launch.py` and `move_group.launch.py` simultaneously —
> move_group.launch.py includes the driver. Running both creates duplicate nodes.

### 10.4 Restarting After Host Reboot

The container restarts automatically (`restart: unless-stopped` in docker-compose.yml). The built
workspace persists via volume mounts. Reattach with:

```bash
docker exec -it ros2_dobot bash
source /ros2_ws/install/setup.bash
```

If source files were edited, rebuild only the changed packages:

```bash
colcon build --symlink-install --packages-select <package_name>
source install/setup.bash
```

---

## 11. Package Reference

### `m1pro_bringup` — Robot Driver

**Type:** C++ `ament_cmake`

**Role:** TCP driver to the Dobot M1 Pro controller. Opens three TCP connections:
- Port 29999: Dashboard commands (enable, disable, clear errors, mode queries)
- Port 30003: Motion commands (MovJ, JointMovJ, ToolDO, SpeedFactor, etc.)
- Port 30004: Real-time 1440-byte binary feedback (joint positions, tool vector, robot status)

**Publishes:**
- `/joint_states` (`sensor_msgs/JointState`) — 4 joints at configurable Hz
- `/m1pro_driver/msg/RobotStatus` (`m1pro_bringup/RobotStatus`) — connection and enable state
- `/m1pro_driver/msg/ToolVectorActual` (`m1pro_bringup/ToolVectorActual`) — TCP position

**Services:** 70 services under `/bringup/srv/` namespace. Key services:

| Service | Type | Description |
|---------|------|-------------|
| `EnableRobot` | `EnableRobot.srv` | Enable motors |
| `DisableRobot` | `DisableRobot.srv` | Disable motors |
| `ClearError` | `ClearError.srv` | Clear robot error state |
| `JointMovJ` | `JointMovJ.srv` | Joint-space move (degrees) |
| `MovJ` | `MovJ.srv` | Cartesian move joint interpolated (mm, degrees) |
| `MovL` | `MovL.srv` | Cartesian move linear interpolated |
| `ToolDO` | `ToolDO.srv` | Set tool digital output (gripper/pump) |
| `SpeedFactor` | `SpeedFactor.srv` | Set global speed ratio (0–100%) |
| `Sync` | `Sync.srv` | Block until all queued motions complete |
| `GetAngle` | `GetAngle.srv` | Query current joint angles |
| `GetPose` | `GetPose.srv` | Query current TCP pose |
| `GetErrorID` | `GetErrorID.srv` | Read active error code |
| `EmergencyStop` | `EmergencyStop.srv` | Immediate halt |

**Joint ordering (firmware → URDF mapping):**
```
Firmware feedback:  [J1=base, J2=elbow, J3=z_slide_mm, J4=eef]
URDF joint order:   [joint1=z_slide_m, joint2=base, joint3=elbow, joint4=eef]
```
The driver applies explicit remapping and mm→m conversion before publishing `/joint_states`.
`JointMovJ` takes degrees; `/joint_states` publishes radians.

**Launch parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `robot_ip` | `10.12.1.224` | Dobot M1 Pro Ethernet IP |
| `JointStatePublishRate` | `10.0` | Joint state publish frequency (Hz) |

---

### `m1pro_description` — URDF Robot Model

**Type:** `ament_cmake`

**Role:** Robot description with correct joint limits derived from the controller's `structure.json`.
References STL meshes from the official Dobot ROS1 repository.

**Joint configuration:**

| Joint | Type | Range | Physical |
|-------|------|-------|----------|
| `joint1` | Prismatic | 0.005–0.245 m | Z-slide (J3) |
| `joint2` | Revolute | ±85° | Base rotation (J1) |
| `joint3` | Revolute | ±135° | Elbow rotation (J2) |
| `joint4` | Revolute | ±360° | EEF rotation (J4) |

**Display launch:**
```bash
ros2 launch m1pro_description display.launch.py
# Opens RViz2 with the URDF visualization
```

---

### `m1pro_moveit` — MoveIt2 Configuration

**Type:** `ament_cmake`

**Role:** MoveIt2 motion planning configuration. Uses Dobot onboard kinematics for execution;
MoveIt2 planning is available as an optional layer.

**Configuration:**

| Parameter | Value |
|-----------|-------|
| Planning group | `m1pro_arm` (chain: `base_link` → `Link4`) |
| Kinematics solver | KDL (`kdl_kinematics_plugin/KDLKinematicsPlugin`) |
| Planning pipeline | OMPL |
| Default planner | RRTConnect |
| Available planners | RRTConnect, RRT, RRTstar, PRM |

---

### `m1pro_camera` — Camera Bringup

**Type:** `ament_cmake` wrapper around `usb_cam`

**Role:** Camera bring-up and calibration tooling for the eMeet C960.

**Camera configuration** (`config/camera_params.yaml`):

| Parameter | Value |
|-----------|-------|
| Device | `/dev/video0` |
| Resolution | 1280×720 |
| Format | `mjpeg2rgb` |
| Frame rate | 30 Hz |
| Exposure | Manual, `exposure_time_absolute=75` |
| Auto white balance | Enabled |
| Calibration | `c960_calibration.yaml` (plumb_bob model) |

**Publishes:**
- `/camera/image_raw` — Raw 1280×720 BGR frames
- `/camera/camera_info` — Calibration parameters
- `/camera/image_rect` — Rectified frames (when `publish_rectified:=true`)
- `/camera/image_rect_cropped` — Cropped rectified frames

**Calibration scripts:**

| Script | Purpose |
|--------|---------|
| `calibrate_camera.py` | Compute affine pixel→robot transform from point correspondences |
| `charuco_camera_to_base_node.py` | ChArUco-based camera→base transform estimation |
| `affine_calibration_gui.py` | GUI for interactive calibration point selection |
| `click_calibration_points.py` | Click-to-collect pixel↔robot point pairs |
| `pixel_to_world_affine.py` | Apply saved affine transform to pixel coordinates |

---

### `m1pro_skipbo_vision` — Skip-Bo Card Detection (Primary Vision)

**Type:** Python `ament_python`

**Role:** Classical computer vision pipeline for Skip-Bo card detection and classification.
No neural network inference — runs efficiently on CPU.

**Detection pipeline:**
1. HSV thresholding to segment white card borders (`S < 45, V > 165`)
2. Morphological open/close to denoise the white mask
3. Contour extraction and filtering by area fraction, aspect ratio (0.55–0.85), and white fill
4. Watershed-based overlap splitting for partially overlapping cards
5. Perspective rectification to a fixed 200×300 px portrait view
6. HSV color classification (green pixels 35–90°H / red 0–15°+170–180°H / purple 125–165°H)
7. Multi-scale normalized template matching against number digit ROIs
8. Color–number consistency scoring with prior weighting

**Publishes:**
- `/skipbo/detections` (`vision_msgs/Detection2DArray`) — Card detections with `color_number` class IDs (e.g. `green_7`)
- `/skipbo/pick_targets` (`geometry_msgs/PoseArray`) — Camera-frame card poses (pixel_x, pixel_y, theta)
- `/skipbo/pick_target_labels` (`std_msgs/String`) — JSON array of labels indexed to pick_targets
- `/skipbo/workspace_poses` (`geometry_msgs/PoseArray`) — TF-transformed poses in robot base frame
- `/camera/skipbo_annotated` (`sensor_msgs/Image`) — Annotated frame with bounding boxes and labels
- `/camera/skipbo_template_roi` (`sensor_msgs/Image`) — Current digit ROI being matched

**Skip-Bo card classes:**
- Numbers: 1–12
- Colors: `green` (numbers 1–4 per card design), `red` (5–8), `purple` (9–12)
- Format: `color_number` (e.g. `green_3`, `red_7`, `purple_11`)

---

### `m1pro_vision` — YOLO Vision Node (Secondary)

**Type:** Python `ament_python`

**Role:** General-purpose YOLO + OpenVINO inference node. Designed for use with a custom-trained
YOLOv8 model when available. Falls back to `yolov8n.pt` (pre-trained COCO) for general-object
detection.

**Publishes:**
- `/detections` (`vision_msgs/Detection2DArray`)
- `/camera/image_annotated` (`sensor_msgs/Image`)

**Key launch parameters:**

| Parameter | Default | Description |
|-----------|---------|-------------|
| `model_path` | `yolov8n.pt` | Path to `.pt` or OpenVINO `.xml` model |
| `device` | `AUTO` | OpenVINO device: AUTO / CPU / GPU |
| `confidence` | `0.5` | Minimum detection confidence |
| `publish_rate` | `10.0` | Max inference rate (Hz) |
| `classes` | `[""]` | Class filter list — empty = all classes |

---

### `m1pro_motion` — Color Sorting Coordinator

**Type:** Python `ament_python`

**Role:** Pick-and-place coordinator. Implements a three-state machine that consumes vision pick
targets and commands the robot through complete sort cycles.

**State machine:**
```
SCANNING ──[picks received]──► SORTING ──[all cards done]──► RETURNING_HOME
   ▲                                                                │
   └────────────────────[at home position]─────────────────────────┘
```

- **SCANNING:** Robot at home (`x=400, y=0, z=240, r=0`). Accepts new pick targets from vision.
- **SORTING:** Processes one card at a time — move to card, pump ON, move to bin, pump OFF + solenoid pulse.
- **RETURNING_HOME:** Returns to home position, then transitions back to SCANNING.

**Services used:**
- `/bringup/srv/ClearError` — Called at startup
- `/bringup/srv/EnableRobot` — Called at startup
- `/bringup/srv/SpeedFactor` — Sets global speed ratio
- `/bringup/srv/MovJ` — Cartesian motion commands
- `/bringup/srv/Sync` — Wait for motion completion
- `/bringup/srv/ToolDO` — Vacuum pump (pin 3) and solenoid (pin 4) control

**Configurable bin positions** (in launch parameters or via ROS params):

| Parameter | Default | Description |
|-----------|---------|-------------|
| `bin_red_pos` | `-18, -303, 130, -94` | Red bin Cartesian pose (mm, degrees) |
| `bin_blue_pos` | `108, -311, 130, -92` | Blue bin Cartesian pose |
| `bin_green_pos` | `234, -319, 130, -91` | Green bin Cartesian pose |
| `home_x/y/z` | `400, 0, 240` | Home scan position (mm) |
| `speed_factor` | `40` | Speed ratio 0–100% |
| `default_pick_z` | `240.0` | Default pick height when vision provides z=0 |

---

## 12. ROS2 Topic & Service Reference

### Topics

| Topic | Type | Publisher | Description |
|-------|------|-----------|-------------|
| `/joint_states` | `sensor_msgs/JointState` | m1pro_bringup | 4 joint positions at 10Hz |
| `/camera/image_raw` | `sensor_msgs/Image` | m1pro_camera | Raw 1280×720 camera frames |
| `/camera/image_rect` | `sensor_msgs/Image` | m1pro_camera | Rectified frames |
| `/camera/camera_info` | `sensor_msgs/CameraInfo` | m1pro_camera | Calibration parameters |
| `/skipbo/detections` | `vision_msgs/Detection2DArray` | m1pro_skipbo_vision | Card detections |
| `/skipbo/pick_targets` | `geometry_msgs/PoseArray` | m1pro_skipbo_vision | Camera-frame pick poses |
| `/skipbo/pick_target_labels` | `std_msgs/String` | m1pro_skipbo_vision | JSON label array |
| `/skipbo/workspace_poses` | `geometry_msgs/PoseArray` | m1pro_skipbo_vision | Robot-frame card poses |
| `/camera/skipbo_annotated` | `sensor_msgs/Image` | m1pro_skipbo_vision | Annotated debug image |
| `/detections` | `vision_msgs/Detection2DArray` | m1pro_vision | YOLO detections |
| `/camera/image_annotated` | `sensor_msgs/Image` | m1pro_vision | YOLO annotated image |

### Key Services (under `/bringup/srv/`)

| Service | Request fields | Notes |
|---------|---------------|-------|
| `EnableRobot` | `args: []` | Must call before motion |
| `DisableRobot` | — | Safe shutdown |
| `ClearError` | — | Must call after any error |
| `JointMovJ` | `j1, j2, j3, j4` (degrees), `param_value` | Joint-space move |
| `MovJ` | `x, y, z, r` (mm/degrees), `param_value` | Cartesian move |
| `MovL` | `x, y, z, r` (mm/degrees), `param_value` | Linear Cartesian move |
| `ToolDO` | `index, status` | Set tool digital output pin |
| `SpeedFactor` | `ratio` (0–100) | Global speed multiplier |
| `Sync` | — | Block until motion queue empty |
| `GetErrorID` | — | Returns active error code |
| `EmergencyStop` | — | Immediate stop |
| `GetAngle` | — | Returns current joint angles |
| `GetPose` | — | Returns current TCP pose |

---

## 13. Configuration Reference

### `skipbo_params.yaml` — Vision Tuning

```yaml
skipbo_vision_node:
  ros__parameters:
    # White border detection (HSV thresholds)
    white_s_max: 45         # Max saturation for white pixels
    white_v_min: 165        # Min value (brightness) for white pixels

    # Card geometry filters
    min_card_area: 5000         # Minimum contour area in pixels
    min_card_area_frac: 0.017   # Min area as fraction of frame
    max_card_area_frac: 0.155   # Max area as fraction of frame
    card_aspect_min: 0.55       # Min short/long side ratio
    card_aspect_max: 0.85       # Max short/long side ratio
    min_white_fill: 0.72        # Min fill ratio of bounding rect

    # Template matching
    template_dir: "/ros2_ws/models/card-templates/templates"
    min_template_score: 0.35    # Minimum match score to accept
    match_upside_down_cards: true
    use_glare_robust_preprocess: false
    use_color_number_prior: true

    # Overlap card splitting
    enable_overlap_split: true
    watershed_dist_peak_ratio: 0.48
```

### `camera_params.yaml` — C960 Camera

```yaml
/**:
  ros__parameters:
    video_device: "/dev/video0"     # Change if camera is at video1/video2
    image_width: 1280
    image_height: 720
    pixel_format: "mjpeg2rgb"
    framerate: 30.0
    autoexposure: false
    exposure: 75                    # Manual exposure value
    camera_info_url: "file:///ros2_ws/src/m1pro_camera/config/calibration/c960_calibration.yaml"
```

### Environment Variables (docker-compose.yml)

| Variable | Default | Description |
|----------|---------|-------------|
| `RMW_IMPLEMENTATION` | `rmw_cyclonedds_cpp` | DDS middleware |
| `ROS_DOMAIN_ID` | `0` | ROS domain for DDS discovery |
| `OPENVINO_DEVICE` | `AUTO` | Inference device: AUTO / CPU / GPU |
| `DISPLAY` | `:0` | X11 display for RViz2 / rqt |

---

## 14. Troubleshooting

| Symptom | Root Cause | Fix |
|---------|-----------|-----|
| `docker compose build` fails — package not found or `/opt/ros` missing | `DOCKER_BUILDKIT=1` (default) blocks apt access to `packages.ros.org` | `export DOCKER_BUILDKIT=0` then rebuild with `--no-cache` |
| `ping 192.168.1.6` fails from WSL2 | Host NIC not configured with a `.1.x` static IP, or robot not powered on / not in Remote Mode | Check Windows Network Adapter settings; verify robot is in Remote Mode via DobotStudio Pro |
| Driver starts but `/joint_states` shows all zeros | Robot is not enabled | Call `EnableRobot` service, or run `GetErrorID` + `ClearError` |
| Driver error: "server disconnected" or connect timeout | Firmware version mismatch or robot not in Remote Mode | Confirm firmware is V1.7.x via DobotStudio Pro — V2.x/V4.x will not work |
| `/camera/image_raw` not publishing | Camera on wrong `/dev/video` path | `ls /dev/video*` inside container; update `video_device` in `camera_params.yaml` |
| Camera not visible at `/dev/video0` in WSL | USB not attached to WSL | Run `usbipd attach --wsl --busid <BUSID>` in PowerShell (Admin) |
| `skipbo_vision_node` reports "Template dir missing" | Templates not captured yet | Run `template_capture.launch.py` for each card number 1–12 |
| Vision publishes detections but pick_targets is empty | Cards not meeting geometry filters | Adjust lighting; check `white_s_max`, `white_v_min`; ensure dark table surface |
| YOLO model fails to load | Container has no internet access for auto-download | Pre-download: `python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"` inside container, or copy `.pt` to `/ros2_ws/models/` |
| `colcon build` error: numpy conflict | pip numpy conflicts with apt residue | Rebuild Docker image — Dockerfile removes apt numpy dist-info before pip install |
| `ros2 pkg list` does not show `m1pro_skipbo_vision` after build | ament_python prefix path hook not generated | Check that `environment/ament_prefix_path.dsv` and `.sh` files exist in the package source; rebuild and re-source |
| MoveIt2 planning always fails | Joint limits too tight for goal, or robot not enabled | Confirm requested pose is within URDF limits; verify `EnableRobot` was called |
| Sorting node stuck in SORTING state | MovJ service timeout or robot in error state | Check bringup terminal for error messages; call `ClearError` then `EnableRobot` |

### T-1: Clean Rebuild After Header Changes

```bash
# Inside container — after editing any .h file in m1pro_bringup/include/
rm -rf /ros2_ws/build/m1pro_bringup /ros2_ws/install/m1pro_bringup
cd /ros2_ws
colcon build --packages-select m1pro_bringup
source install/setup.bash
```

### T-2: Rebuilding the Docker Image

```bash
# In WSL2
cd /mnt/d/robotics_main/ros2_dobot_ws
docker compose -f docker/docker-compose.yml down
export DOCKER_BUILDKIT=0
docker compose -f docker/docker-compose.yml build --no-cache
docker compose -f docker/docker-compose.yml up -d
```

### T-3: NumPy ABI Mismatch at Runtime

The container entrypoint checks NumPy version at every startup and downgrades to `1.26.x` if
`2.x` is detected. If you see NumPy-related import errors:

```bash
# Inside container
pip3 install --no-cache-dir --break-system-packages "numpy>=1.26,<2.0"
```

---

## 15. Architecture Notes

### Why Firmware V1.7.x Only

The TCP real-time feedback stream on port 30004 uses a fixed 1440-byte binary packet format
(`RealTimeData` struct, packed). V2.x and V4.x firmware use an incompatible packet format and a
different protocol negotiation sequence. DobotStudio Pro 4.6 requires V4.x protocol, making direct
TCP/ROS2 integration the only viable path with V1.7.x firmware. Do not update firmware without
validating driver compatibility.

### network_mode: host

`network_mode: host` is required in `docker-compose.yml`. Without it the container cannot reach
`192.168.1.6` over the physical Ethernet NIC (Docker's bridge network does not forward raw
Ethernet), and CycloneDDS multicast discovery fails across machines on the same domain.

### OpenCV Packaging

The apt packages `libopencv-dev` and `python3-opencv` are intentionally excluded from the
Dockerfile. They ship OpenCV 4.6.x without the `contrib` module, which is required for ArUco and
ChArUco board support used in extrinsic calibration. `opencv-contrib-python-headless==4.10.0.84`
is installed via pip and takes full ownership of the `cv2` namespace. Do not install both — they
conflict.

### Joint Remapping

Firmware sends joint feedback as `[J1=base, J2=elbow, J3=z_slide_mm, J4=eef]`. URDF joints are
ordered `[joint1=z_slide_m, joint2=base, joint3=elbow, joint4=eef]`. The driver applies explicit
index remapping and mm→m conversion before publishing `/joint_states`. This ordering is fixed by
the original URDF export and must be preserved throughout the stack.

### Workspace Persistence

`install/` and `build/` are volume-mounted from the host to the container. Never run
`colcon build --merge-install` — it produces a non-persistent layout incompatible with this
volume mount strategy.

---

## 16. License

**Educational Use License**

Copyright (c) 2026 Seth Robertson — SAU Engineering Robotics, 2026

This software and associated documentation files (the "Software") are made available for
**educational and academic use only**. Use, reproduction, modification, and distribution are
permitted exclusively for non-commercial educational purposes, including coursework, academic
research, and educational demonstrations.

**Restrictions:**
- Commercial use of this Software, in whole or in part, is expressly prohibited.
- Redistribution for commercial purposes is expressly prohibited.
- Any educational use must retain this copyright notice and license statement.
- Academic publications or coursework referencing this Software should cite the original repository:
  `github.com/SethRobertson71/scara_card_manipulator_ros`

**Disclaimer:**
THE SOFTWARE IS PROVIDED "AS IS" FOR EDUCATIONAL PURPOSES, WITHOUT WARRANTY OF ANY KIND.
THE AUTHORS ARE NOT LIABLE FOR ANY DAMAGES ARISING FROM USE OF THE SOFTWARE, INCLUDING
DAMAGE TO HARDWARE OR PERSONAL INJURY. ALWAYS ENSURE THE ROBOT WORKSPACE IS CLEAR OF
PERSONS AND OBJECTS BEFORE COMMANDING ANY ROBOT MOTION.

---

*Project: SAU Engineering Robotics — SCARA Card Manipulator, 2026*
*Repository: [github.com/SethRobertson71/scara_card_manipulator_ros](https://github.com/SethRobertson71/scara_card_manipulator_ros)*
