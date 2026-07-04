# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this repo is

A 2D pygame simulator plus a ROS 2 controller for a TurtleBot4 (iRobot Create 3 base) that navigates autonomously using LiDAR obstacle avoidance and YOLO-based sign detection (`left` / `right` / `stop` signs). The project's purpose is "sim-to-real": the same control-loop logic is meant to run unchanged against either a simulated robot or the physical robot, by having both robot classes expose an identical interface.

## Running things

There is no build step — this is a script-based project. `test_*.py` files at the **root** are interactive pygame demos / manual test harnesses, not automated tests. Actual automated tests live in `tests/` (plain scripts with asserts, runnable directly: `python3 tests/test_navigation_smoke.py`; no ROS/pygame required).

```bash
# Draw/edit the 2D map (obstacles + left/right/stop signs + robot start pose), saves to world_map.json
python test_manual_simulation.py --new     # start a blank map in the editor
python test_manual_simulation.py --edit    # load world_map.json into the editor to add to it

# Manual keyboard-controlled simulation (arrow keys drive the robot) against world_map.json
python test_manual_simulation.py

# Full autonomous controller in the simulator (state machine + LiDAR evasion + vision), with
# pause/rewind time-travel debugging (Space to pause, Left/Right arrows to scrub history, L to
# toggle global/robot-local view)
python test_autonomous_controller.py

# Real robot (run from a machine on the robot's ROS_DOMAIN_ID, needs ROS 2 + rclpy + cv_bridge installed)
python run_real_autonomous.py       # full autonomous controller, mirrors test_autonomous_controller.py
python test_controller.py           # simpler 3-zone (red/yellow/green) LiDAR-only avoidance, no vision
python test_vision.py               # vision-only: rotates in place to center on detected signs, never drives forward
```

Dependencies are not pinned in a root requirements file. Simulator scripts need `pygame`, `numpy`, `opencv-python`. `TurtleBotController/turtlebot.py` additionally needs `rclpy`, `geometry_msgs`/`sensor_msgs` (ROS 2 — only available in a ROS 2 environment, e.g. on the robot or its companion computer) and `depthai<3` (vision now runs on the OAK-D's VPU, not via `ultralytics`/`cv_bridge` — see `TurtleBotController/vpu_vision.py` and `DEPLOY_ROBOT_VPU.md`). **Must be the depthai 2.x branch** — 3.x removed `YoloDetectionNetwork`/`XLinkIn`/`XLinkOut` and changed how `Pipeline` binds to a `Device`, so `pip install depthai` alone (which grabs 3.x today) breaks both `vpu_vision.py` and `vpu_deployment/test_depthai_yolo.py`. `yolonano/requirements.txt` lists `ultralytics` + `opencv-python-headless` for the older, now-unused CPU model's standalone vision test (`yolonano/test_webcam.py`); `vpu_deployment/requirements.txt` lists `depthai`/`opencv-python`/`numpy` for the VPU sandbox script.

## Architecture

### The dual-robot abstraction (the core design)

Two classes implement the exact same public interface — `move(v, omega, dt)`, `get_lidar_scan()`, `get_vision_detections()`, `stop()` (mock only exposes `stop` implicitly via direct instantiation), plus attributes `radius`, `lidar_resolution`, `lidar_max_range`, `camera_fov`:

- **`Simulator/TurtleBotSim/turtlebot.py: TurtleBotMock`** — a physics/sensor mock. `move()` integrates a differential-drive kinematic model by hand (no ODE solver), applies a 3-frame command delay and Gaussian actuator noise to mimic hardware latency, and resolves wall collisions by projecting the robot's position onto each obstacle segment and pushing it out of penetration. `get_lidar_scan()` raycasts 360 rays via `Simulator/core/raycasting.py:cast_ray` against `world.obstacles` and adds Gaussian noise. `get_vision_detections()` fakes YOLO by checking which `world.signals` fall inside the camera FOV cone, are within `[camera_min_range, camera_max_range]`, and aren't occluded (cross-checked against the same raycaster).
- **`TurtleBotController/turtlebot.py: TurtleBotReal`** — the real robot. Spins an internal `rclpy` node (`_TurtleBotRosNode`) in a background daemon thread that subscribes to `/scan` (LaserScan) and publishes `Twist`/`TwistStamped` on `/cmd_vel`. `get_lidar_scan()` resamples the raw ROS ranges to `lidar_resolution` (360) and **rotates the array by 90 indices** because the physical sensor's zero angle points to the robot's right, not front — this calibration correction is critical and simulator-side code assumes index 0 = straight ahead. `get_vision_detections()` delegates to `TurtleBotController/vpu_vision.py: VpuYoloDetector`, which runs the `yolonanov2` model (4 classes: `left`/`right`/`stop`/`finish`, as a `.blob`) directly on the OAK-D's onboard VPU (Myriad X) via `depthai` — the `ColorCamera` and `YoloDetectionNetwork` run entirely on-device, so no image ever crosses into ROS or the Raspberry Pi's CPU; only resolved detections (class + confidence + bbox) come back over USB. Distance is still estimated from bounding-box width via the same pinhole-camera approximation as before (assumes ~20cm real-world sign width). See `DEPLOY_ROBOT_VPU.md` for the end-to-end deployment/run guide and `vpu_deployment/` for how the `.blob` was produced and validated standalone.

Because both classes expose the same interface, the navigation logic lives in a single shared module: **`controller/navigation.py: NavigationController`** (state machine + `buscar_camino_libre()`). `run_real_autonomous.py` and `test_autonomous_controller.py` are thin shells that instantiate a robot + a controller and loop `step(lidar_scan, vision_dets, dt) -> (v, w)`. Platform differences are constructor parameters, not code differences: the real robot passes `v_max=0.3` (hardware cap; the sim uses 0.8) and `lidar_min_valid=0.18` (discard chassis self-reflections; the sim passes `None`). All tuning constants (repulsion range, emergency thresholds, turn duration, gains…) are constructor parameters whose defaults equal the historical values. The controller also exposes `snapshot()`/`restore()` (used by the simulator's pause/rewind time-travel), `estado`, `last_signal`, `dist_frente`, and a `debug` dict with the last `buscar_camino_libre` sweep (populated when `collect_debug=True`, for the sim's debug rendering). `tests/test_navigation_smoke.py` exercises the state transitions with synthetic scans — no ROS or pygame needed. See `ROADMAP.md` for the improvement plan and the log of findings (e.g. the numpy front-cone bug fixed during the T1 refactor).

### Navigation state machine

`controller/navigation.py: NavigationController` (used by both `run_real_autonomous.py` and `test_autonomous_controller.py`; the older, simpler `test_controller.py` has its own inline logic) runs a state machine over states like `EXPLORANDO` (cruising), `BUSCANDO_IZQ`/`BUSCANDO_DER` (a sign was seen, looking for room to turn), `GIRANDO_IZQ`/`GIRANDO_DER` (executing a fixed ~80° turn), `DETENIDO` (stopped at a `stop` sign, resumes after 3s), `FINALIZADO` (terminal state entered on a `finish` sign — stays stopped permanently; the real-robot script `break`s out of its main loop, the simulator just freezes the state so pause/rewind still works), and `EVASION_EMERGENCIA` (reactive collision avoidance overriding everything else, triggered when the nearest LiDAR point is inside a safety margin).

`buscar_camino_libre(lidar_points, radio_robot, direccion, margen_extra)` is the shared escape-route search: it sweeps a small fan of candidate angles (front/left/right/any), and for each angle checks a few sample distances ahead for LiDAR points closer than `radio_robot + margen_extra`, returning the first clear angle found. It's used both to decide when it's safe to execute a signed turn and, with a tighter margin, to find an emergency escape heading.

### World / map data

`Simulator/WorldSim/world.py: World` holds `obstacles` (list of line-segment walls), `signals` (list of `{type, x, y}` — type is `left`/`right`/`stop`), and `robot_start` pose, serialized to/from `world_map.json` at the repo root. Maps are authored interactively via the editor mode in `test_manual_simulation.py` (`--new`/`--edit`): click to place wall vertices, `C` to close a polygon, number keys `1`/`2`/`3` + click to drop `left`/`right`/`stop` signs snapped to the nearest wall, `R`/`T` + click to set the robot's start position/orientation, `S`/Enter to save.

### Units and conventions

Distances in meters, angles in radians internally (degrees at some UI/logging boundaries — watch for this when editing). LiDAR is always 360 rays covering 0–360°, index 0 is straight ahead, increasing counter-clockwise. Robot radius is fixed at 0.17m (Create 3 footprint) across both implementations. `TurtleBotController/config.json` holds real-robot-only settings: ROS domain id/topics, `max_linear`/`max_angular` velocity caps, and vision settings (`vpu_blob_path`, `classes_path`, `num_classes`, `confidence_threshold` (0.85), `iou_threshold`, `fps`, `camera_fov_deg`) — the simulator has no equivalent config file, all constants are inline in `TurtleBotSim/turtlebot.py`.

### YOLO model

`yolonano/best.pt` is a trained YOLO nano model detecting the three sign classes (`left`, `right`, `stop`). `yolonano/test_webcam.py` is a standalone smoke test for the model against a local webcam, independent of the robot/simulator code.
