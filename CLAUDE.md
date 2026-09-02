# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

A ROS 2 (colcon) workspace that drives an InMoov-style humanoid robot's arms and hands from a
motion-capture skeleton feed (ZED body-tracking JSON, "Body34"/"Body38" formats). A separate MQTT
bridge script receives skeleton frames and writes them to a JSON file; ROS nodes read that file,
convert joint quaternions/keypoints into servo angles, and drive Dynamixel + PCA9685 hardware.

## Build & run

```bash
# from the workspace root (this directory)
source /opt/ros/humble/setup.bash   # or the distro actually installed
colcon build
source install/setup.bash
```

Run nodes individually with `ros2 run <package> <executable>`:

```bash
ros2 run communication_pkg processamento_body34      # or processamento_body38
ros2 run controller_pkg hand_controller               # right hand, Dynamixel, /dev/ttyUSB0 id=1
ros2 run controller_pkg left_hand_controller           # left hand, Dynamixel, /dev/ttyUSB1 id=9
ros2 run controller_pkg pca_hand_controller            # right hand fingers via PCA9685
ros2 run controller_pkg joints_pca_control             # both arms' shoulder/elbow via PCA9685
```

Dynamixel controllers accept `--ros-args -p devicename:=... -p baudrate:=... -p dxl_id:=...` to
override serial port/motor id. There is no test suite, linter config, or CI in this repo beyond the
default `ament_lint_auto`/`ament_flake8`/`ament_pep257` test_depends declared in the package
manifests (not currently wired to a runnable command here).

`utils/scan_dynamixel.py --port <dev> --baud <rate>` is a standalone (non-ROS) diagnostic to
broadcast-ping a serial bus and list connected Dynamixel IDs — useful when a hand controller can't
find its motor.

## Data flow / architecture

```
receivejson.py (MQTT subscriber, topic "pingpong/ros")
  -> writes full skeleton frame to recebido.json
       -> communication_pkg processamento node (polls the JSON at 20 Hz)
            -> publishes Int32 angles on /left|right/shoulder_{abd,flex,rot}, /elbow_flex
            -> publishes Bool on /left|right/hand_open
                 -> controller_pkg nodes subscribe and move real hardware
```

`receivejson.py` and `recebido.json` at the repo root are **not** ROS nodes — they're the
MQTT-to-file bridge that's meant to run as a plain Python process on the machine, independent of
`colcon`/`ros2 run`. `recebido.json` is the live/sample data file the processing nodes poll.

### Two interchangeable body-tracking mappers (communication_pkg)

`body34_processamento_node.py` and `body38_processamento_node.py` both consume the same
`recebido.json` shape and publish the exact same topic set, but compute angles differently — they
are alternative implementations, not a pipeline:

- **body34** (`processamento_body34`): uses per-joint orientation quaternions
  (`local_orientation_quat`), decomposing shoulder flex via Euler pitch (needs
  `tf_transformations`) and abduction via rotating a reference vector by the quaternion.
- **body38** (`processamento_body38`): pure geometry from 3D keypoints
  (`keypoints_3d`) — builds a torso reference frame (up/lateral/forward from pelvis/neck/shoulders)
  and derives flex/abduction/elbow/rotation with atan2 on projected vectors (needs `numpy`). Also
  publishes `Float32` `*_raw` debug topics before smoothing/slewing. Its own header comment notes
  flex/abd/elbow are good but rotation is not, and it carries calibrated `ROT_SCALE`/`ROT_OFFSET`
  constants from an offline CSV calibration run — don't treat those as generic defaults.

`save_body34.py` / `save_body38.py` in the same directory are byte-for-byte snapshots of the two
node files above, kept as backups — they are **not** registered as `console_scripts` in `setup.py`
and are not built/installed. When fixing a bug in one of the "live" nodes, the snapshot will drift
out of sync unless updated too (or just left as a historical reference).

Both live nodes hard-code the skeleton JSON path to `/home/atena/fei-atena-tcc/recebido.json` (the
deployment machine's path) rather than reading it relative to the workspace — that will not match
the `recebido.json` at this repo's root on a different machine/checkout. `body38` additionally
falls back to `./recebido.json` (cwd-relative) if the absolute path is missing.

`communication_pkg/setup.py` also registers a third entry point, `processamento_node ->
communication_pkg.processamento_node:main`, but no `processamento_node.py` module exists in the
package — that console script is currently dead/broken if invoked.

### Hardware controllers (controller_pkg)

- `hand_controller.py` / `left_hand_controller.py`: near-identical Dynamixel Protocol-2.0 position
  controllers (right hand on `/right/hand_open`, left on `/left/hand_open`), stepping smoothly
  between an open/closed goal position (`current_position ± TOTAL_MOVEMENT`) rather than absolute
  angles, so behavior depends on the motor's position at startup.
- `pca_hand_controller.py`: drives the right hand's 5 finger servos on a PCA9685
  (`adafruit_servokit`) directly from webcam video — no ROS input topic. It reads frames with
  OpenCV, runs MediaPipe Hands to get 21 landmarks, measures each finger's bend angle (base/mid/tip
  landmark triplet), and maps that continuously to the finger's servo channel — so each finger opens
  and closes independently, following the tracked hand in near-real time. `FINGER_CHANNELS`/
  `FINGER_INVERTED` at the top of the file encode which PCA9685 channel drives which finger and
  which servos are mounted backwards. `ANGLE_OPEN_DEG`/`ANGLE_CLOSED_DEG` are **per-finger** dicts
  (not shared scalars) — the thumb's base/knuckle/tip metric has a much narrower natural range
  (measured ~122-179°) than the other four fingers (~5-178°), so it needs its own calibration bounds;
  giving it the same range as the others left it permanently reading as "closed". `angle_to_t()` /
  `t_to_servo()` deliberately compute the open/closed state before applying `FINGER_INVERTED`, so the
  logical state stays correct regardless of which channels are wired backwards. The camera is opened
  with `CAP_PROP_FOURCC` forced to `MJPG` at `CAMERA_WIDTH`x`CAMERA_HEIGHT` (960x540 here) —
  raw/high-res capture produced corrupted (solid green or black) frames when the webcam reached this
  dev box over a `usbipd-win` USB/IP passthrough, most likely because USB/IP doesn't reliably sustain
  the bandwidth/timing of an isochronous webcam transfer. `utils/test_webcam_hands.py` is a standalone
  (non-ROS) preview of the exact same tracking/calibration logic — showing the MediaPipe skeleton and
  each finger's live angle/servo/state, and printing a per-finger min/max angle summary on exit — for
  validating or retuning `ANGLE_OPEN_DEG`/`ANGLE_CLOSED_DEG` against real hardware before touching
  servos; keep its calibration constants in sync with this file's copy. Needs `mediapipe` and
  `opencv-python` (`cv2`) in addition to `adafruit-circuitpython-servokit`.
- `joints_pca_control.py`: drives both arms' shoulder abd/flex/rot + elbow on 8 PCA9685 channels,
  subscribed to the `Int32` topics the processamento nodes publish. Applies a per-joint mechanical
  `OFFSETS` dict and flushes all 8 channels unconditionally on a 20 Hz timer (not just on message
  receipt).

`src/pca_control_teste.py` (repo root `src/`, not inside either package) is a standalone manual test
node for sweeping all 16 PCA9685 channels from a single `/motor_values` topic — not part of the
colcon build.

### Non-obvious runtime dependencies

Beyond what's declared in `package.xml`, several nodes require pip-installed libraries not managed
by rosdep here: `paho-mqtt` (`receivejson.py`), `tf_transformations` (body34 node — apt package
`ros-<distro>-tf-transformations`), `numpy` (body38 node), `adafruit-circuitpython-servokit` (both
PCA9685 controller nodes and `pca_control_teste.py`), and `mediapipe` + `opencv-python`
(`pca_hand_controller.py`'s webcam tracking, also used standalone by `utils/test_webcam_hands.py`).
