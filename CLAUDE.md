# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## What this is

Teleoperates one InMoov-style robotic hand's 5 finger servos from a webcam feed: `hand_teleop.py`
runs MediaPipe Hands on the video, measures each finger's bend angle, and sends the resulting
servo angles over USB serial to an ESP32 (MicroPython), which writes them to a PCA9685 board over
I2C. No tracking/calibration logic runs on the ESP32 — it's a dumb angle-to-PWM relay. Full setup,
wiring, and step-by-step testing instructions are in `README.md`; this file covers architecture
and gotchas that aren't obvious from reading a single file.

Deliberately not a ROS 2 workspace — there is no `src/`, no `colcon`, no `rclpy` anywhere in this
repo. It was trimmed down from a larger InMoov arms+hands ROS 2 project (ZED skeleton tracking,
MQTT bridge, Dynamixel-driven hands, a `pca_hand_controller.py` ROS 2 node) to just this one
script; that history — and any of the removed pieces, if ever needed again — is in `git log`.

## Build & run

No build step, no test suite, no linter config. Everything is plain Python:

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python opencv-python "mediapipe==0.10.14" pyserial
uv run hand_teleop.py --port /dev/ttyUSB0
```

`utils/test_webcam_hands.py` (webcam+MediaPipe only) and `utils/test_esp32_hand.py`
(ESP32+PCA9685 only, over serial) isolate each half of `hand_teleop.py`;
`esp32/test_pca9685_standalone.py` isolates the PCA9685/servo hardware alone, running entirely on
the ESP32 via Thonny with no PC involved. README.md's "Testando, em ordem" section explains why to
use each one and in what order — don't skip straight to `hand_teleop.py` when debugging something.

## Architecture

```
webcam --(cv2 + MediaPipe Hands)--> per-finger angle --(calibration + smoothing)--> servo value
    --(USB serial, one CSV-ish line per tick)--> ESP32 --(I2C)--> PCA9685 --> 5 servos
```

### Two files share the same constants and math — keep them in sync

`hand_teleop.py` (the real program) and `utils/test_webcam_hands.py` (webcam-only preview) each
independently define `ANGLE_OPEN_DEG`, `ANGLE_CLOSED_DEG`, `FINGER_CHANNELS`, `FINGER_INVERTED`,
`FINGER_LANDMARKS`, and the `joint_angle_deg()` / `angle_to_t()` / `t_to_servo()` functions. This
is deliberate duplication (no shared module between them), not drift — if you retune calibration
or fix a bug in one, copy it to the other.

- `ANGLE_OPEN_DEG`/`ANGLE_CLOSED_DEG` are **per-finger** dicts, not shared scalars — the thumb's
  base/knuckle/tip landmark triplet has a much narrower natural range (~125-175°) than the other
  four fingers (~40-160°); giving it the same range as the others left it permanently reading as
  "closed" no matter what the hand did.
- `angle_to_t()` computes a wiring-independent open/closed state (`t`, 1.0=open, 0.0=closed)
  *before* `t_to_servo()` applies `FINGER_INVERTED`. Computing the label from the post-inversion
  servo value instead (an earlier bug) silently flips it for any inverted channel.
- `FINGER_CHANNELS` here is also the **entire** contract with `esp32/hand_pca9685_server.py` — the
  ESP32 has no other way to know which PCA9685 channel is which finger, so if this mapping
  changes on the PC side, mirror it in the ESP32 file's own copy too (three files in total for
  this one mapping).

### ESP32 side (`esp32/`)

`esp32/hand_pca9685_server.py` is MicroPython, edited/run via Thonny — there's no build/packaging
step for it, it's just a file you copy onto the device. It reads newline-terminated
`thumb=90,index=45,...` lines from **stdin over the same USB/UART cable Thonny's REPL uses**
(`select.poll()` on `sys.stdin`) and writes each to the matching PCA9685 channel via bit-banged
I2C register writes (no external driver library needed on the device). Because it shares the
console UART, **Thonny and `hand_teleop.py` can never hold the port open at the same time**.

WiFi/HTTP was the original design but was abandoned: with the ESP32 on a Windows Mobile Hotspot,
WSL2 couldn't reach that subnet even after enabling `networkingMode=mirrored` in `.wslconfig` —
the hotspot's virtual adapter isn't one of the interfaces WSL2 mirrors. Serial sidesteps
networking entirely, at the cost of needing `usbipd attach --wsl` for the ESP32's serial device
too (same as the webcam).

### Known hardware gotcha: PCA9685 `OE` pin

Generic/clone PCA9685 breakouts (this project used an "HW-170" board) don't tie the `OE`
(Output Enable) pin to GND internally by default, unlike genuine Adafruit boards. With `OE`
floating, the chip accepts I2C writes and its PWM registers read back exactly as written (so
`esp32/test_pca9685_standalone.py`'s register-readback check passes), but the physical output
pins stay disabled — no signal reaches the servos, and current draw doesn't change at all. If
servos don't move despite everything checking out in software, check for a jumper wire from `OE`
to any `GND` on the board before suspecting the code.

### WSL2-specific dev-box notes

The camera is opened with `CAP_PROP_FOURCC` forced to `MJPG` at 960x540
(`CAMERA_WIDTH`/`CAMERA_HEIGHT`) — raw/high-res capture over the `usbipd-win` passthrough on this
box produced corrupted (solid green or black) frames, most likely because USB/IP doesn't reliably
sustain the bandwidth/timing of an isochronous webcam transfer; 1280x720 reproduced the corruption
consistently, 960x540 didn't. This constant may not need to exist at all on a Linux box with the
webcam plugged in directly.

## Non-obvious runtime dependencies

Nothing here is managed by a lockfile/requirements.txt — `mediapipe` (pinned to `0.10.14` — newer
releases dropped the `mp.solutions.hands` API this code uses, in favor of a Tasks API that needs a
separately downloaded model file), `opencv-python`, and `pyserial` all need to be pip-installed
into whatever environment runs these scripts (see README.md's `uv venv` setup).
