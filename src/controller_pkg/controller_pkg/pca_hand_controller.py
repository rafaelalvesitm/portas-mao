#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Teleoperates the PCA9685 hand by tracking finger curl from a webcam feed
(MediaPipe Hands), instead of listening for an open/closed state on a ROS topic.
Each finger is driven independently based on how bent it is in the video.
"""

import math
import cv2
import mediapipe as mp
import rclpy
from rclpy.node import Node
from adafruit_servokit import ServoKit

CAMERA_INDEX = 0
# 960x540 is the practical ceiling over a usbipd-win passthrough on this setup —
# 1280x720 consistently produced corrupted (black/green) frames, likely from
# USB/IP not sustaining the extra bandwidth for the isochronous webcam transfer
# in time. Raise this if running with a directly-attached (non-passthrough) camera.
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 540
RATE_HZ = 20.0
SMOOTH_ALPHA = 0.4  # 0..1, higher = follows the hand faster but jitters more

# Per-finger calibration: the raw joint angle range that maps to fully
# open (1.0) / fully closed (0.0). The thumb's base->knuckle->tip metric has a
# much narrower natural range than the other fingers (measured ~122-179 deg vs
# ~5-178 deg for the others), so it needs its own bounds — using the same 40/160
# range as the other fingers left the thumb unable to ever read as "closed".
# Retune against your own hand if a finger doesn't reach the servo end-stops.
ANGLE_OPEN_DEG = {
    "thumb": 175.0,
    "index": 160.0,
    "middle": 160.0,
    "ring": 160.0,
    "pinky": 160.0,
}
ANGLE_CLOSED_DEG = {
    "thumb": 125.0,
    "index": 40.0,
    "middle": 40.0,
    "ring": 40.0,
    "pinky": 40.0,
}

# PCA9685 channel wired to each finger (matches the original whole-hand mapping).
FINGER_CHANNELS = {
    "thumb": 4,
    "index": 0,
    "middle": 1,
    "ring": 2,
    "pinky": 3,
}

# True = that channel's servo is mounted backwards (0 deg = closed, 180 deg = open).
FINGER_INVERTED = {
    "thumb": True,
    "index": False,
    "middle": False,
    "ring": False,
    "pinky": False,
}

# MediaPipe Hands landmark ids: (base, mid, tip) used to measure each finger's curl.
FINGER_LANDMARKS = {
    "thumb": (1, 2, 4),
    "index": (5, 6, 8),
    "middle": (9, 10, 12),
    "ring": (13, 14, 16),
    "pinky": (17, 18, 20),
}


def joint_angle_deg(a, b, c) -> float:
    """Angle at point b between segments b->a and b->c, in degrees."""
    v1 = (a.x - b.x, a.y - b.y, a.z - b.z)
    v2 = (c.x - b.x, c.y - b.y, c.z - b.z)
    n1 = math.sqrt(sum(p * p for p in v1))
    n2 = math.sqrt(sum(p * p for p in v2))
    if n1 < 1e-9 or n2 < 1e-9:
        return 180.0  # degenerate vectors: treat as straight/open
    cos_a = sum(p * q for p, q in zip(v1, v2)) / (n1 * n2)
    cos_a = max(-1.0, min(1.0, cos_a))
    return math.degrees(math.acos(cos_a))


def angle_to_t(angle_deg: float, name: str) -> float:
    """Normalizes a raw joint angle to 1.0 (open) .. 0.0 (closed) using that
    finger's own calibration — independent of servo wiring direction."""
    lo, hi = ANGLE_CLOSED_DEG[name], ANGLE_OPEN_DEG[name]
    t = (angle_deg - lo) / (hi - lo)
    return max(0.0, min(1.0, t))


def t_to_servo(t: float, inverted: bool) -> int:
    servo = (1.0 - t) * 180.0  # 0 = open, 180 = closed
    if inverted:
        servo = 180.0 - servo
    return int(round(servo))


class HandservoNode(Node):
    def __init__(self):
        super().__init__('handservo_node')

        self.kit = ServoKit(channels=16)

        self.cap = cv2.VideoCapture(CAMERA_INDEX)
        if not self.cap.isOpened():
            self.get_logger().error(f"Não foi possível abrir a câmera {CAMERA_INDEX}")
        # Força MJPG: YUYV bruto pode chegar corrompido (bloco verde) sob
        # usbipd-win, já que USB/IP não lida bem com transferências isócronas
        # de alta banda. MJPG usa bem menos banda e evita o problema.
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)

        self.hands = mp.solutions.hands.Hands(
            max_num_hands=1,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.6,
        )

        self.last_servo = {name: 0 for name in FINGER_CHANNELS}

        self.create_timer(1.0 / RATE_HZ, self.loop)
        self.get_logger().info("handservo_node: teleoperação por webcam (MediaPipe Hands).")

    def loop(self):
        ok, frame = self.cap.read()
        if not ok:
            return

        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        result = self.hands.process(frame_rgb)
        if not result.multi_hand_landmarks:
            return

        landmarks = result.multi_hand_landmarks[0].landmark

        for name, channel in FINGER_CHANNELS.items():
            base_i, mid_i, tip_i = FINGER_LANDMARKS[name]
            angle = joint_angle_deg(landmarks[base_i], landmarks[mid_i], landmarks[tip_i])
            t = angle_to_t(angle, name)
            target = t_to_servo(t, FINGER_INVERTED[name])

            smoothed = int(round(
                (1.0 - SMOOTH_ALPHA) * self.last_servo[name] + SMOOTH_ALPHA * target
            ))
            self.last_servo[name] = smoothed
            self.kit.servo[channel].angle = smoothed

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        self.hands.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = HandservoNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
