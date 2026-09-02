#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Standalone (non-ROS) test tool: verify webcam finger tracking before wiring it
to real servos. Shows the MediaPipe hand skeleton plus, for each finger, the
measured bend angle and the servo value it would produce with the current
ANGLE_OPEN_DEG/ANGLE_CLOSED_DEG calibration.

These constants and the angle math mirror controller_pkg/pca_hand_controller.py
on purpose — if you retune calibration here, copy the same values over there.

Usage:
    python3 utils/test_webcam_hands.py [--camera 0] [--no-mirror]

Press 'q' or Esc to quit.
"""

import argparse
import math

import cv2
import mediapipe as mp

# ---- Keep these blocks identical to controller_pkg/pca_hand_controller.py ----
# Per-finger calibration: the raw joint angle range that maps to fully
# open (1.0) / fully closed (0.0). The thumb's base->knuckle->tip metric has a
# much narrower natural range than the other fingers (measured ~122-179 deg vs
# ~5-178 deg for the others), so it needs its own bounds — using the same 40/160
# range as the other fingers left the thumb unable to ever read as "closed".
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

FINGER_INVERTED = {
    "thumb": True,
    "index": False,
    "middle": False,
    "ring": False,
    "pinky": False,
}

FINGER_LANDMARKS = {
    "thumb": (1, 2, 4),
    "index": (5, 6, 8),
    "middle": (9, 10, 12),
    "ring": (13, 14, 16),
    "pinky": (17, 18, 20),
}
# ------------------------------------------------------------------------------

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

FINGER_LABELS_PT = {
    "thumb": "Polegar",
    "index": "Indicador",
    "middle": "Medio",
    "ring": "Anelar",
    "pinky": "Mindinho",
}

# Requested capture resolution (actual value depends on what the camera/driver
# supports in MJPG mode; the negotiated size is printed on startup). 960x540 is
# the practical ceiling over a usbipd-win passthrough on this setup — 1280x720
# consistently produced corrupted (black/green) frames, likely from USB/IP not
# sustaining the extra bandwidth for the isochronous webcam transfer in time.
CAMERA_WIDTH = 960
CAMERA_HEIGHT = 540

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.45
FONT_THICKNESS = 1
LINE_HEIGHT = 20


def draw_text_with_bg(frame, text, x, y, color):
    """Draws `text` with its baseline at (x, y), over an opaque black box."""
    (w, h), baseline = cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS)
    cv2.rectangle(frame, (x - 3, y - h - 3), (x + w + 3, y + baseline + 3), (0, 0, 0), cv2.FILLED)
    cv2.putText(frame, text, (x, y), FONT, FONT_SCALE, color, FONT_THICKNESS, cv2.LINE_AA)


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


def main():
    parser = argparse.ArgumentParser(description="Test webcam finger tracking.")
    parser.add_argument("--camera", type=int, default=0, help="Webcam device index.")
    parser.add_argument("--no-mirror", action="store_true", help="Disable selfie-view mirroring.")
    args = parser.parse_args()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Não foi possível abrir a câmera {args.camera}")
        return
    # Força MJPG: YUYV bruto costuma chegar corrompido (bloco verde) sob
    # usbipd-win, já que USB/IP não lida bem com transferências isócronas
    # de alta banda. MJPG usa bem menos banda e evita o problema.
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, CAMERA_WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAMERA_HEIGHT)
    print(f"Resolução negociada: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
          f"{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")

    mp_hands = mp.solutions.hands
    mp_drawing = mp.solutions.drawing_utils
    hands = mp_hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.6,
        min_tracking_confidence=0.6,
    )

    angle_min = {name: math.inf for name in FINGER_ORDER}
    angle_max = {name: -math.inf for name in FINGER_ORDER}

    print("Pressione 'q' ou Esc para sair.")
    print("Abra e feche a mao (e o polegar sozinho) para mapear a faixa de cada dedo.")
    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Falha ao ler frame da câmera.")
                break

            if not args.no_mirror:
                frame = cv2.flip(frame, 1)

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            result = hands.process(frame_rgb)

            y = 20
            if result.multi_hand_landmarks:
                hand_landmarks = result.multi_hand_landmarks[0]
                mp_drawing.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)
                landmarks = hand_landmarks.landmark

                for name in FINGER_ORDER:
                    base_i, mid_i, tip_i = FINGER_LANDMARKS[name]
                    angle = joint_angle_deg(landmarks[base_i], landmarks[mid_i], landmarks[tip_i])
                    angle_min[name] = min(angle_min[name], angle)
                    angle_max[name] = max(angle_max[name], angle)

                    t = angle_to_t(angle, name)
                    servo = t_to_servo(t, FINGER_INVERTED[name])
                    state = "ABERTO" if t >= 0.5 else "FECHADO"
                    label = FINGER_LABELS_PT[name]
                    text = f"{label:10s} ang={angle:5.1f} servo={servo:3d} {state}"
                    draw_text_with_bg(frame, text, 10, y, (0, 255, 0))
                    y += LINE_HEIGHT
            else:
                draw_text_with_bg(frame, "Nenhuma mao detectada", 10, y, (0, 0, 255))

            cv2.imshow("Hand tracking test", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):  # q or Esc
                break
    finally:
        cap.release()
        hands.close()
        cv2.destroyAllWindows()

        print("\nFaixa de angulo bruto observada nesta sessao (graus):")
        print(f"{'dedo':10s} {'min':>7s} {'max':>7s} {'calib.fechado':>14s} {'calib.aberto':>13s}")
        for name in FINGER_ORDER:
            lo, hi = angle_min[name], angle_max[name]
            if lo == math.inf:
                print(f"{FINGER_LABELS_PT[name]:10s} sem dados (mao nao detectada)")
            else:
                print(f"{FINGER_LABELS_PT[name]:10s} {lo:7.1f} {hi:7.1f} "
                      f"{ANGLE_CLOSED_DEG[name]:14.0f} {ANGLE_OPEN_DEG[name]:13.0f}")


if __name__ == "__main__":
    main()
