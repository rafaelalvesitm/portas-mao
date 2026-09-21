#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Main program: webcam -> MediaPipe -> per-finger calibration/smoothing ->
serial -> ESP32 -> real servos. Shows the MediaPipe skeleton and each finger's
live angle/servo value on screen while it drives the actual hardware.

Keep ANGLE_OPEN_DEG / ANGLE_CLOSED_DEG / FINGER_CHANNELS / FINGER_INVERTED /
SMOOTH_ALPHA in sync with utils/test_webcam_hands.py if you retune any of
them — see README.md.

Usage:
    python3 hand_teleop.py --port /dev/ttyUSB0
    python3 hand_teleop.py --port /dev/ttyUSB0 --camera 0 --no-mirror

Press 'q' or Esc to quit.
"""

import argparse
import math

import cv2
import mediapipe as mp
import serial

# ---- Keep these blocks identical to utils/test_webcam_hands.py ----
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

FINGER_CHANNELS = {
    "thumb": 4,
    "index": 0,
    "middle": 1,
    "ring": 2,
    "pinky": 3,
}

FINGER_INVERTED = {
    "thumb": False,
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

SMOOTH_ALPHA = 0.4
# ------------------------------------------------------------------------------

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]

FINGER_LABELS_PT = {
    "thumb": "Polegar",
    "index": "Indicador",
    "middle": "Medio",
    "ring": "Anelar",
    "pinky": "Mindinho",
}

CAMERA_WIDTH = 960
CAMERA_HEIGHT = 540
SERIAL_BAUDRATE = 115200

FONT = cv2.FONT_HERSHEY_SIMPLEX
FONT_SCALE = 0.45
FONT_THICKNESS = 1
LINE_HEIGHT = 20


def draw_text_with_bg(frame, text, x, y, color):
    (w, h), baseline = cv2.getTextSize(text, FONT, FONT_SCALE, FONT_THICKNESS)
    cv2.rectangle(frame, (x - 3, y - h - 3), (x + w + 3, y + baseline + 3), (0, 0, 0), cv2.FILLED)
    cv2.putText(frame, text, (x, y), FONT, FONT_SCALE, color, FONT_THICKNESS, cv2.LINE_AA)


def joint_angle_deg(a, b, c) -> float:
    v1 = (a.x - b.x, a.y - b.y, a.z - b.z)
    v2 = (c.x - b.x, c.y - b.y, c.z - b.z)
    n1 = math.sqrt(sum(p * p for p in v1))
    n2 = math.sqrt(sum(p * p for p in v2))
    if n1 < 1e-9 or n2 < 1e-9:
        return 180.0
    cos_a = sum(p * q for p, q in zip(v1, v2)) / (n1 * n2)
    cos_a = max(-1.0, min(1.0, cos_a))
    return math.degrees(math.acos(cos_a))


def angle_to_t(angle_deg: float, name: str) -> float:
    lo, hi = ANGLE_CLOSED_DEG[name], ANGLE_OPEN_DEG[name]
    t = (angle_deg - lo) / (hi - lo)
    return max(0.0, min(1.0, t))


def t_to_servo(t: float, inverted: bool) -> int:
    servo = (1.0 - t) * 180.0
    if inverted:
        servo = 180.0 - servo
    return int(round(servo))


def main():
    parser = argparse.ArgumentParser(description="Webcam-to-ESP32 hand teleoperation.")
    parser.add_argument("--port", required=True, help="ESP32 serial port, e.g. /dev/ttyUSB0.")
    parser.add_argument("--camera", type=int, default=0, help="Webcam device index.")
    parser.add_argument("--no-mirror", action="store_true", help="Disable selfie-view mirroring.")
    args = parser.parse_args()

    esp32 = serial.Serial(args.port, SERIAL_BAUDRATE, timeout=0)
    print(f"ESP32 conectada em {args.port}.")

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        print(f"Não foi possível abrir a câmera {args.camera}")
        esp32.close()
        return
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

    last_servo = {name: 0 for name in FINGER_ORDER}

    print("Pressione 'q' ou Esc para sair.")
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
                    t = angle_to_t(angle, name)
                    target = t_to_servo(t, FINGER_INVERTED[name])
                    last_servo[name] = int(round(
                        (1.0 - SMOOTH_ALPHA) * last_servo[name] + SMOOTH_ALPHA * target
                    ))

                    state = "ABERTO" if t >= 0.5 else "FECHADO"
                    label = FINGER_LABELS_PT[name]
                    text = f"{label:10s} ang={angle:5.1f} servo={last_servo[name]:3d} {state}"
                    draw_text_with_bg(frame, text, 10, y, (0, 255, 0))
                    y += LINE_HEIGHT

                line = ",".join(f"{name}={last_servo[name]}" for name in FINGER_ORDER)
                try:
                    esp32.write((line + "\n").encode())
                    esp32.reset_input_buffer()
                except Exception as e:
                    draw_text_with_bg(frame, f"Erro serial: {e}", 10, y, (0, 0, 255))
            else:
                draw_text_with_bg(frame, "Nenhuma mao detectada", 10, y, (0, 0, 255))

            cv2.imshow("Teleoperacao da mao (webcam -> ESP32)", frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
    finally:
        cap.release()
        hands.close()
        cv2.destroyAllWindows()
        esp32.close()


if __name__ == "__main__":
    main()
