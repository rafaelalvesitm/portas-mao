#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""Standalone (non-ROS) test tool: verify the ESP32 + PCA9685 hand hardware
over its USB serial cable before wiring it to the webcam tracker. Sweeps each
finger servo through its range one at a time so you can confirm the right
channel moves the right finger, in the right direction.

The ESP32's serial port must be attached to WSL2 (`usbipd attach --wsl`, same
as the webcam) and Thonny must be disconnected from it — only one process can
hold the port open at a time.

Usage:
    python3 utils/test_esp32_hand.py --port /dev/ttyUSB0
    python3 utils/test_esp32_hand.py --port /dev/ttyUSB0 --finger thumb --angle 90
"""

import argparse
import time

import serial

FINGER_ORDER = ["thumb", "index", "middle", "ring", "pinky"]
BAUDRATE = 115200


def send(ser: serial.Serial, angles: dict) -> str:
    line = ",".join(f"{name}={angle}" for name, angle in angles.items())
    started = time.monotonic()
    ser.reset_input_buffer()
    ser.write((line + "\n").encode())
    reply = ser.readline().decode(errors="replace").strip()  # blocks up to ser.timeout
    elapsed_ms = (time.monotonic() - started) * 1000
    return f"{reply or '(sem resposta)'} ({elapsed_ms:.0f} ms)"


def sweep(ser: serial.Serial):
    print("Testando ESP32 — um dedo de cada vez, 0 -> 180 -> 0.")
    print("Confira visualmente se o dedo certo se move, na direção certa.\n")
    for name in FINGER_ORDER:
        print(f"-- {name} --")
        for angle in (0, 90, 180, 90, 0):
            reply = send(ser, {name: angle})
            print(f"  angle={angle:3d} -> {reply}")
            time.sleep(0.6)
    print("\nTeste concluído.")


def main():
    parser = argparse.ArgumentParser(description="Test the ESP32/PCA9685 hand over serial.")
    parser.add_argument("--port", required=True, help="Serial port, e.g. /dev/ttyUSB0.")
    parser.add_argument("--finger", choices=FINGER_ORDER, help="Move only this finger instead of sweeping all.")
    parser.add_argument("--angle", type=int, help="Servo angle (0-180) to send with --finger.")
    args = parser.parse_args()

    with serial.Serial(args.port, BAUDRATE, timeout=2.0) as ser:
        time.sleep(2.0)  # let the ESP32 finish its boot prints before we start talking to it
        if args.finger:
            if args.angle is None:
                parser.error("--finger requires --angle")
            print(send(ser, {args.finger: args.angle}))
        else:
            sweep(ser)


if __name__ == "__main__":
    main()
