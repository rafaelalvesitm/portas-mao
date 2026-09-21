# MicroPython firmware for ESP32 (edit/run with Thonny).
#
# Receives already-computed servo angles (0-180) over the USB serial cable from
# the webcam+MediaPipe finger tracker (hand_teleop.py on the PC side) and
# writes them to a PCA9685 board over I2C. All the tracking/calibration logic
# stays in Python on the PC; this file only turns "thumb=122" into the right
# PWM pulse on the right PCA9685 channel.
#
# Uses the SAME USB cable/UART Thonny's REPL runs on — no WiFi needed. That
# also means: once this is running as main.py for real use, Thonny's Shell
# can't be connected to the board at the same time (both would fight over the
# same serial port) — disconnect Thonny before hand_teleop.py opens the port.
#
# Wire protocol (one line per update, comma-separated "name=angle"):
#   thumb=90,index=45,middle=60,ring=30,pinky=10\n
#
# Setup:
#   1. Flash MicroPython onto the ESP32 (once).
#   2. Open this file in Thonny, connected to the ESP32.
#   3. Check I2C_SDA_PIN / I2C_SCL_PIN match your wiring below.
#   4. Run it (F5) and confirm the Shell prints the PCA9685 at 0x40 in the I2C
#      scan. Test with utils/test_esp32_hand.py --port <serial-port>.
#   5. Once it works, use Thonny's "Save as... > MicroPython device" to save
#      this file as main.py on the ESP32 so it auto-runs on power-up.

import sys
import select
import time
from machine import I2C, Pin

# I2C pins wired to the PCA9685 (SDA/SCL). 21/22 are the typical default on a
# plain ESP32 DevKit — check your board/wiring if it doesn't detect the PCA9685.
I2C_SDA_PIN = 21
I2C_SCL_PIN = 22

PCA9685_ADDRESS = 0x40
SERVO_FREQ_HZ = 50
PULSE_MIN_US = 500
PULSE_MAX_US = 2500

# Same finger -> PCA9685 channel mapping as hand_teleop.py.
FINGER_CHANNELS = {
    "thumb": 4,
    "index": 0,
    "middle": 1,
    "ring": 2,
    "pinky": 3,
}

_MODE1 = 0x00
_PRESCALE = 0xFE
_LED0_ON_L = 0x06


class PCA9685:
    def __init__(self, i2c, address=PCA9685_ADDRESS):
        self.i2c = i2c
        self.address = address
        self.i2c.writeto_mem(self.address, _MODE1, bytes([0x00]))

    def set_freq_hz(self, freq_hz):
        prescale = int(25000000.0 / (4096 * freq_hz) - 1)
        old_mode = self.i2c.readfrom_mem(self.address, _MODE1, 1)[0]
        self.i2c.writeto_mem(self.address, _MODE1, bytes([(old_mode & 0x7F) | 0x10]))  # sleep
        self.i2c.writeto_mem(self.address, _PRESCALE, bytes([prescale]))
        self.i2c.writeto_mem(self.address, _MODE1, bytes([old_mode]))
        time.sleep_ms(5)
        self.i2c.writeto_mem(self.address, _MODE1, bytes([old_mode | 0xA1]))  # restart + auto-increment

    def set_pwm(self, channel, on, off):
        reg = _LED0_ON_L + 4 * channel
        self.i2c.writeto_mem(self.address, reg, bytes([
            on & 0xFF, (on >> 8) & 0xFF,
            off & 0xFF, (off >> 8) & 0xFF,
        ]))

    def set_angle(self, channel, angle_deg):
        angle_deg = max(0, min(180, angle_deg))
        pulse_us = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * (angle_deg / 180.0)
        period_us = 1_000_000.0 / SERVO_FREQ_HZ
        ticks = int(pulse_us * 4096 / period_us)
        self.set_pwm(channel, 0, ticks)


def parse_line(line):
    params = {}
    for pair in line.split(","):
        if "=" in pair:
            k, v = pair.split("=", 1)
            params[k.strip()] = v.strip()
    return params


def handle_line(line, pca):
    applied = {}
    for name, channel in FINGER_CHANNELS.items():
        params = parse_line(line)
        if name in params:
            try:
                angle = int(params[name])
            except ValueError:
                continue
            pca.set_angle(channel, angle)
            applied[name] = angle
    return applied


def serve(pca):
    print("Pronto. Aguardando comandos na serial, ex: thumb=90,index=45,middle=60,ring=30,pinky=10")

    poll = select.poll()
    poll.register(sys.stdin, select.POLLIN)

    buf = ""
    while True:
        if not poll.poll(0):
            time.sleep_ms(2)
            continue

        ch = sys.stdin.read(1)
        if ch in ("\n", "\r"):
            line = buf.strip()
            buf = ""
            if not line:
                continue
            try:
                applied = handle_line(line, pca)
                print("ok", applied)
            except Exception as e:
                print("erro:", e)
        else:
            buf += ch
            if len(buf) > 200:  # guard against a malformed/never-terminated line
                buf = ""


def main():
    i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
    print("Dispositivos I2C encontrados:", [hex(a) for a in i2c.scan()])

    pca = PCA9685(i2c)
    pca.set_freq_hz(SERVO_FREQ_HZ)

    serve(pca)


if __name__ == "__main__":
    main()
