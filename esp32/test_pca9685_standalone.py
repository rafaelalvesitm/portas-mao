# Teste isolado de hardware — rode direto no Thonny (F5), com a ESP32 conectada.
# NÃO depende do hand_pca9685_server.py nem de nada vindo do PC: só I2C direto
# pra PCA9685. Serve pra confirmar se o problema é na placa/fiação/energia dos
# servos, sem qualquer camada de software (serial, protocolo, etc.) no meio.
#
# Ao rodar, ele faz uma varredura automática dos canais 0-4 e volta pro prompt
# do Shell — depois disso você pode digitar comandos direto, por exemplo:
#
#   >>> pca.set_angle(4, 90)
#   >>> pca.set_angle(0, 0)
#
# para testar qualquer canal/ângulo na hora, sem re-rodar o script.

import time
from machine import I2C, Pin

I2C_SDA_PIN = 21
I2C_SCL_PIN = 22
PCA9685_ADDRESS = 0x40
SERVO_FREQ_HZ = 50
PULSE_MIN_US = 500
PULSE_MAX_US = 2500

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

    def get_pwm(self, channel):
        reg = _LED0_ON_L + 4 * channel
        data = self.i2c.readfrom_mem(self.address, reg, 4)
        on = data[0] | (data[1] << 8)
        off = data[2] | (data[3] << 8)
        return on, off

    def set_angle(self, channel, angle_deg):
        angle_deg = max(0, min(180, angle_deg))
        pulse_us = PULSE_MIN_US + (PULSE_MAX_US - PULSE_MIN_US) * (angle_deg / 180.0)
        period_us = 1_000_000.0 / SERVO_FREQ_HZ
        ticks = int(pulse_us * 4096 / period_us)
        self.set_pwm(channel, 0, ticks)


i2c = I2C(0, scl=Pin(I2C_SCL_PIN), sda=Pin(I2C_SDA_PIN), freq=400000)
print("Dispositivos I2C encontrados:", [hex(a) for a in i2c.scan()])

pca = PCA9685(i2c)
pca.set_freq_hz(SERVO_FREQ_HZ)

CHANNELS_TO_TEST = [0, 1, 2, 3, 4]  # index, middle, ring, pinky, thumb

print("\nVarredura automatica dos canais 0-4 (0 -> 90 -> 180 -> 90 -> 0):\n")
for ch in CHANNELS_TO_TEST:
    print(f"--- Canal {ch} ---")
    for angle in (0, 90, 180, 90, 0):
        pca.set_angle(ch, angle)
        on, off = pca.get_pwm(ch)
        # 'off' e o registrador que define a largura de pulso (em ticks de 4096
        # por periodo de 20ms) -- se ele mudar a cada angulo, a escrita I2C esta
        # OK; se o servo mesmo assim nao se move, o problema e eletrico
        # (energia V+, fiacao do servo, ou pino OE), nao I2C/software.
        print(f"  angulo={angle:3d}  registrador ON={on} OFF={off}")
        time.sleep(1.0)

print("\nVarredura concluida. Agora voce pode testar interativamente, ex:")
print("  pca.set_angle(4, 90)")
print("  pca.set_angle(0, 0)")
