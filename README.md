# Teleoperação da mão (webcam → ESP32 → PCA9685)

Controla os 5 dedos de uma mão robótica (estilo InMoov) individualmente, a partir do
rastreamento de uma mão humana pela webcam. Um script em Python (PC) usa o MediaPipe Hands
para medir o quanto cada dedo está dobrado e manda os ângulos calculados por cabo serial USB
para uma ESP32, que aplica esses ângulos nos servos através de uma placa PCA9685 (I2C).

```
Webcam --> MediaPipe Hands (Python/PC) --> serial USB --> ESP32 (MicroPython)
                                                                    |
                                                                  I2C
                                                                    v
                                                               PCA9685 --> 5 servos (dedos)
```

Nenhuma lógica de calibração roda na ESP32 — ela só recebe ângulos já prontos (0-180) e escreve
no canal certo da PCA9685. Todo o rastreamento/calibração fica no lado Python.

Este repositório é propositalmente simples: sem ROS 2, sem workspace `colcon`. É só um script
Python + o firmware da ESP32 + algumas ferramentas de teste — pensado para controlar essa mão,
não um robô inteiro.

## Estrutura do repositório

```
hand_teleop.py                  programa principal: webcam -> MediaPipe -> serial -> ESP32 -> servos

esp32/
  hand_pca9685_server.py       firmware MicroPython "de verdade" (roda como main.py na ESP32)
  test_pca9685_standalone.py   teste isolado de hardware, só roda no Thonny (sem serial/PC)

utils/
  test_webcam_hands.py         só webcam + MediaPipe, mostra ângulos na tela (sem hardware)
  test_esp32_hand.py           só ESP32 + PCA9685 por serial (varredura 0->180->0 por dedo)
```

## Hardware necessário

- Webcam USB
- ESP32 (testado com uma placa DevKit genérica, chip serial CH340)
- Placa PCA9685 (testado com um clone "HW-170")
- 5 micro servos (um por dedo)
- Fonte externa 5V para os servos (**não** alimente os servos pela porta USB/3.3V da ESP32)

### Ligações da PCA9685

- `VCC`, `GND`, `SCL`, `SDA` → ESP32 (I2C; nesta configuração `SDA=GPIO21`, `SCL=GPIO22`)
- `V+` / `GND` (bloco de terminais separado) → fonte externa de 5V. Esse é o barramento que
  alimenta os servos — é **diferente** do `VCC` da lógica, que pode vir da própria ESP32.
- **Pino `OE` (Output Enable)**: em placas genéricas tipo HW-170, esse pino **não** vem aterrado
  internamente por padrão. Sem um jumper ligando `OE` a qualquer `GND` da placa, o chip aceita
  comandos I2C normalmente mas nunca gera o sinal PWM de verdade nos servos. Se os servos não se
  mexerem mesmo com tudo certo no software, confira isso primeiro.
- Servos nos canais 0-4 (mapeamento em `FINGER_CHANNELS`, ver seção Calibração).

## Setup

### 1. Firmware da ESP32 (Thonny)

1. Grave o MicroPython na ESP32 (uma vez só):
   ```bash
   pip install esptool
   esptool.py --chip esp32 erase_flash
   esptool.py --chip esp32 write_flash -z 0x1000 esp32-<versao>.bin
   ```
   (baixe o `.bin` genérico em https://micropython.org/download/ESP32_GENERIC/)
2. No Thonny, conecte na ESP32 (Ferramentas → Opções → Interpretador → MicroPython/ESP32).
3. Abra `esp32/test_pca9685_standalone.py` e rode com F5 — confirma que o I2C encontra a PCA9685
   (`0x40` na lista) e testa os servos interativamente (`pca.set_angle(canal, angulo)` no Shell)
   **antes** de envolver qualquer coisa do lado do PC.
4. Confirmando que os servos respondem, abra `esp32/hand_pca9685_server.py`, confira
   `I2C_SDA_PIN`/`I2C_SCL_PIN`, rode com F5 e teste via `utils/test_esp32_hand.py` (próxima seção).
5. Quando estiver tudo certo, salve `hand_pca9685_server.py` no dispositivo como `main.py`
   (Thonny: Arquivo → Salvar como → Dispositivo MicroPython) para ele iniciar sozinho ao ligar.

**Importante**: `hand_pca9685_server.py` lê comandos pelo mesmo cabo USB que o Thonny usa pra
REPL. Só um processo pode segurar essa porta serial por vez — desconecte o Thonny antes de rodar
qualquer script do PC que fale com a ESP32.

### 2. Ambiente Python (PC)

Este projeto foi desenvolvido/testado num WSL2, usando um ambiente virtual isolado com
[uv](https://github.com/astral-sh/uv):

```bash
uv venv .venv --python 3.11
uv pip install --python .venv/bin/python opencv-python "mediapipe==0.10.14" pyserial
```

**Por que `mediapipe==0.10.14` e não a versão mais nova**: o MediaPipe recente (≥1.0) removeu a
API `mp.solutions.hands` usada aqui em favor de uma nova "Tasks API" que exige baixar um arquivo
de modelo separado. A 0.10.14 ainda tem a API antiga e funciona bem no Python 3.11.

### 3. Se estiver usando WSL2

A webcam e a porta serial da ESP32 não aparecem automaticamente no WSL2 — precisam ser
"emprestadas" do Windows via [usbipd-win](https://github.com/dorssel/usbipd-win):

```powershell
# No PowerShell (Admin), uma vez por dispositivo/boot:
usbipd list                              # ache o BUSID da webcam e da ESP32
usbipd bind --busid <BUSID>               # uma vez só, persiste entre reboots
usbipd attach --wsl --busid <BUSID>       # repita a cada reconexão
```

Depois de anexado, seu usuário precisa estar nos grupos `video` (webcam) e `dialout` (serial):
```bash
sudo usermod -aG video,dialout $USER
# abra um terminal novo pra valer o grupo
```

Se a `usbipd attach` falhar com "Device busy": feche qualquer app que esteja usando a câmera
(Teams, Câmera do Windows, etc.) antes de tentar de novo.

## Testando, em ordem

Cada passo isola uma camada — não pule direto pro programa principal se algo não tiver sido
validado antes.

1. **Hardware puro** (`esp32/test_pca9685_standalone.py`, só no Thonny): confirma que a PCA9685 e
   os servos respondem, sem nenhum software do PC envolvido.
2. **ESP32 por serial** (do PC, com o Thonny desconectado):
   ```bash
   uv run utils/test_esp32_hand.py --port /dev/ttyUSB0
   ```
   Varre cada dedo 0→180→0. Confirma canal certo, direção certa, e a comunicação serial PC↔ESP32.
3. **Webcam + MediaPipe, sem hardware**:
   ```bash
   uv run utils/test_webcam_hands.py
   ```
   Mostra o esqueleto da mão e o ângulo/estado de cada dedo na tela, sem mandar nada pra ESP32.
   Ao apertar `q`, imprime a faixa mín/máx de ângulo observada por dedo — útil pra recalibrar
   `ANGLE_OPEN_DEG`/`ANGLE_CLOSED_DEG`.
4. **Programa completo** (webcam → serial → servo de verdade):
   ```bash
   uv run hand_teleop.py --port /dev/ttyUSB0
   ```

## Calibração

As constantes abaixo aparecem repetidas em dois arquivos (`hand_teleop.py` e
`utils/test_webcam_hands.py`) — **se mudar uma, replique na outra**:

- `FINGER_CHANNELS`: qual canal da PCA9685 controla qual dedo (esse mapeamento também precisa
  bater com `FINGER_CHANNELS` em `esp32/hand_pca9685_server.py`).
- `FINGER_INVERTED`: `True` para um canal cujo servo está montado ao contrário (0° = fechado,
  180° = aberto). Hoje nenhum dedo precisa disso nesta montagem.
- `ANGLE_OPEN_DEG` / `ANGLE_CLOSED_DEG`: faixa de ângulo bruto (medido pelo MediaPipe) que
  corresponde a "totalmente aberto"/"totalmente fechado", **por dedo**. O polegar usa uma faixa
  diferente dos outros dedos (~125°-175°, contra ~40°-160°) porque a métrica de ângulo dele (base
  → nó → ponta) tem uma faixa de movimento natural bem mais estreita — usar a mesma faixa dos
  outros dedos deixava o polegar preso perto de "fechado" o tempo todo.

Para recalibrar: rode `utils/test_webcam_hands.py`, abra e feche a mão (e o polegar sozinho)
várias vezes, e use o resumo de mín/máx impresso ao sair como ponto de partida para os novos
limites.
