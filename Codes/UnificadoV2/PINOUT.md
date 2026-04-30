# PINOUT - ESP32 DOIT DevKit V1

## Ponte H L298N

| Função | GPIO | Observação |
|---|---:|---|
| Motor esquerdo IN1 | GPIO 26 | Direção |
| Motor esquerdo IN2 | GPIO 27 | Direção |
| Motor esquerdo ENA/PWM | GPIO 25 | PWM canal 0, 20 kHz |
| Motor direito IN1 | GPIO 14 | Direção |
| Motor direito IN2 | GPIO 12 | Direção; cuidado com boot strapping se houver pull externo |
| Motor direito ENB/PWM | GPIO 13 | PWM canal 1, 20 kHz |

## Encoders ópticos E2-Q2

| Função | GPIO | Observação |
|---|---:|---|
| Encoder esquerdo A | GPIO 34 | Entrada apenas, interrupção |
| Encoder esquerdo B | GPIO 35 | Entrada apenas, interrupção |
| Encoder direito A | GPIO 32 | Entrada, interrupção |
| Encoder direito B | GPIO 33 | Entrada, interrupção |

> GPIO 34 e GPIO 35 não possuem pull-up interno efetivo em algumas placas. Use pull-up externo se o encoder exigir.

## Barramento I2C

| Função | GPIO | Observação |
|---|---:|---|
| SDA | GPIO 21 | MPU6050 + VL53L0X |
| SCL | GPIO 22 | MPU6050 + VL53L0X |

## Sensores

| Dispositivo | Interface | Endereço típico |
|---|---|---:|
| MPU6050 | I2C | 0x68 |
| VL53L0X | I2C | 0x29 |

## Alimentação

| Linha | Uso |
|---|---|
| 5V | Lógica de alguns módulos, conforme regulador usado |
| 3V3 | Sensores I2C quando os módulos suportarem 3,3 V |
| GND | Terra comum entre ESP32, sensores e driver dos motores |
| VMOT L298N | Alimentação separada dos motores |

## Avisos

- Não alimente motores pelo 5V do ESP32.
- Confirme se os módulos MPU6050 e VL53L0X possuem conversão de nível ou se operam diretamente em 3,3 V.
- O GPIO 12 é pino de strapping; caso a placa falhe no boot, mova `RIGHT_IN2` para outro GPIO livre e atualize `RobotConfig.h`.
