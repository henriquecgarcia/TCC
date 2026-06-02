# PINOUTS.md — Pinagem atual do projeto

## ESP32 DOIT DevKit V1

## I2C

| Sinal | GPIO |
|---|---:|
| SDA | 21 |
| SCL | 22 |

Sensores no barramento:

- MPU6050;
- VL53L0X.

## Motores — L298N

### Motor direito

| Função | GPIO |
|---|---:|
| IN1 | 14 |
| IN2 | 12 |
| PWM | 13 |
| Canal LEDC | 0 |

### Motor esquerdo

| Função | GPIO |
|---|---:|
| IN1 | 26 |
| IN2 | 27 |
| PWM | 25 |
| Canal LEDC | 1 |

## Encoders

### Encoder direito

| Canal | GPIO |
|---|---:|
| A | 39 |
| B | 36 |

### Encoder esquerdo

| Canal | GPIO |
|---|---:|
| A | 34 |
| B | 35 |

## LED

| Função | GPIO |
|---|---:|
| LED carroceria/status | 2 |

## Observações

- GPIOs 34, 35, 36 e 39 são somente entrada no ESP32.
- Confirme alimentação externa dos motores; não alimente motores pelo 5V do ESP32.
- Mantenha GND comum entre ESP32, L298N e sensores.

