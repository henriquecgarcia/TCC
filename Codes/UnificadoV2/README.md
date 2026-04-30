# ESP32 Autonomous Mobile Robot - PlatformIO

Projeto completo para um robô móvel autônomo com ESP32 DOIT DevKit V1, ponte H L298N, encoders, MPU6050, VL53L0X, planejamento A*, Pure Pursuit, EKF, avoidance dinâmico, SPIFFS e WebSocket.

## Arquitetura

- **Core 0**
  - `sensorsTask`: leitura do MPU6050 e VL53L0X.
  - `navigationTask`: avoidance, atualização do mapa dinâmico e A*.
- **Core 1**
  - `controlTask`: EKF, Pure Pursuit, conversão diferencial, PID e PWM.
  - `webTask`: telemetria JSON, limpeza de clientes WebSocket e PowerManager.

O `loop()` principal não executa lógica pesada. O sistema roda por tarefas FreeRTOS fixadas em núcleo.

## Pipeline de controle

```text
A* -> Path em grid -> Pure Pursuit -> curvatura -> vL/vR -> PID por roda -> PWM L298N
```

## Como compilar e enviar

```bash
pio run
pio run --target upload
pio run --target uploadfs
pio device monitor
```

A interface web fica disponível no access point:

- SSID: `ESP32_ROBOT`
- Senha: `esp32robot`
- URL: `http://192.168.4.1/`

## Mapa

O arquivo `/data/map.bin` usa 1 byte por célula:

- `0`: livre
- `1`: ocupado

O firmware lê o mapa estático via SPIFFS com `seek()`, sem carregar o bitmap inteiro na RAM. Obstáculos detectados pelo VL53L0X são gravados em overlay RAM e têm prioridade sobre o mapa estático.

## WebSocket

Endpoint:

```text
/ws
```

Telemetria enviada:

```json
{
  "x": 0.1,
  "y": 0.1,
  "theta": 0.0,
  "tof": 400,
  "active": true,
  "w": 32,
  "h": 32,
  "map": [0, 1, 2],
  "path": [{"x": 0.15, "y": 0.25}]
}
```

Comandos aceitos:

```json
{"cmd":"goal","gx":5,"gy":5}
{"cmd":"pose","x":0.05,"y":0.05,"theta":0}
{"cmd":"stop"}
{"cmd":"clearDynamic"}
```

## Ajustes necessários no robô real

Antes dos testes reais, calibre:

- `WHEEL_BASE_M`
- `WHEEL_DIAMETER_M`
- `TICKS_PER_REV`
- ganhos PID em `DriveBase.cpp`
- `OBSTACLE_MM`
- `CELL_SIZE_M`
- sentido físico de cada motor e encoder

## Observações de segurança

- Teste primeiro com o robô suspenso.
- Confirme polaridade dos motores e alimentação separada do ESP32.
- Una GND da fonte dos motores e GND do ESP32.
- O L298N tem queda de tensão considerável; ajuste bateria e limites de PWM conforme carga real.

## Atualização: comandos manuais e avisos na UI

A interface LCARS agora possui quatro comandos manuais via WebSocket:

- `VÁ PARA FRENTE`: aciona os dois lados do robô para frente por `MANUAL_LINEAR_DURATION_MS`.
- `VÁ PARA TRÁS`: aciona os dois lados do robô para trás por `MANUAL_LINEAR_DURATION_MS`.
- `VIRE 90° PARA DIREITA`: executa giro no próprio eixo até -90° em relação ao theta atual, com timeout de segurança.
- `VIRE 90° PARA ESQUERDA`: executa giro no próprio eixo até +90° em relação ao theta atual, com timeout de segurança.

Para reduzir o problema de motores parados por zona morta do L298N/motores DC, o `DriveBase` agora aplica um feed-forward mínimo (`MIN_MOVING_PWM`) sempre que existe comando de velocidade diferente de zero.

Quando o A* não consegue gerar caminho, o firmware para os motores e envia um campo `toast` na telemetria JSON. A UI exibe essa mensagem como um aviso visual no canto inferior direito.

O arquivo `/data/map.bin` foi regenerado como mapa 32x32 com obstáculos internos e com a célula `(0,0)` livre.


## Alteracao V3 - WebSocket e movimento manual

- A telemetria agora envia o mapa completo apenas periodicamente, mantendo as atualizacoes de pose mais leves.
- Os comandos WebSocket agora recebem ACK simples para indicar se foram aceitos pelo ESP32.
- O comando manual de frente/tras passou a ter timeout de seguranca maior, evitando o comportamento de andar apenas poucos centimetros. Use o botao **PARADA** para interromper antes do timeout.
- A UI preserva o ultimo mapa recebido quando uma mensagem de telemetria leve chega sem o campo `map`.
