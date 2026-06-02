# ARCHITECTURE.md — Arquitetura do firmware

## Objetivo

O projeto implementa um robô móvel baseado em ESP32 para navegação em grade, com células de **30 cm x 30 cm**, controle por encoders, auxílio inercial com MPU6050, detecção frontal com VL53L0X, planejamento A* e replanejamento dinâmico.

## Estrutura principal

```txt
src/
  carrinho.ino              Firmware principal e orquestração
  RobotConfig.h             Constantes globais simples

lib/
  Motor/                    Controle de motor L298N com PWM LEDC e PID de RPM
  Encoder/                  Leitura dos encoders ópticos
  PID/                      Controlador PID genérico
  MPU6050_Custom/           Interface da IMU
  VL53L0X/                  Interface do sensor ToF frontal
  PoseEKF/                  Estimativa de pose x, y, theta
  Mapping/                  Bitmap de ocupação e A* com penalidade de curvas
  ExperimentLogger/         Log CSV dos experimentos do TCC
  WebInterface/             Servidor HTTP/WebSocket
  BrownoutLogger/           Registro de brownout

data/
  dashboard.html/css/js     Interface moderna do robô, sem tema LCARS
  brownout.html/css/js      Painel de brownout
  monitor.html/css/js       Monitoramento auxiliar
```

## Fluxo de controle

1. O `loop()` atualiza sensores, EKF e mapa discreto.
2. A `PonteH` controla motores e giros.
3. O `GridCellNavigator` executa células de 30 cm.
4. Durante cada célula, o navegador calcula erro angular pelo EKF.
5. A `PonteH` aplica PID angular contínuo nos motores.
6. O VL53L0X detecta obstáculo frontal.
7. A célula à frente é marcada como ocupada no mapa.
8. Se o robô estiver executando um caminho A*, o caminho é recalculado.
9. O `ExperimentLogger` registra eventos e telemetria em CSV.

## Estados principais do GridCellNavigator

| Estado | Função |
|---|---|
| `idle` | Sem navegação ativa |
| `align_to_path_step` | Alinha o robô para a próxima célula do caminho A* |
| `wait_path_turn` | Aguarda término do giro para seguir caminho |
| `start_cell` | Inicializa deslocamento de uma célula |
| `moving_cell` | Anda 30 cm com PID angular contínuo |
| `settle_after_cell` | Pausa curta para estabilização |
| `correct_heading` | Corrige erro angular residual |
| `wait_turn` | Aguarda correção angular residual |
| `finished` | Navegação concluída |
| `aborted` | Navegação abortada por erro, obstáculo sem caminho ou cancelamento |

## Decisões de projeto

- O movimento não usa `delay()`; a navegação é não bloqueante.
- O mapa é um bitmap para reduzir uso de RAM.
- O A* considera quatro direções e penaliza curvas.
- A célula lógica tem 0,30 m para facilitar validação física em pista.
- O log CSV foi colocado na SPIFFS para simplificar coleta de dados na banca.

## Observação sobre modularização

A estrutura já possui módulos em `lib/`. A classe `GridCellNavigator` ainda permanece em `src/carrinho.ino` porque depende diretamente da classe `PonteH`, que também está no arquivo principal. A próxima refatoração recomendada é mover `PonteH` para `lib/DriveBase/` e então mover `GridCellNavigator` para `lib/Navigator/` sem dependências circulares.


## Mapa binário 7x7 do TCC

A versão atual usa um mapa binário fixo de 7x7 células, onde cada célula representa 30 cm x 30 cm no ambiente físico.

Convenção de coordenadas:

- `(0,0)` é o canto superior esquerdo da matriz.
- `x` cresce para a direita.
- `y` cresce para baixo.
- `1` representa célula ocupada/parede.
- `0` representa célula livre.
- O robô inicia em `(1,1)`, logo dentro da borda ocupada.

Mapa inicial:

```txt
1111111
1000001
1000001
1000001
1000001
1000001
1111111
```

A pose contínua do EKF inicia em `(0.0 m, 0.0 m, 0.0 rad)`, mas essa origem contínua é convertida para a célula discreta `(1,1)` usando `MAP_ORIGIN_X = 1` e `MAP_ORIGIN_Y = 1`.

A telemetria enviada ao WebSocket agora inclui também o objeto `location`, com:

```json
{
  "grid_x": 1,
  "grid_y": 1,
  "world_x_m": 0.0,
  "world_y_m": 0.0,
  "theta_rad": 0.0,
  "theta_deg": 0.0,
  "cell_size_m": 0.5,
  "origin_grid_x": 1,
  "origin_grid_y": 1
}
```

Isso evita confusão entre posição contínua em metros e posição discreta no mapa binário.

## Atualização: sensores em threads FreeRTOS

A leitura do MPU6050 e do VL53L0X foi separada do `loop()` principal usando duas tarefas FreeRTOS:

- `mpu6050_task`: atualiza giroscópio, acelerômetro e temperatura em cache.
- `vl53l0x_task`: atualiza a distância frontal em cache.

As duas tarefas compartilham o barramento I²C por meio de `i2cBusMutex`, evitando acessos simultâneos ao `Wire`. O controle de motores, EKF, WebSocket e navegador por células usam apenas o cache por meio das funções `getCachedGyroZ()` e `getCachedFrontDistanceMm()`.

Essa separação evita que uma leitura lenta do VL53L0X ou do MPU6050 bloqueie o controle do robô durante uma célula.

## Atualização: anti-drift entre células

Ao concluir cada célula, o navegador agora fixa a pose contínua exatamente na célula discreta esperada. Isso sincroniza:

- pose do EKF em metros;
- célula atual do mapa;
- heading-alvo da navegação;
- ticks de referência dos encoders.

Durante as pausas entre células, `holdCurrentPose()` mantém `x`, `y` e `theta` congelados e apenas sincroniza os ticks atuais dos encoders. Assim, ruído do MPU6050 ou pulsos espúrios dos encoders não fazem a localização andar sozinha enquanto o robô está parado.


## Atualização de navegação em ré

A execução de caminho A* agora avalia o heading do próximo passo. Se a célula planejada estiver atrás do robô, o `GridCellNavigator` marca a próxima célula como `reverse` e chama `PonteH::backward()` em vez de girar 180 graus. A pose contínua é mantida no heading atual e corrigida ao final da célula com `snapPoseToExpectedCell()`.

Para manter a localização correta durante ré, o firmware usa contadores assinados para o EKF. Os encoders físicos continuam sendo lidos normalmente, mas os deltas são convertidos para sinais positivos ou negativos conforme o movimento atual da PonteH.
