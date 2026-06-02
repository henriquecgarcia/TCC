# CALIBRATION.md — Calibração do robô do TCC

Este documento define o processo de calibração recomendado para o robô móvel baseado em ESP32, encoders, MPU6050, VL53L0X e navegação em grade de **30 cm x 30 cm**.

## 1. Calibração física da célula

Cada célula lógica do projeto representa **0,30 m**. Antes de validar A* ou replanejamento, valide o deslocamento de uma única célula.

Procedimento:

1. Marque no chão uma linha inicial e uma linha a 30 cm.
2. Posicione o robô alinhado com a linha inicial.
3. Envie o comando:

```json
{"action":"forward_cells","cells":1}
```

4. Meça a distância real percorrida.
5. Repita pelo menos 10 vezes.
6. Calcule média, erro absoluto, erro percentual e desvio padrão.

Tabela sugerida:

| Ensaio | Esperado cm | Medido cm | Erro cm | Erro % | Observação |
|---|---:|---:|---:|---:|---|
| 1 | 50 |  |  |  |  |
| 2 | 50 |  |  |  |  |
| 3 | 50 |  |  |  |  |

## 2. Calibração dos encoders

Confira os parâmetros físicos usados pelo código:

- roda: 65 mm de diâmetro;
- raio configurado no EKF: 0,0325 m;
- disco encoder: 10 dentes;
- leitura em quadratura: validar no código se o total efetivo é 40 transições por volta.

Se o robô anda menos que 30 cm de forma consistente, revise:

- diâmetro real da roda sob carga;
- folga mecânica;
- redução entre motor e roda;
- contagem real dos ticks por volta;
- reset de encoder dentro do ciclo de controle.

## 3. Calibração do MPU6050

A correção angular usa `poseEKF->getTheta()` e `sensorMPU->getGyroscopeZ()`.

Procedimento:

1. Ligue o robô parado em superfície plana.
2. Aguarde estabilizar por alguns segundos.
3. Verifique se o ângulo estimado não deriva rapidamente.
4. Execute giro de 90° e compare com medição real.
5. Ajuste ruído do EKF pelos comandos `ekf_config` se necessário.

Exemplo:

```json
{"action":"ekf_config","type":"smooth"}
```

## 4. Calibração do VL53L0X

O replanejamento dinâmico usa `FRONT_OBSTACLE_REPLAN_MM = 220`.

Procedimento:

1. Coloque um obstáculo a 30 cm.
2. Confirme telemetria de distância frontal.
3. Coloque a 20 cm.
4. Confirme que o robô para, marca a célula à frente como ocupada e tenta replanejar.

Caso o robô pare cedo demais, reduza `FRONT_OBSTACLE_REPLAN_MM`. Caso pare tarde demais, aumente o valor, considerando o espaço de frenagem do robô.

## 5. PID angular contínuo

Durante cada célula, a `PonteH` recebe erro angular externo do `GridCellNavigator` e aplica o PID `pidCellHeading`.

Parâmetros iniciais:

```cpp
PID pidCellHeading = PID(2.4, 0.0, 0.18, controlInterval / 1000.0);
pidCellHeading.setMaxMin(1.2, -1.2);
```

Ajuste recomendado:

1. Comece com `Ki = 0`.
2. Aumente `Kp` até o robô corrigir sem oscilar demais.
3. Aumente `Kd` para amortecer oscilação.
4. Só use `Ki` se houver erro residual constante.


## Ajuste emergencial de giro: `TURN_COMMAND_SCALE`

Durante os testes físicos, foi observado que o carrinho girava aproximadamente o dobro do ângulo solicitado. Para compensar isso sem alterar a semântica dos comandos de alto nível, foi adicionada a constante:

```cpp
static constexpr double TURN_COMMAND_SCALE = 0.50;
```

Essa constante fica em `src/carrinho.ino` e é aplicada dentro de `PonteH::turnLeft()` e `PonteH::turnRight()`.

Com o valor atual:

- comando lógico de `90°` usa alvo interno de `45°`;
- comando lógico de `45°` usa alvo interno de `22,5°`;
- o A* continua pensando em giros normais, mas a PonteH corta o giro pela metade.

Se no teste real o robô ainda girar demais, reduza o valor para `0.45` ou `0.40`.
Se ele girar de menos, aumente para `0.55` ou `0.60`.

O ajuste definitivo recomendado para a monografia é calibrar o eixo Z do MPU6050, validar o sinal do giroscópio e comparar o ângulo integrado com uma medição física externa. Este fator de escala foi mantido como solução prática para estabilizar os testes atuais.
