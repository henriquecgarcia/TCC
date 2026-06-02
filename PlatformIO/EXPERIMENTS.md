# EXPERIMENTS.md — Ensaios formais para a monografia

Este documento propõe ensaios mensuráveis para validar a navegação autônoma do robô.

## Log experimental CSV

O firmware grava dados em:

```txt
/api/tcc-log.csv
```

Também é possível limpar o arquivo por:

```txt
POST /api/tcc-log/clear
```

Comandos WebSocket:

```json
{"action":"experiment_log","mode":"enable"}
{"action":"experiment_log","mode":"disable"}
{"action":"experiment_log","mode":"clear"}
```

Campos registrados:

- tempo em `millis`;
- evento da navegação;
- estado da máquina de estados;
- células solicitadas/concluídas;
- célula atual do mapa;
- pose estimada `x`, `y`, `theta`;
- heading alvo;
- erro angular;
- distância percorrida na célula atual;
- distância frontal VL53L0X;
- obstáculo detectado;
- modo caminho A*;
- índice do caminho;
- se houve replanejamento.

## Experimento 1 — Deslocamento por célula

Objetivo: validar se uma célula lógica de 30 cm corresponde ao deslocamento real.

Comandos:

```json
{"action":"forward_cells","cells":1}
{"action":"forward_cells","cells":2}
{"action":"forward_cells","cells":3}
{"action":"forward_cells","cells":5}
```

Métricas:

- distância final esperada;
- distância final real;
- erro absoluto;
- erro percentual;
- repetibilidade.

## Experimento 2 — Controle angular contínuo

Objetivo: medir se o PID angular mantém o robô alinhado durante cada célula.

Procedimento:

1. Ative o log experimental.
2. Execute 5 células para frente.
3. Analise o campo `heading_error_rad`.
4. Compare erro máximo e erro final.

Métricas:

- erro angular máximo;
- erro angular médio;
- erro angular residual após cada parada;
- número de correções residuais.

## Experimento 3 — Planejamento A*

Objetivo: verificar se o robô executa um caminho planejado em grade.

Comando:

```json
{"action":"path_to","x":8,"y":4,"execute":true}
```

Métricas:

- tamanho do caminho;
- número de curvas;
- sucesso/falha;
- distância total percorrida;
- diferença entre caminho planejado e executado.

## Experimento 4 — Obstáculo dinâmico e replanejamento

Objetivo: verificar se o robô detecta obstáculo imprevisto, atualiza o mapa e recalcula o caminho.

Procedimento:

1. Envie um destino com `path_to`.
2. Durante o trajeto, coloque um obstáculo à frente.
3. Observe se o mapa marca uma célula ocupada.
4. Verifique se o log registra `obstacle_replanned`.
5. Confirme se o robô tenta seguir um novo caminho.

Métricas:

- distância de detecção;
- tempo até parada;
- sucesso no replanejamento;
- número de replanejamentos;
- chegada ao alvo.

