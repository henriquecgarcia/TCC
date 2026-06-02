# Correções de debug - log 2026-05-28

Correções aplicadas com base no log `logs-2026-05-28T18-50-19-331Z(1).txt`.

## 1. Giro acumulando para o lado errado

No log, a curva solicitada tinha alvo interno positivo, mas o `Turning Angle Z` ia para negativo e o erro aumentava até timeout. Foi adicionada calibração explícita do sinal do gyro Z:

```cpp
GYRO_Z_LEFT_TURN_SIGN = -1.0;
GYRO_Z_RIGHT_TURN_SIGN = 1.0;
```

Agora a PonteH converte o gyro bruto em progresso angular positivo da manobra antes de comparar com o alvo.

## 2. Primeira célula com distância absurda

A primeira célula apareceu com `1.226 m`, apesar do alvo de `0.30 m`. Isso indica delta velho/lixo dos encoders depois de reset interno dos motores.

Foi criada a função:

```cpp
syncEncoderReferenceForEKF();
```

Ela sincroniza a referência bruta dos encoders sempre que o motor para, avança, dá ré ou inicia curva.

## 3. Distância da célula calculada por delta dos encoders

A distância de célula agora é calculada pelo delta assinado dos encoders desde o início da célula, e não apenas pelo deslocamento da pose contínua do EKF.

## 4. Proteção contra célula impossível

Se a distância estimada da célula ultrapassar `CELL_OVERRUN_ABORT_FACTOR * GRID_CELL_SIZE_M`, a navegação aborta com:

```txt
cell_distance_overrun
```

## 5. Mapa não atualiza no meio da célula

Durante navegação ativa, a célula oficial do mapa fica travada e só é atualizada ao iniciar/finalizar célula ou ao replanejar. Isso evita que drift intermediário faça o A* pular célula.

## Teste recomendado

1. Rodar uma célula:

```json
{"action":"forward_cells","cells":1}
```

A distância no log deve ficar próxima de `0.30 m`.

2. Testar giro isolado:

```txt
left
```

O `Turning Angle Z` deve crescer positivo até o alvo interno.

3. Testar caminho:

```json
{"action":"path_to","x":5,"y":5,"execute":true}
```

## Correção de preservação do mapa 7x7

O mapa-base voltou a ser estritamente o mapa binário 7x7 carregado na inicialização:

```txt
1111111
1000001
1000001
1000001
1000001
1000001
1111111
```

Detecções do VL53L0X não escrevem mais `1` no bitmap oficial via `setCell()`. Quando um obstáculo frontal é detectado, o robô para e aborta o movimento com `obstacle_detected_map_preserved`, preservando o mapa carregado.
