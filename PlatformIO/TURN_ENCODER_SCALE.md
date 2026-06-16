# TURN_ENCODER_SCALE

Foi adicionada a constante `TURN_ENCODER_SCALE` em `src/carrinho.ino` para calibrar o giro por encoder sem precisar alterar `ODOM_WHEEL_BASE_M` diretamente.

```cpp
static constexpr double TURN_ENCODER_SCALE = 0.50;
```

A escala é aplicada apenas na estimativa angular de curva:

```cpp
encoderTurnRad = rawEncoderTurnRad * TURN_ENCODER_SCALE;
```

## Como ajustar

- Se o robô ainda virar pouco fisicamente, reduza o valor.
  - Exemplo: `0.50` -> `0.40`.
- Se o robô virar demais fisicamente, aumente o valor.
  - Exemplo: `0.50` -> `0.60`.

O log agora mostra os dois valores:

```text
[PonteH] Encoder Turn Z scaled: ... | raw: ... deg | scale: 0.50
```

O valor `raw` é a estimativa antes da escala. O valor `scaled` é o valor usado para decidir quando parar a curva.

## Observação

As curvas voltaram a ser finalizadas por encoder. O QMC5883L fica apenas como referência/telemetria durante curvas, porque os logs mostraram saltos magnéticos grandes com os motores ligados.
