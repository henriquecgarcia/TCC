# Correção do giro por encoder - filtro anti-spike

Esta versão mantém `TURN_ENCODER_SCALE = 0.50`, mas adiciona uma segunda proteção porque o log mostrou um problema diferente: pulos impossíveis no encoder.

Exemplo do log anterior:

- raw ~54°
- no ciclo seguinte raw ~276°

Isso fazia a curva finalizar cedo, mesmo com a escala aplicada.

## O que foi adicionado

- Filtro incremental de giro por encoder.
- Rejeição de delta bruto impossível por ciclo.
- Rejeição de pulso reverso grande durante giro.
- Contador `turn_rejected_spikes` na telemetria.
- Trava de picos nos ticks antes do EKF.
- QMC continua disponível, mas só corrige frente/ré se a inovação for menor que 25°.
- Após giro planejado, o heading lógico é fixado em `targetHeadingRad` para impedir correções residuais absurdas de 170°/180°.

## Como calibrar

Comece com:

```cpp
static constexpr double TURN_ENCODER_SCALE = 0.50;
```

Se ainda virar pouco fisicamente, diminua para `0.45` ou `0.40`.
Se virar demais fisicamente, aumente para `0.55` ou `0.60`.

Se aparecerem muitos logs de spike, o problema é elétrico/físico nos encoders: ruído, pull-up, fio solto, cabo passando perto do motor ou falta de GND comum confiável.
