# Correção do controle de giro com QMC5883L

## Problema observado

Durante os testes, o robô ficava girando sem conseguir decidir quando parar. O log mostrava saltos grandes no heading do QMC5883L, por exemplo valores alternando entre ângulos positivos e negativos distantes em poucos ciclos de controle.

Isso acontece porque o QMC5883L é um magnetômetro. Ele mede campo magnético, não velocidade angular. Durante a rotação do robô, o L298N, os motores, a bateria e os cabos de corrente podem interferir fortemente na leitura.

## Correção aplicada

O controle de parada da curva deixou de depender do QMC5883L.

Agora:

1. O comando `left` ou `right` define a direção física dos motores.
2. O alvo angular é convertido para um alvo de deslocamento diferencial pelos encoders.
3. A curva para quando o progresso estimado pelos encoders atinge o ângulo alvo.
4. O QMC5883L continua sendo usado para telemetria/heading, mas não para decidir o fim da curva.

## Importante

O EKF também deixou de fazer `updateWithHeading()` durante curvas. Durante giro, o `theta` vem da odometria diferencial dos encoders. Isso evita que o heading magnético ruidoso jogue a localização para qualquer lado.

## Ajustes úteis

No arquivo `src/carrinho.ino`:

- `ODOM_WHEEL_BASE_M`: distância entre rodas.
- `ODOM_WHEEL_RADIUS_M`: raio da roda.
- `ODOM_TICKS_PER_REV`: ticks por volta usados na odometria.
- `turnToleranceRad`: tolerância angular para finalizar curva.
- RPMs dentro do bloco de giro:
  - fase grossa: `105 RPM`;
  - fase fina: `55 RPM`;
  - final: `38–45 RPM`.

Se o robô parar antes/depois do ângulo desejado, calibre primeiro `ODOM_WHEEL_BASE_M`.
