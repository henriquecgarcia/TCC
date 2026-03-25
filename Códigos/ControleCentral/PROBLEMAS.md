# Analise tecnica de riscos - ESP32 Carrinho (PlatformIO)

## Escopo analisado

Arquivos de controle, sensores e dinamica:
- src/carrinho.ino
- lib/PID/PID.h
- lib/PID/PID.cpp
- lib/Encoder/Encoder.h
- lib/Encoder/Encoder.cpp
- lib/Odometry/Odometry.h
- lib/Odometry/Odometry.cpp
- lib/VL53L0X/VL53L0X.h
- lib/VL53L0X/VL53L0X.cpp
- lib/PowerManager/PowerManager.h
- lib/PowerManager/PowerManager.cpp
- lib/Mapping/Map.h
- lib/Mapping/Map.cpp
- lib/MediaMovel/MediaMovel.h
- lib/MediaMovel/MediaMovel.cpp
- lib/LED/LED.h
- lib/LED/LED.cpp
- platformio.ini

## Resumo executivo

O projeto tem base funcional, mas ha pontos que podem gerar comportamento instavel em robo real: erro de logica no switch de movimento, controle angular com comparacao em valor absoluto, acumulacao/normalizacao de angulo inconsistente, risco de corrida nos encoders e dt de controle sensivel a jitter/bloqueio.

Os itens mais graves estao no controle de giro e no pipeline encoder -> PID -> PWM.

## Achados detalhados

## 1) CRITICO - Queda de caso no switch (fall-through) altera controle em frente

Local:
- src/carrinho.ino:555
- src/carrinho.ino:567

Problema:
No bloco `case MOVEMENT_FORWARD`, quando NAO ha obstaculo, falta `break;` e a execucao cai em `MOVEMENT_BACKWARDS`.

Impacto real:
- Durante movimento para frente, logica de compensacao de giroscopio da marcha re pode ser aplicada indevidamente.
- Pode causar comando de PWM inesperado, oscilacao de rumo e desvio lateral.

Correcao sugerida:
Adicionar `break;` ao final do `case MOVEMENT_FORWARD` fora do `if (d < 100)`.

## 2) CRITICO - Limite de giro acumulativo e sem reset robusto

Local:
- src/carrinho.ino:396
- src/carrinho.ino:489
- src/carrinho.ino:510
- src/carrinho.ino:522

Problema:
`turnLimitRad` comeca em 90 graus, mas nos comandos de giro ele eh somado/subtraido acumulativamente (`+=`/`-=`) e nao eh restaurado no `stop()`.

Impacto real:
- Giro de 90 graus pode virar 180, 270... dependendo do historico.
- Comando de giro deixa de ser deterministico.

Correcao sugerida:
Sempre recalcular `turnLimitRad` por comando, a partir de zero, e guardar sinal/direcao separadamente.

## 3) CRITICO - Funcao de normalizacao existe, mas nao e aplicada no fluxo de giro

Local:
- src/carrinho.ino:415
- src/carrinho.ino:584

Problema:
A funcao `_fixTurning()` nao e chamada em nenhum lugar.

Impacto real:
- `turningAngleZ` cresce sem normalizacao ao integrar `gyroZ * dt`.
- A partir de longos periodos pode haver degradacao numerica e comparacoes inconsistentes.

Correcao sugerida:
Normalizar apos cada integracao angular ou, preferencialmente, trabalhar com erro angular minimo (shortest angle error) em vez de comparar absolutos.

## 4) CRITICO - Comparacao angular por valor absoluto perde informacao de direcao e quebra em fronteira +/-PI

Local:
- src/carrinho.ino:597
- src/carrinho.ino:598
- src/carrinho.ino:610

Problema:
O controle usa `absTurn = fabs(turningAngleZ)` e `absTurnLimit = fabs(turnLimitRad)`.

Impacto real:
- Perto de +PI/-PI pode haver salto aparente no erro.
- O controle ignora o sinal do erro angular, favorecendo overshoot/undershoot e inversao tardia.

Correcao sugerida:
Calcular erro assinado:
`erro = normalizeAngle(targetHeading - currentHeading)`

E parar quando `fabs(erro) < tolerancia`.

## 5) ALTO - Formula de normalizacao fornecida no enunciado e incorreta para varios casos

Formula citada:
```cpp
turnLimitRad = fmod(turnLimitRad + 2 * M_PI, 2 * M_PI) - M_PI;
```

Problemas da formula:
- Se `turnLimitRad == 0`, resultado vira `-PI` (errado).
- Nao implementa corretamente o deslocamento para intervalo centrado em zero.
- Pode introduzir descontinuidade artificial.

Implementacao robusta recomendada:
```cpp
static inline float normalizeAnglePi(float a) {
    a = fmodf(a + M_PI, 2.0f * M_PI);
    if (a < 0.0f) a += 2.0f * M_PI;
    return a - M_PI; // intervalo [-PI, PI)
}
```

Alternativa ainda melhor (quando disponivel):
```cpp
static inline float normalizeAnglePi(float a) {
    return remainderf(a, 2.0f * M_PI); // aproxima para [-PI, PI]
}
```

## 6) ALTO - Classe PID escala sempre para [0..255], misturando unidades fisicas

Local:
- lib/PID/PID.cpp:31
- lib/PID/PID.cpp:43

Problema:
`PID::compute()` aplica clamp no dominio fisico e depois remapeia sempre para 0..255.

Impacto real:
- O mesmo PID vira "controlador em unidade fisica" e "gerador de PWM" ao mesmo tempo.
- Dificulta sintonia e gera comportamento contraintuitivo (ex.: controle angular reaproveitado com escala de PWM).

Correcao sugerida:
Separar camadas:
- PID retorna na unidade do erro (rad/s, rad, etc.).
- Conversao para PWM feita apenas na camada de atuacao.

## 7) ALTO - `targetRPM` telemetria inconsistente

Local:
- src/carrinho.ino:240
- src/carrinho.ino:274
- src/carrinho.ino:368

Problema:
`setTargetRPM()` altera `targetRadS`, mas `targetRPM` nao eh atualizado. `getTargetRPM()` retorna valor defasado.

Impacto real:
- Diagnostico via dashboard pode mostrar setpoint incorreto.
- Dificulta sintonia e depuracao de oscilacao.

Correcao sugerida:
Atualizar `targetRPM` dentro de `setTargetRPM()`.

## 8) ALTO - Leitura/reset de encoder sem secao critica completa

Local:
- lib/Encoder/Encoder.cpp:36
- lib/Encoder/Encoder.cpp:50
- lib/Encoder/Encoder.cpp:62

Problema:
`getRPM()` e `reset()` usam variaveis atualizadas em ISR sem proteger leitura + zeramento como operacao atomica unica.

Impacto real:
- Perda de pulsos em fronteiras de amostragem.
- Medicao de RPM ruidosa, podendo excitar PID (oscilar PWM).

Correcao sugerida:
- Criar funcao atomica para snapshot+reset de contadores.
- Evitar acesso intercalado entre loop e ISR sem travamento curto.

## 9) ALTO - ISR de encoder sem IRAM_ATTR e com digitalRead dentro da interrupcao

Local:
- lib/Encoder/Encoder.h:26
- lib/Encoder/Encoder.cpp:6
- lib/Encoder/Encoder.cpp:11

Problema:
No ESP32, ISR idealmente em IRAM e com caminho minimo. `digitalRead()` em ISR adiciona latencia/jitter.

Impacto real:
- Perda de pulsos em altas RPM.
- Maior jitter no controle de velocidade.

Correcao sugerida:
- Marcar ISR com `IRAM_ATTR`.
- Reduzir custo dentro da ISR (leitura direta GPIO ou logica simplificada).

## 10) ALTO - dt de controle nao eh estritamente deterministico

Local:
- src/carrinho.ino:542
- src/carrinho.ino:543
- src/carrinho.ino:594
- src/carrinho.ino:655

Problema:
`deltaTime` depende de `millis()`, mas ha operacoes bloqueantes e pesadas no mesmo loop (Wi-Fi reconnect com timeout, logs, websocket, JSON).

Impacto real:
- Integracao do gyro com passo variavel e jitter alto.
- Erro angular acumulado e resposta imprevisivel em curva.

Correcao sugerida:
- Usar timestamp em micros para integracao IMU.
- Isolar reconexao Wi-Fi em maquina de estados nao bloqueante.
- Reduzir logs no caminho critico.

## 11) MEDIO - Drift do giroscopio tratado apenas na primeira leitura

Local:
- src/carrinho.ino:201
- src/carrinho.ino:203
- src/carrinho.ino:215

Problema:
Bias do gyro eh calibrado so na primeira amostra.

Impacto real:
- Drift termico ao longo do tempo desloca heading.
- Giro de 90 graus perde repetibilidade apos alguns minutos.

Correcao sugerida:
Recalibracao adaptativa quando parado (janela estatistica + limiar de movimento) e filtro complementar (gyro + referencia externa quando disponivel).

## 12) MEDIO - Ordem de inicializacao pode acessar SPIFFS antes da montagem

Local:
- src/carrinho.ino:1080
- src/carrinho.ino:1189

Problema:
Servidor HTTP eh preparado antes de `SPIFFS.begin(true)`.

Impacto real:
- Requisicoes precoces podem falhar ou retornar erro indevido.

Correcao sugerida:
Montar SPIFFS antes de registrar rotas que dependem de arquivo.

## 13) MEDIO - Definicao de M_PI em Encoder como variavel global em header

Local:
- lib/Encoder/Encoder.h:6
- lib/Encoder/Encoder.h:7
- lib/Encoder/Encoder.cpp:3
- lib/Encoder/Encoder.cpp:4

Problema:
`unsigned long double M_PI = ...` em header cria simbolo global em cada unidade de compilacao.

Impacto real:
- Risco de multiplas definicoes/conflitos dependendo do toolchain.
- Sem necessidade (M_PI ja pode vir de math.h, ou use constexpr local).

Correcao sugerida:
Substituir por `constexpr` local no .cpp ou macro protegida.

## 14) MEDIO - Parametros de sentido do encoder declarados e nao aplicados

Local:
- src/carrinho.ino:65
- src/carrinho.ino:66
- lib/Encoder/Encoder.cpp:46

Problema:
Constantes de direcao existem, mas `setForwardClockwise()` nao eh chamada para os encoders.

Impacto real:
- Sinal de ticks pode ficar invertido em algum motor dependendo da montagem.
- Odometria e controle direcional podem ficar inconsistentes.

Correcao sugerida:
Aplicar explicitamente no setup de cada encoder.

## 15) MEDIO - Mudanca de frequencia de CPU afeta suposicoes de temporizacao

Local:
- lib/PowerManager/PowerManager.cpp:28
- lib/PowerManager/PowerManager.cpp:39
- src/carrinho.ino:1238

Problema:
PowerManager alterna modo de energia e frequencia CPU enquanto controle usa dt fixo em PID interno.

Impacto real:
- Se periodos reais variarem, ganho efetivo muda.
- Pode piorar estabilidade em borda.

Correcao sugerida:
Controlador com dt medido real, ou manter frequencia constante durante movimento.

## Sugestoes de implementacao (funcoes melhoradas)

## A) Normalizacao de angulo robusta

```cpp
static inline float normalizeAnglePi(float a) {
    a = fmodf(a + M_PI, 2.0f * M_PI);
    if (a < 0.0f) a += 2.0f * M_PI;
    return a - M_PI; // [-PI, PI)
}

static inline float shortestAngleError(float target, float current) {
    return normalizeAnglePi(target - current);
}
```

## B) Integracao de heading mais estavel (gyro Z)

```cpp
struct HeadingIntegrator {
    float heading = 0.0f;
    float gyroBias = 0.0f;
    float lastGz = 0.0f;
    uint32_t lastUs = 0;

    void reset(float heading0 = 0.0f) {
        heading = normalizeAnglePi(heading0);
        lastUs = micros();
        lastGz = 0.0f;
    }

    void update(float gzRaw, bool isStopped) {
        uint32_t nowUs = micros();
        float dt = (nowUs - lastUs) * 1e-6f;
        lastUs = nowUs;

        // Protege contra spikes de tempo (Wi-Fi/OTA/log)
        if (dt <= 0.0f || dt > 0.05f) dt = 0.02f;

        // Recalibracao lenta de bias apenas parado
        if (isStopped) {
            const float alpha = 0.01f;
            gyroBias = (1.0f - alpha) * gyroBias + alpha * gzRaw;
        }

        float gz = gzRaw - gyroBias;

        // Integracao trapezoidal reduz ruido de discretizacao
        heading = normalizeAnglePi(heading + 0.5f * (gz + lastGz) * dt);
        lastGz = gz;
    }
};
```

## C) Controle de giro por erro angular assinado

```cpp
// Exemplo conceitual
float err = shortestAngleError(targetHeading, currentHeading);

if (fabsf(err) < 0.03f) { // ~1.7 graus
    stop();
} else {
    // PID deve operar em unidade angular (nao PWM direto)
    float wCmd = pidAngle.compute(0.0f, -err);

    // converte velocidade angular desejada para setpoint de roda
    float base = 40.0f; // rpm minimo para vencer atrito
    float add = constrain(fabsf(wCmd), 0.0f, 80.0f);
    float rpm = base + add;

    if (err > 0.0f) {
        // virar para um lado
        motorRight->forward();
        motorLeft->backward();
    } else {
        motorRight->backward();
        motorLeft->forward();
    }

    motorRight->setTargetRPM(rpm);
    motorLeft->setTargetRPM(rpm);
}
```

## D) Snapshot atomico de encoder para RPM

```cpp
// Na classe Encoder
long readAndResetPulses() {
    noInterrupts();
    const long pulses = (countA + countB) / 2;
    countA = 0;
    countB = 0;
    interrupts();
    return pulses;
}
```

## Checklist rapido para robo real (seguranca e estabilidade)

- Corrigir fall-through do `MOVEMENT_FORWARD`.
- Tornar alvo de giro deterministico por comando (sem acumulacao historica).
- Usar erro angular assinado e normalizado para controle.
- Garantir leitura atomica de encoder e ISR enxuta.
- Remover bloqueios longos do loop de controle.
- Recalibrar bias do gyro quando parado.
- Verificar saturacao de atuacao e anti-windup coerente por unidade fisica.

## Conclusao

No estado atual, o sistema pode funcionar em demonstracoes simples, mas ainda possui riscos de instabilidade dinamica para operacao robusta em robo real, principalmente em manobras de giro e em condicoes de jitter temporal. As correcoes acima sao diretas e elevam significativamente a previsibilidade do controle.
