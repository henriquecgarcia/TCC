# PROBLEMAS2.md — Segunda Análise do Workspace

> Análise complementar ao `PROBLEMAS.md`. Todos os itens abaixo são problemas **ainda não cobertos** (ou não totalmente descritos) na análise anterior.

---

## CRÍTICO

---

### P2-01 — Correção do giroscópio nunca é aplicada; domínios PWM e rad/s misturados

**Localização:** `src/carrinho.ino` — `PonteH::loop()`, case `MOVEMENT_BACKWARDS` (e fall-through de `MOVEMENT_FORWARD`)

**Descrição:**

```cpp
double gz = mpuSensor->getGyroscopeZ();
if (nextRight) {
    float gyroCorrection = pidGyro.compute(gz, 0.0);
    lastGyroZ = pidGyro.scaleToPWM(gyroCorrection); // salva como PWM [0..255]
} else {
    gz = lastGyroZ;  // usa o "valor salvo"
}
doUpdate(motorRight,  gz);   // <-- passa gz, NÃO gyroCorrection nem lastGyroZ
doUpdate(motorLeft,  -gz);
```

**Problema 1 — ciclo `nextRight = true`:**  
`gyroCorrection` (saída do PID, domínio ±0.5 rad/s) é calculada e convertida para PWM em `lastGyroZ`, mas o resultado é **ignorado**. O valor bruto `gz` do giroscópio (também em rad/s) é passado diretamente a `Motor::update()` como `gyroError`, contornando o PID completamente.

**Problema 2 — ciclo `nextRight = false`:**  
`gz` é sobrescrito com `lastGyroZ`, que contém um valor em escala PWM [0..255] (calculado por `scaleToPWM`). Esse valor é então passado como `gyroError` a `Motor::update()`, onde é **somado** à velocidade-alvo em rad/s:

```cpp
float targetRadS = this->targetRadS + gyroError;
// ex.: 10.47 rad/s + (-127.5 PWM) ≈ -117 rad/s
```

Isso resulta em target negativo extremo, zerando ou travando o motor naquele ciclo. Em seguida, o guard de target baixo pode disparar (ver P2-02).

**Impacto:** A correção de trajetória reta nunca funcionará corretamente. No ciclo "ímpar" o PID é ignorado; no ciclo "par" o motor recebe um valor absurdamente fora de escala.

**Correção sugerida:** Usar `gyroCorrection` diretamente (sem `scaleToPWM`) como `gyroError` dos motores, e passar o mesmo valor para ambos os motores (direito positivo, esquerdo negativo):

```cpp
double gz = mpuSensor->getGyroscopeZ();
float gyroCorrection = pidGyro.compute(gz, 0.0);
doUpdate(motorRight,  gyroCorrection);
doUpdate(motorLeft,  -gyroCorrection);
```

---

### P2-02 — Variável local `targetRadS` em `Motor::update()` sombreia o membro, disparando reset incorreto

**Localização:** `src/carrinho.ino` — `Motor::update()`

```cpp
void update(double gyroError = 0.0) {
    // ...
    float targetRadS  = this->targetRadS + gyroError;  // variável LOCAL
    float currentRadS = rpmToRadS(currentRPM);
    float rpmControl  = pidRPM.compute(targetRadS, currentRadS);
    // ...
    if (targetRadS < 0.01) {           // checa a variável LOCAL
        webLog("... resetting to 100 RPM\n");
        this->targetRadS = rpmToRadS(100.0);  // modifica o MEMBRO
    }
}
```

**Descrição:**  
A variável local `targetRadS` (tipo `float`) é declarada com o mesmo nome do membro `this->targetRadS` (tipo `double`). Quando o bug P2-01 ocorre no ciclo `nextRight = false`, `gyroError` recebe um valor PWM (~127.5), resultando em `targetRadS` (local) ≈ −117 rad/s, que satisfaz `< 0.01`. O guard dispara e **força `this->targetRadS` de volta para 100 RPM** depois que o `compute()` já rodou com o target errado — ou seja, o reset não evita o dano, ele apenas mascara o estado para o próximo ciclo.

Mesmo sem o bug P2-01, o guard é logicamente perigoso: se `gyroError` legítimo for negativo e maior que `this->targetRadS`, o motor em movimento teria seu target arbitrariamente sobrescrito para 100 RPM.

**Correção sugerida:** Renomear a variável local (ex.: `float effectiveTargetRadS`) e checar `this->targetRadS` no guard:

```cpp
float effectiveTargetRadS = static_cast<float>(this->targetRadS) + static_cast<float>(gyroError);
// ...
if (this->targetRadS < 0.01) { /* motor realmente parado */ }
```

---

## ALTO

---

### P2-03 — `pidTurn` declarado, inicializado e resetado, mas `compute()` nunca é chamado

**Localização:** `src/carrinho.ino` — `PonteH` (membro privado + casos TURN na `loop()`)

```cpp
PID pidTurn = PID(100.0, 5.0, 0.5, controlInterval / 1000.0);
```

O `pidTurn` é resetado em `turnLeft()`, `turnRight()` e `stop()`, mas no corpo da `loop()` ele nunca chama `compute()`. As curvas são controladas apenas pela integração bruta de `turningAngleZ` e por um threshold fixo (`absTurn >= absTurnLimit`). O PID de curva existe em comentário e estrutura mas é código morto.

**Impacto:** Curvas não têm controle de velocidade diferencial via PID; a parada no ângulo certo depende inteiramente do timing do loop e da integração numérica do giroscópio, sem realimentação.

---

### P2-04 — `_fixTurning()` definida mas nunca chamada (normalização de ângulos nunca ocorre)

**Localização:** `src/carrinho.ino` — `PonteH::_fixTurning()` (~linha 420)

```cpp
void _fixTurning() {
    turnLimitRad = normalizeAngle(turnLimitRad);
    turningAngleZ = normalizeAngle(turningAngleZ);
}
```

A função garante que `turnLimitRad` e `turningAngleZ` estejam no intervalo [−π, π], prevenindo comparações ruins quando valores acumulam além de ±π. Ela nunca é chamada no loop de curva, onde esses valores são ativamente usados.

**Impacto:** Após acumulação de vários giros, `turningAngleZ` pode exceder ±π sem passar por normalização, fazendo com que a comparação `absTurn >= absTurnLimit` falhe ou funcione de forma imprevisível.

---

### P2-05 — `PID::scaleToPWM()` divide por zero se `top_celing == bottom_floor`

**Localização:** `lib/PID/PID.cpp` — método `scaleToPWM()`

```cpp
float PID::scaleToPWM(float output) {
    float scaled = (output - bottom_floor) * (255.0f / (top_celing - bottom_floor));
    //                                                   ^^^^^^^^^^^^^^^^^^^^^^^^^^
    //                                                   denominador pode ser zero
    return clamp(scaled, 0.0f, 255.0f);
}
```

Se `setMaxMin()` for chamado com os dois limites iguais (ou se nunca for chamado e os padrões coincidirem), a divisão resulta em `Inf` ou `NaN`, que propagam para o `analogWrite()` como valores inválidos.

**Correção sugerida:**
```cpp
float PID::scaleToPWM(float output) {
    const float range = top_celing - bottom_floor;
    if (fabsf(range) < 1e-6f) return 0.0f;
    return clamp((output - bottom_floor) * (255.0f / range), 0.0f, 255.0f);
}
```

---

### P2-06 — `last_sent_percent` nunca é atualizado no callback de progresso OTA

**Localização:** `src/carrinho.ino` — `init_ota()` → `ArduinoOTA.onProgress(...)`

```cpp
int last_sent_percent = 0;  // variável global

ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
    int current_percent = (progress / (total / 100));
    // ...
    if (current_percent == last_sent_percent)
        Serial.print(logMessage);
    else
        webLog(logMessage);
    // last_sent_percent NUNCA é atualizado aqui!
});
```

`last_sent_percent` começa em 0 e permanece 0 para sempre. A partir do primeiro callback (quando `current_percent` já é ≥ 1), a condição `current_percent == last_sent_percent` é **sempre falsa**, portanto `webLog()` é chamado **em toda invocação**. O Serial nunca recebe mensagens de progresso. O WebSocket recebe uma mensagem por porcentagem (± 100 mensagens por upload), potencialmente travando clientes lentos.

**Correção sugerida:** Atualizar `last_sent_percent = current_percent;` ao final do callback. A condição provavelmente também está invertida: deveria enviar ao WebSocket apenas **quando mudar** e ao Serial em todos os casos (ou vice-versa).

---

## MÉDIO

---

### P2-07 — PID usa `_dt` fixo de construção; ignora o tempo real transcorrido

**Localização:** `lib/PID/PID.cpp` — `compute()` + todos os construtores de PID em `carrinho.ino`

```cpp
float PID::compute(float setpoint, float measurement) {
    float error = setpoint - measurement;
    _integral += error * _dt;          // _dt fixo
    float derivative = (error - _prevError) / _dt;  // _dt fixo
    // ...
}
```

O `_dt` é definido na construção (`thinkInterval/1000.0f = 0.5s` para o PID de RPM; `controlInterval/1000.0f = 0.1s` para os PIDs da PonteH). Na prática, o loop principal tem jitter, WiFi e OTA competem por tempo, e o `thinkInterval` no Motor é um guard de mínimo — não garante exatamente 500 ms entre chamadas.

**Impacto:** O termo integral acumula a uma taxa incorreta; o termo derivativo é calculado com denominador errado. Isso se agrava se `powerManager` alterar a frequência de CPU (80/160/240 MHz), mudando efetivamente o tempo de execução sem atualizar `_dt`.

**Correção sugerida:** Passar o `deltaTime` real medido como parâmetro de `compute()`, ou medir internamente com `millis()`.

---

### P2-08 — Calibração do MPU6050 usa uma única amostra na primeira leitura

**Localização:** `src/carrinho.ino` — `MPU6050::loop()`

```cpp
if (firstRead) {
    offsetX = g.gyro.x;
    offsetY = g.gyro.y;
    offsetZ = g.gyro.z;
    firstRead = false;
    // idem para acelerômetro
}
```

O sensor IMU precisa de alguns milissegundos para estabilizar após ligar. A primeira leitura pode conter ruído de inicialização ou valores transientes. Um único sample define permanentemente o offset de todas as leituras subsequentes.

**Impacto:** Se a primeira leitura for ruidosa (comum logo após `Wire.begin()`), o offset ficará enviesado e o giroscópio reportará drift contínuo mesmo com o robô parado, afetando diretamente a integração de ângulo para as curvas.

**Correção sugerida:** Coletar *N* amostras (ex.: 50) com pequeno atraso entre elas e usar a média como offset.

---

### P2-09 — `sensor->loop()` chamado duas vezes por ciclo quando o carro vai para frente

**Localização:** `src/carrinho.ino` — `PonteH::loop()` (case `MOVEMENT_FORWARD`) e `sendTelemetryFrame()`

```cpp
// Em PonteH::loop():
int d = frontDistSensor->loop();  // 1ª chamada → faz medição
if (d < 100) { stop(); break; }

// Em sendTelemetryFrame() (chamada logo abaixo na mesma iteração):
doc["distance"] = sensor->loop();  // 2ª chamada → nova medição
```

`VL53L0X::loop()` atualiza estado interno (contagem de erros consecutivos, cache do valor). Duas chamadas na mesma iteração de 100 ms podem fazer o sensor iniciar duas medições sequenciais sem aguardar o tempo de integração completo, retornando valores inconsistentes ou incrementando contadores de erro incorretamente.

Adicionalmente, a checagem de obstáculo `d < 100` captura tanto "obstáculo a menos de 100 mm" quanto **retornos de erro negativos** do sensor (ex.: timeout retorna −1), causando parada falsa.

**Correção:** Cachear o resultado da primeira chamada e reutilizá-lo na telemetria. Checar distância com `d > 0 && d < 100`.

---

### P2-10 — `ConnectToWiFi()` bloqueia o loop principal por até 5 segundos

**Localização:** `src/carrinho.ino` — `loop()` e `ConnectToWiFi()`

```cpp
void loop() {
    ArduinoOTA.handle();
    // ...
    if (WiFi.status() != WL_CONNECTED) {
        ponte->stop();
        ConnectToWiFi(5000);  // loop com delay(100) por até 5 s
        return;
    }
    // ...
}
```

Durante a tentativa de reconexão, o loop inteiro para: sensores não são lidos, odometria não é atualizada, e após o `return` o OTA recebe apenas uma chamada antes do próximo ciclo de reconexão. Se o WiFi oscilar, o robô fica paralisado por múltiplos períodos de 5 s.

**Impacto:** Perda de odometria durante reconexão; impossibilidade de atualização OTA se WiFi cair durante um upload parcial.

---

### P2-11 — Comentários de pinagem no cabeçalho do arquivo estão errados

**Localização:** `src/carrinho.ino` — comentário do cabeçalho (linhas 1–35 aprox.)

| O que o comentário diz | O que o código realmente usa |
|---|---|
| Motor direito PWM → pino 14 | `Motor(12, 13, **32**, encoderD, ...)` → PWM = 32 |
| Motor esquerdo IN1 → pino 33 | `Motor(**26**, 25, 33, encoderE, ...)` → IN1 = 26 |
| Encoder direito → pinos 26, 27 | `new Encoder(15, 4)` → CH A=4, CH B=15 |
| Encoder esquerdo → pinos 35, 34 | `new Encoder(16, 17)` → CH A=17, CH B=16 |

Todos os pinos dos motores e encoders descritos no cabeçalho diferem dos valores reais no código. O comentário provavelmente documenta uma versão anterior do hardware.

**Impacto:** Qualquer pessoa montando o circuito com base no comentário conectará os fios errados, podendo danificar o hardware ou causar comportamento imprevisível.

---

### P2-12 — `DynamicJsonDocument(16384)` alocado no heap a cada chamada de `getMapSnapshotJson()`

**Localização:** `src/carrinho.ino` — `getMapSnapshotJson()`

```cpp
DynamicJsonDocument doc(16384);
```

Esta função é chamada ao conectar cada cliente WebSocket, em resposta a `path_to`, ao comando `map`, e após cálculo de rota. Cada chamada aloca e libera 16 KB no heap dinâmico. No ESP32, alocações repetidas de blocos grandes fragmentam o heap; após muitas chamadas, `malloc` pode falhar mesmo com memória livre suficiente (por falta de bloco contíguo).

**Sugestão:** Usar `StaticJsonDocument` com tamanho adequado, ou manter uma instância estática reutilizável, limpando-a com `doc.clear()` antes de cada uso.

---

## Referência rápida

| ID | Severidade | Resumo |
|---|---|---|
| P2-01 | CRÍTICO | Gyro correction usa gz bruto e mistura domínios PWM/rad/s |
| P2-02 | CRÍTICO | Local `targetRadS` sombreia membro; reset incorreto para 100 RPM |
| P2-03 | ALTO | `pidTurn` é dead code — `compute()` nunca chamado |
| P2-04 | ALTO | `_fixTurning()` definida mas nunca chamada |
| P2-05 | ALTO | `scaleToPWM()` divide por zero se limites iguais |
| P2-06 | ALTO | `last_sent_percent` nunca atualizado; WebSocket inundado durante OTA |
| P2-07 | MÉDIO | PID com `_dt` fixo ignora tempo real transcorrido |
| P2-08 | MÉDIO | Calibração MPU6050 de uma amostra; offset ruidoso |
| P2-09 | MÉDIO | `sensor->loop()` chamado duas vezes por ciclo em FORWARD |
| P2-10 | MÉDIO | `ConnectToWiFi()` bloqueia loop por até 5 s |
| P2-11 | MÉDIO | Comentários de pinagem no cabeçalho completamente errados |
| P2-12 | MÉDIO | `DynamicJsonDocument(16384)` fragmenta heap em chamadas frequentes |
