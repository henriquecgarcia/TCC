# PoseEKF - Extended Kalman Filter para Estimação de Pose

## Visão Geral

O **PoseEKF** é um Filtro de Kalman Estendido implementado para estimar a pose (x, y, θ) do robô com melhor precisão do que a odometria pura. Ele combina:

- **Odometria (encoders)**: Predição da posição usando movimento dos motores
- **Magnetômetro (QMC5883L)**: Correção contínua da orientação (θ)
- **Sensor de Distância (VL53L0X)**: Detecção de obstáculos (expansível para updates de posição)

## Arquitetura

### Estado
```
x[k] = [x, y, θ]^T

onde:
- x, y: posição em metros (sistema global)
- θ: orientação em radianos (0 = leste, π/2 = norte)
```

### Ciclo EKF

```
1. PREDIÇÃO (predict)
   - Lê ticks dos encoders
   - Calcula distância percorrida
   - Atualiza posição usando cinemática diferencial
   - Propaga covariância P através Jacobiano F

2. ATUALIZAÇÃO (updateWithGyro)
   - Lê taxa angular do magnetômetro
   - Integra para obter θ_medido
   - Computa Ganho de Kalman
   - Corrige estado e reduz covariância

3. ATUALIZAÇÃO (updateWithDistance)
   - Detecta obstáculos próximos
   - Aplica correção adaptativa (em desenvolvimento)
```

## Parâmetros de Configuração

### Ruído do Processo (Q)
Define a incerteza do modelo de movimento. Valores maiores = menos confiança na predição.

```cpp
void setProcessNoise(
    float qx,       // Incerteza em X (padrão: 0.005)
    float qy,       // Incerteza em Y (padrão: 0.005)
    float qTheta    // Incerteza em θ (padrão: 0.01)
);
```

### Ruído da Medição (R)
Define a confiabilidade dos sensores. Valores maiores = menos confiança na medição.

```cpp
void setMeasurementNoise(
    float rTheta,      // Incerteza do magnetômetro (padrão: 0.02)
    float rDistance,   // Incerteza de distância (padrão: 0.05)
    float rDrift       // Incerteza de drift (padrão: 0.01)
);
```

**Padrões recomendados:**

| Cenário | qx | qy | qTheta | rTheta | rDistance | rDrift |
|---------|----|----|--------|--------|-----------|--------|
| Terreno Liso | 0.003 | 0.003 | 0.005 | 0.01 | 0.03 | 0.005 |
| Terreno Rugoso | 0.01 | 0.01 | 0.02 | 0.05 | 0.1 | 0.02 |
| Corredor Estreito | 0.005 | 0.005 | 0.01 | 0.02 | 0.05 | 0.01 |

## Uso no Código

### Inicialização

```cpp
// Criar instância com parâmetros do robô
PoseEKF* ekf = new PoseEKF(
    0.0325f,  // wheelRadius (m)
    0.1400f,  // wheelBase (m)
    0.05f,    // mapCellSize (m)
    20.0f     // encoderTicksPerRev
);

// Inicializar no setup
ekf->initialize(
    0.0f, 0.0f, 0.0f,               // x0, y0, θ0
    encoderE->getTotalTicks(),       // leftTicks0
    encoderD->getTotalTicks()        // rightTicks0
);
```

### No Loop

```cpp
// Ciclo de controle (ex: a cada 20ms)
float deltaTime_s = 0.020f;

// PREDIÇÃO: usar odometria
ekf->predict(
    encoderE->getTotalTicks(),
    encoderD->getTotalTicks(),
    deltaTime_s
);

// ATUALIZAÇÃO: usar magnetômetro para orientação
ekf->updateWithGyro(
    sensorQMC->getAngularVelocityZRadS(),
    deltaTime_s
);

// ATUALIZAÇÃO: usar sensor de distância (opcional)
ekf->updateWithDistance(
    sensor->getDistance_mm(),
    distance_mm < 150  // obstácleDetected
);
```

### Leitura de Pose

```cpp
float x = ekf->getX();           // posição X (m)
float y = ekf->getY();           // posição Y (m)
float theta = ekf->getTheta();   // orientação (rad)

// Incerteza (confiança baixa = covariância alta)
float cov_x = ekf->getCovariance_X();
float cov_y = ekf->getCovariance_Y();
float cov_theta = ekf->getCovariance_Theta();

// Usar confiança para tomada de decisão
if (cov_theta < 0.05f) {
    // Confiança alta em θ, usar para planejamento de trajetória
}
```

## Dados Telemétricos (JSON)

O sistema envia dados do EKF via WebSocket:

```json
{
  "ekf": {
    "x": 1.234,
    "y": 0.567,
    "theta": 0.785,
    "cov_x": 0.015,
    "cov_y": 0.015,
    "cov_theta": 0.008
  },
  "odometry": {
    "x": 1.240,
    "y": 0.560,
    "theta": 0.780,
    "mapX": 24,
    "mapY": 11
  }
}
```

**Comparação:**
- `odometry`: Estimativa não-filtrada (acumula erro com o tempo)
- `ekf`: Estimativa filtrada (mais precisa e com confiança quantificada)

## Tuning de Desempenho

### Cenário: Drift do Magnetômetro Muito Alto

```cpp
ekf->setMeasurementNoise(
    0.01f,   // Reduz rTheta (confia mais no gyro)
    0.05f,
    0.01f
);
```

### Cenário: Magnetômetro Muito Ruidoso

```cpp
ekf->setMeasurementNoise(
    0.05f,   // Aumenta rTheta (confia menos no gyro)
    0.05f,
    0.01f
);
```

### Cenário: Perda Frequente de Contato com Solo

```cpp
ekf->setProcessNoise(
    0.02f,   // Aumenta Q (predição menos confiável)
    0.02f,
    0.05f
);
```

## Matemática

### Modelo de Movimento (Predição)

Para um robô diferencial com rodas independentes:

```
d_left  = ticks_left / ticksPerRev × 2π × r_wheel
d_right = ticks_right / ticksPerRev × 2π × r_wheel

d_avg = (d_left + d_right) / 2
Δθ = (d_right - d_left) / wheelBase

θ_mid = θ_anterior + Δθ / 2

Δx = d_avg × cos(θ_mid)
Δy = d_avg × sin(θ_mid)

x_novo = x + Δx
y_novo = y + Δy
θ_novo = θ + Δθ
```

### Jacobiano (Linearização)

```
F = ∂f/∂x |_x_pred

P_novo = F × P × F^T + Q
```

### Atualização com Medição

```
H = matriz de observação (ex: [0 0 1] para θ)
innovation = z_medido - h(x_predito)
S = H × P × H^T + R

K = P × H^T / S
x = x + K × innovation
P = (I - K × H) × P
```

## Limitações Conhecidas

1. **Observabilidade Limitada**: Não há atualização de posição absoluta (x, y) - apenas de orientação
   - Solução futura: adicionar câmera ou marcadores

2. **Não-linearidades**: Comportamento não-linear em curvas fechadas
   - Mitigado com Jacobiano na predição

3. **Hiperparâmetros Críticos**: Qualidade depende muito de Q e R
   - Solução: usar adaptive EKF com estima de ruído online

## Extensões Futuras

- [ ] Atualização com câmera (visual odometry)
- [ ] Atualização com marcadores de referência (ArUco, QR codes)
- [ ] Adaptive noise estimation
- [ ] Unscented Kalman Filter (UKF) para melhor linearização
- [ ] Mapa de landmarks com SLAM

## Referências

- Thrun, S., Burgard, W., & Fox, D. (2005). Probabilistic Robotics.
- Simon, D. (2006). Optimal State Estimation: Kalman, H-infinity, and Nonlinear Approaches.
- Solá, J. (2014). "Quaternion kinematics for the error-state Kalman filter"
