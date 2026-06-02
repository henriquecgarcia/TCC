#include "PoseEKF.h"

PoseEKF::PoseEKF(
    float wheelRadius_m,
    float wheelBase_m,
    float mapCellSize_m,
    float encoderTicksPerRev_val
) : wheelRadius(wheelRadius_m),
    wheelBase(wheelBase_m),
    mapCellSize(mapCellSize_m),
    encoderTicksPerRev(encoderTicksPerRev_val),
    lastLeftTicks(0),
    lastRightTicks(0),
    lastTheta(0.0f),
    lastUpdateTime(0) {

    // Inicializa estado
    state[0] = 0.0f;  // x
    state[1] = 0.0f;  // y
    state[2] = 0.0f;  // theta

    // Inicializa covariância (incerteza inicial alta)
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = (i == j) ? 0.1f : 0.0f;  // Matriz diagonal inicial
        }
    }

    // Ruído do processo (odometria) - padrão
    setProcessNoise(0.005f, 0.005f, 0.01f);

    // Ruído da medição (sensores) - padrão
    setMeasurementNoise(0.02f, 0.05f, 0.01f);
}

void PoseEKF::initialize(
    float x0,
    float y0,
    float theta0,
    long leftTicks0,
    long rightTicks0
) {
    state[0] = x0;
    state[1] = y0;
    state[2] = theta0;

    lastLeftTicks = leftTicks0;
    lastRightTicks = rightTicks0;
    lastTheta = theta0;
    lastUpdateTime = millis();

    // Reseta covariância a um nível moderado
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            P[i][j] = (i == j) ? 0.1f : 0.0f;
        }
    }
}

void PoseEKF::setProcessNoise(float qx, float qy, float qTheta) {
    Q[0][0] = qx;
    Q[0][1] = 0.0f;
    Q[0][2] = 0.0f;
    Q[1][0] = 0.0f;
    Q[1][1] = qy;
    Q[1][2] = 0.0f;
    Q[2][0] = 0.0f;
    Q[2][1] = 0.0f;
    Q[2][2] = qTheta;
}

void PoseEKF::setMeasurementNoise(float rTheta, float rDistance, float rDrift) {
    R[0][0] = rTheta;
    R[0][1] = 0.0f;
    R[0][2] = 0.0f;
    R[1][0] = 0.0f;
    R[1][1] = rDistance;
    R[1][2] = 0.0f;
    R[2][0] = 0.0f;
    R[2][1] = 0.0f;
    R[2][2] = rDrift;
}

void PoseEKF::normalizeAngle(float& angle) {
    while (angle > M_PI) angle -= 2.0f * M_PI;
    while (angle < -M_PI) angle += 2.0f * M_PI;
}

float PoseEKF::ticksToDistance(long tickDelta) {
    // Converte ticks para distância linear
    // distância = (ticks / ticksPerRev) * (2 * π * raioRoda)
    return (float)tickDelta / encoderTicksPerRev * (2.0f * M_PI * wheelRadius);
}

float PoseEKF::computeTheta(float leftDist, float rightDist) {
    // Diferença na distância / distância entre rodas = ângulo girado
    // dTheta = (rightDist - leftDist) / wheelBase
    return (rightDist - leftDist) / wheelBase;
}

void PoseEKF::predict(long leftTicks, long rightTicks, float deltaTime_s) {
    // Calcula incrementos de ticks
    long deltaLeftTicks = leftTicks - lastLeftTicks;
    long deltaRightTicks = rightTicks - lastRightTicks;

    // Converte para distâncias
    float leftDist = ticksToDistance(deltaLeftTicks);
    float rightDist = ticksToDistance(deltaRightTicks);

    // Distância média percorrida
    float avgDist = (leftDist + rightDist) / 2.0f;

    // Mudança de ângulo
    float deltaTheta = computeTheta(leftDist, rightDist);

    // Modelo cinemático simples (diferencial drive)
    float theta_mid = state[2] + deltaTheta / 2.0f;
    normalizeAngle(theta_mid);

    // Atualiza pose
    state[0] += avgDist * cosf(theta_mid);
    state[1] += avgDist * sinf(theta_mid);
    state[2] += deltaTheta;
    normalizeAngle(state[2]);

    // Atualiza últimos ticks
    lastLeftTicks = leftTicks;
    lastRightTicks = rightTicks;

    // ===== PROPAGAÇÃO DE COVARIÂNCIA =====
    // Jacobianos simplificados do modelo de movimento
    // F = matriz Jacobiana de transição de estado
    float F[3][3] = {
        {1.0f, 0.0f, -avgDist * sinf(theta_mid)},
        {0.0f, 1.0f,  avgDist * cosf(theta_mid)},
        {0.0f, 0.0f,  1.0f}
    };

    // P = F * P * F^T + Q
    float FP[3][3], FPFt[3][3], Ft[3][3];

    matTranspose3x3(F, Ft);
    matMultiply3x3(F, P, FP);
    matMultiply3x3(FP, Ft, FPFt);
    matAdd3x3(FPFt, Q, P);

    // Garante que P seja semi-definida positiva (diagonal não negativa)
    for (int i = 0; i < 3; i++) {
        if (P[i][i] < 0.0f) P[i][i] = 0.0f;
    }
}

void PoseEKF::updateWithGyro(float gyroZ_rad_s, float deltaTime_s) {
    // Integra giroscópio para obter theta
    float measuredDeltaTheta = gyroZ_rad_s * deltaTime_s;
    float measuredTheta = lastTheta + measuredDeltaTheta;
    normalizeAngle(measuredTheta);

    // Inova (diferença entre medição e predição)
    float innovation = measuredTheta - state[2];
    normalizeAngle(innovation);

    // S = H * P * H^T + R (onde H é a matriz de observação)
    // Para theta, H = [0 0 1]
    float S = P[2][2] + R[0][0];

    if (S > 0.0001f) {
        // Ganho de Kalman: K = P * H^T / S
        float K[3];
        K[0] = P[0][2] / S;
        K[1] = P[1][2] / S;
        K[2] = P[2][2] / S;

        // Atualiza estado
        state[0] += K[0] * innovation;
        state[1] += K[1] * innovation;
        state[2] += K[2] * innovation;
        normalizeAngle(state[2]);

        // Atualiza covariância: P = (I - K * H) * P
        // onde H = [0 0 1]
        float newP[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                newP[i][j] = P[i][j];
                if (j == 2) {
                    newP[i][j] -= K[i] * P[2][j];
                }
            }
        }

        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                P[i][j] = newP[i][j];
            }
        }
    }

    lastTheta = measuredTheta;
}

void PoseEKF::holdCurrentPose(long leftTicksNow, long rightTicksNow) {
    // Não altera x, y ou theta. Apenas evita que ticks acumulados durante
    // a parada sejam interpretados como deslocamento no próximo predict().
    lastLeftTicks = leftTicksNow;
    lastRightTicks = rightTicksNow;
    lastTheta = state[2];
    lastUpdateTime = millis();
}

void PoseEKF::updateWithDistance(float distance_mm, bool obstacleDetected) {
    if (!obstacleDetected) {
        return;  // Sem atualização se nenhum obstáculo
    }

    // Se obstáculo detectado próximo, pode ajustar posição
    // Este é um exemplo simplificado - em aplicações reais seria mais sofisticado
    float distance_m = distance_mm / 1000.0f;

    // Se distância é muito pequena (obstáculo imediato)
    if (distance_m < 0.15f) {
        // Pode aplicar uma atualização leve
        // Por enquanto, apenas aumenta a incerteza (cautela)
        P[0][0] *= 1.05f;
        P[1][1] *= 1.05f;
    }
}

// ===== FUNÇÕES AUXILIARES DE ÁLGEBRA LINEAR =====

void PoseEKF::matMultiply3x3(const float a[3][3], const float b[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = 0.0f;
            for (int k = 0; k < 3; k++) {
                result[i][j] += a[i][k] * b[k][j];
            }
        }
    }
}

void PoseEKF::matMultiply3x1(const float a[3][3], const float b[3], float result[3]) {
    for (int i = 0; i < 3; i++) {
        result[i] = 0.0f;
        for (int j = 0; j < 3; j++) {
            result[i] += a[i][j] * b[j];
        }
    }
}

void PoseEKF::matTranspose3x3(const float a[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = a[j][i];
        }
    }
}

void PoseEKF::matAdd3x3(const float a[3][3], const float b[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = a[i][j] + b[i][j];
        }
    }
}

void PoseEKF::matSubtract3x3(const float a[3][3], const float b[3][3], float result[3][3]) {
    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            result[i][j] = a[i][j] - b[i][j];
        }
    }
}

void PoseEKF::matInverse3x3(const float a[3][3], float result[3][3]) {
    // Calcula inversa de matriz 3x3 por método analítico
    float det = a[0][0] * (a[1][1] * a[2][2] - a[1][2] * a[2][1])
              - a[0][1] * (a[1][0] * a[2][2] - a[1][2] * a[2][0])
              + a[0][2] * (a[1][0] * a[2][1] - a[1][1] * a[2][0]);

    if (fabs(det) < 0.0001f) {
        // Matriz singular, retorna identidade
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                result[i][j] = (i == j) ? 1.0f : 0.0f;
            }
        }
        return;
    }

    float invDet = 1.0f / det;

    result[0][0] = (a[1][1] * a[2][2] - a[1][2] * a[2][1]) * invDet;
    result[0][1] = (a[0][2] * a[2][1] - a[0][1] * a[2][2]) * invDet;
    result[0][2] = (a[0][1] * a[1][2] - a[0][2] * a[1][1]) * invDet;

    result[1][0] = (a[1][2] * a[2][0] - a[1][0] * a[2][2]) * invDet;
    result[1][1] = (a[0][0] * a[2][2] - a[0][2] * a[2][0]) * invDet;
    result[1][2] = (a[0][2] * a[1][0] - a[0][0] * a[1][2]) * invDet;

    result[2][0] = (a[1][0] * a[2][1] - a[1][1] * a[2][0]) * invDet;
    result[2][1] = (a[0][1] * a[2][0] - a[0][0] * a[2][1]) * invDet;
    result[2][2] = (a[0][0] * a[1][1] - a[0][1] * a[1][0]) * invDet;
}

void PoseEKF::reset(
    float x0,
    float y0,
    float theta0,
    long leftTicks0,
    long rightTicks0
) {
    initialize(x0, y0, theta0, leftTicks0, rightTicks0);
}
