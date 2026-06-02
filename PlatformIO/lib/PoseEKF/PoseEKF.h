#ifndef POSE_EKF_H
#define POSE_EKF_H

#include <Arduino.h>
#include <math.h>

/**
 * Extended Kalman Filter para estimação de pose (x, y, theta)
 * Combina:
 *   - Odometria (encoders) como predição
 *   - Giroscópio (MPU6050) para correção de theta
 *   - Sensor de distância (VL53L0X) para detecção de obstáculos
 *
 * Estado: [x, y, theta]
 */
class PoseEKF {
private:
    // Estado: [x, y, theta]
    float state[3];  // [x (m), y (m), theta (rad)]

    // Matriz de covariância (P) - 3x3
    float P[3][3];

    // Matrizes de ruído
    float Q[3][3];  // Covariância do processo (movimento)
    float R[3][3];  // Covariância da medição (sensores)

    // Parâmetros do robô
    float wheelRadius;
    float wheelBase;
    float mapCellSize;

    // Última leitura de ticks para cálculo incremental
    long lastLeftTicks;
    long lastRightTicks;

    // Última leitura de theta do giroscópio
    float lastTheta;

    // Contadores para atualização adaptativa
    unsigned long lastUpdateTime;

    // Métodos auxiliares
    static void normalizeAngle(float& angle);

    // Multiplicação de matrizes
    void matMultiply3x3(const float a[3][3], const float b[3][3], float result[3][3]);
    void matMultiply3x1(const float a[3][3], const float b[3], float result[3]);
    void matTranspose3x3(const float a[3][3], float result[3][3]);
    void matInverse3x3(const float a[3][3], float result[3][3]);
    void matAdd3x3(const float a[3][3], const float b[3][3], float result[3][3]);
    void matSubtract3x3(const float a[3][3], const float b[3][3], float result[3][3]);

    // Funções da cinemática
    float ticksToDistance(long tickDelta);
    float computeTheta(float leftDist, float rightDist);

public:
    /**
     * Construtor com configuração de parâmetros do robô
     */
    PoseEKF(
        float wheelRadius_m = 0.0325f,
        float wheelBase_m = 0.1400f,
        float mapCellSize_m = 0.05f,
        float encoderTicksPerRev = 20.0f
    );

    /**
     * Inicializa o filtro com pose inicial
     * Deve ser chamado no setup()
     */
    void initialize(
        float x0 = 0.0f,
        float y0 = 0.0f,
        float theta0 = 0.0f,
        long leftTicks0 = 0,
        long rightTicks0 = 0
    );

    /**
     * Configura matrizes de ruído
     */
    void setProcessNoise(float qx, float qy, float qTheta);
    void setMeasurementNoise(float rTheta, float rDistance, float rDrift);

    /**
     * Predição: atualiza estado usando modelo de movimento (odometria)
     * Deve ser chamado frequentemente (a cada ciclo de controle)
     */
    void predict(long leftTicks, long rightTicks, float deltaTime_s);

    /**
     * Atualização com giroscópio: corrige theta usando MPU6050
     */
    void updateWithGyro(float gyroZ_rad_s, float deltaTime_s);

    /**
     * Atualização com distância: detecta obstáculo próximo e ajusta
     */
    void updateWithDistance(float distance_mm, bool obstacleDetected = false);

    /**
     * Getters da pose estimada
     */
    float getX() const { return state[0]; }
    float getY() const { return state[1]; }
    float getTheta() const { return state[2]; }

    /**
     * Getters da confiança (covariância diagonal)
     */
    float getCovariance_X() const { return P[0][0]; }
    float getCovariance_Y() const { return P[1][1]; }
    float getCovariance_Theta() const { return P[2][2]; }

    /**
     * Mantém a pose atual e apenas sincroniza os ticks de referência.
     * Use quando o robô está fisicamente parado para evitar drift de localização.
     */
    void holdCurrentPose(long leftTicksNow, long rightTicksNow);

    /**
     * Reseta o filtro
     */
    void reset(
        float x0 = 0.0f,
        float y0 = 0.0f,
        float theta0 = 0.0f,
        long leftTicks0 = 0,
        long rightTicks0 = 0
    );

private:
    float encoderTicksPerRev;
};

#endif // POSE_EKF_H
