#ifndef ODOMETRY_H
#define ODOMETRY_H

#include <Arduino.h>
#include <math.h>

#include "Map.h"

// Parametros de calibracao do robo (SI).
#define WHEEL_RADIUS 0.0325f           // metros
#define ENCODER_TICKS_PER_REV 20       // ticks por volta
#define WHEEL_BASE 0.1400f             // metros

// Conversao posicao continua (m) -> grid discreto (celulas).
#define MAP_CELL_SIZE_M 0.05f          // metros por celula
#define MAP_ORIGIN_X 0                 // deslocamento em celulas no eixo X
#define MAP_ORIGIN_Y 0                 // deslocamento em celulas no eixo Y

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

class Odometry {
public:
    Odometry();
    explicit Odometry(Map* mapRef);

    void attachMap(Map* mapRef);

    // Atualizacao incremental baseada em contagens acumuladas de ticks.
    void update(long leftTicks, long rightTicks);

    float getX() const;
    float getY() const;
    float getTheta() const;

    long getLeftTicks() const;
    long getRightTicks() const;
    int getMapX() const;
    int getMapY() const;

    // Reinicia pose e referencia de ticks para evitar salto na proxima atualizacao.
    void reset(float x0 = 0.0f, float y0 = 0.0f, float theta0 = 0.0f, long leftTicks0 = 0, long rightTicks0 = 0);

private:
    float x;
    float y;
    float theta;

    long currentLeftTicks;
    long currentRightTicks;
    long lastLeftTicks;
    long lastRightTicks;
    bool initialized;

    Map* map;

    static float normalizeAngle(float angle);
    static float ticksToDistance(long tickDelta);
    int toMapX() const;
    int toMapY() const;
    bool updateMapPosition();
};

#endif