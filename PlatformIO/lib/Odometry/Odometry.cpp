#include "odometry.h"

Odometry::Odometry()
    : x(0.0f), y(0.0f), theta(0.0f),
      currentLeftTicks(0), currentRightTicks(0),
      lastLeftTicks(0), lastRightTicks(0),
      initialized(false), map(nullptr) {}

Odometry::Odometry(Map* mapRef)
    : x(0.0f), y(0.0f), theta(0.0f),
      currentLeftTicks(0), currentRightTicks(0),
      lastLeftTicks(0), lastRightTicks(0),
      initialized(false), map(mapRef) {}

void Odometry::attachMap(Map* mapRef) {
    map = mapRef;
    updateMapPosition();
}

float Odometry::normalizeAngle(float angle) {
    while (angle > static_cast<float>(M_PI)) {
        angle -= 2.0f * static_cast<float>(M_PI);
    }
    while (angle < -static_cast<float>(M_PI)) {
        angle += 2.0f * static_cast<float>(M_PI);
    }
    return angle;
}

float Odometry::ticksToDistance(long tickDelta) {
    if (ENCODER_TICKS_PER_REV <= 0) {
        return 0.0f;
    }

    const float revolutions = static_cast<float>(tickDelta) / static_cast<float>(ENCODER_TICKS_PER_REV);
    return (2.0f * static_cast<float>(M_PI) * WHEEL_RADIUS) * revolutions;
}

bool Odometry::updateMapPosition() {
    if (!map) {
        return false;
    }
    if (MAP_CELL_SIZE_M <= 0.0f) {
        return false;
    }

    const int mapX = toMapX();
    const int mapY = toMapY();

    if (mapX < 0 || mapY < 0) {
        return false;
    }

    return map->setPosition(static_cast<unsigned int>(mapX), static_cast<unsigned int>(mapY));
}

int Odometry::toMapX() const {
    return static_cast<int>(x / MAP_CELL_SIZE_M) + MAP_ORIGIN_X;
}

int Odometry::toMapY() const {
    return static_cast<int>(y / MAP_CELL_SIZE_M) + MAP_ORIGIN_Y;
}

void Odometry::update(long leftTicks, long rightTicks) {
    currentLeftTicks = leftTicks;
    currentRightTicks = rightTicks;

    // Primeira leitura apenas sincroniza referencia para calculo incremental.
    if (!initialized) {
        initialized = true;
        lastLeftTicks = currentLeftTicks;
        lastRightTicks = currentRightTicks;
        updateMapPosition();
        return;
    }

    const long deltaLeftTicks = currentLeftTicks - lastLeftTicks;
    const long deltaRightTicks = currentRightTicks - lastRightTicks;

    lastLeftTicks = currentLeftTicks;
    lastRightTicks = currentRightTicks;

    const float distanceLeft = ticksToDistance(deltaLeftTicks);
    const float distanceRight = ticksToDistance(deltaRightTicks);

    // Cinematica diferencial para obter deslocamento linear e angular.
    const float deltaS = 0.5f * (distanceLeft + distanceRight);
    const float deltaTheta = (WHEEL_BASE > 0.0f) ? ((distanceRight - distanceLeft) / WHEEL_BASE) : 0.0f;

    x += deltaS * cosf(theta);
    y += deltaS * sinf(theta);
    theta = normalizeAngle(theta + deltaTheta);

    updateMapPosition();
}

float Odometry::getX() const {
    return x;
}

float Odometry::getY() const {
    return y;
}

float Odometry::getTheta() const {
    return theta;
}

long Odometry::getLeftTicks() const {
    return currentLeftTicks;
}

long Odometry::getRightTicks() const {
    return currentRightTicks;
}

int Odometry::getMapX() const {
    return toMapX();
}

int Odometry::getMapY() const {
    return toMapY();
}

void Odometry::reset(float x0, float y0, float theta0, long leftTicks0, long rightTicks0) {
    x = x0;
    y = y0;
    theta = normalizeAngle(theta0);

    currentLeftTicks = leftTicks0;
    currentRightTicks = rightTicks0;
    lastLeftTicks = leftTicks0;
    lastRightTicks = rightTicks0;
    initialized = true;

    updateMapPosition();
}