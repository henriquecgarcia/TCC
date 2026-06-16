#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#include "MediaMovel.h"

/**
 * Driver simples e editável para o magnetômetro QMC5883L/QMC5883 compatível.
 *
 * O QMC5883L NÃO é IMU completa: ele não fornece acelerômetro e, para este
 * projeto, não será usado como sensor de temperatura. A classe expõe apenas:
 * - campo magnético bruto X/Y/Z;
 * - campo magnético convertido para Gauss;
 * - heading/yaw calculado no plano X/Y;
 * - taxa angular Z estimada pela variação do heading.
 */
class QMC5883LCompass {
public:
    // ==========================
    // Configurações de alto nível
    // ==========================

    /** Endereço preferencial do sensor no I²C. Edite aqui se seu módulo usar outro endereço. */
    static constexpr uint8_t DEFAULT_I2C_ADDRESS = 0x2C;

    /** Endereço alternativo comum em módulos QMC5883L. */
    static constexpr uint8_t FALLBACK_I2C_ADDRESS = 0x0D;

    /** Configuração: OSR=512, RNG=8G, ODR=200Hz, MODE=Continuous. */
    static constexpr uint8_t CONFIG_CONTINUOUS_200HZ_8G = 0x1D;

    /** Sensibilidade conforme datasheet: ±8G = 3000 LSB/Gauss. */
    static constexpr float DEFAULT_LSB_PER_GAUSS = 3000.0f;

    /** Endereço I²C em uso. Pode ser alterado antes de setup(). */
    uint8_t i2cAddress = DEFAULT_I2C_ADDRESS;

    /** Sensibilidade usada para converter raw -> Gauss. */
    float lsbPerGauss = DEFAULT_LSB_PER_GAUSS;

    /** Declinação magnética local em graus. Para testes iniciais, mantenha 0. */
    float declinationDeg = 0.0f;

    /** Offsets brutos para calibração manual simples do magnetômetro. */
    int16_t offsetX = 0;
    int16_t offsetY = 0;
    int16_t offsetZ = 0;

    /**
     * Offset angular do heading/yaw.
     *
     * Este valor define qual direção do robô será tratada como Z=0 rad.
     * O QMC5883L não mede roll/pitch reais, então não há offset angular X/Y.
     */
    float angleZeroOffsetZRad = 0.0f;

private:
    // ==========================
    // Registradores do QMC5883L
    // ==========================
    static constexpr uint8_t REG_X_LSB = 0x00;
    static constexpr uint8_t REG_Y_LSB = 0x02;
    static constexpr uint8_t REG_Z_LSB = 0x04;
    static constexpr uint8_t REG_STATUS = 0x06;
    static constexpr uint8_t REG_CONTROL_1 = 0x09;
    static constexpr uint8_t REG_CONTROL_2 = 0x0A;
    static constexpr uint8_t REG_SET_RESET = 0x0B;
    static constexpr uint8_t REG_CHIP_ID = 0x0D;

    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    float xGauss = 0.0f;
    float yGauss = 0.0f;
    float zGauss = 0.0f;

    /** Heading relativo ao zero configurado, em radianos, normalizado para [-PI, PI]. */
    float headingZRad = 0.0f;

    /** Taxa angular estimada pela variação do headingZRad. */
    float angularVelocityZRadS = 0.0f;

    uint8_t status = 0;
    uint8_t chipId = 0;

    unsigned long lastReadMs = 0;
    bool initialized = false;

    MediaMovel filteredRateZ{5};
    bool hasHeadingSample = false;
    static constexpr float HEADING_LPF_ALPHA = 0.35f;

    /** Normaliza um ângulo para [-PI, PI]. */
    static float normalizeSignedAngle(float angleRad);

    /** Normaliza um heading para [0, 2PI). */
    static float normalizeUnsignedAngle(float angleRad);

    /** Escreve um byte em um registrador do QMC5883L. */
    bool writeRegister(uint8_t reg, uint8_t value);

    /** Lê um byte de um registrador do QMC5883L. */
    uint8_t readRegister(uint8_t reg);

    /** Verifica se há dispositivo respondendo no endereço informado. */
    bool addressAvailable(uint8_t address);

    /** Lê um valor int16 little-endian a partir do buffer atual do Wire. */
    static int16_t read16LEFromWire();

    /** Lê os 6 bytes brutos do campo magnético, aplicando offsets raw. */
    bool readRawMagnetometer();

    /** Calcula heading relativo ao offset configurado. */
    void updateHeadingFromRaw();

public:
    QMC5883LCompass() = default;

    /** Inicializa o QMC5883L no barramento I²C já iniciado por Wire.begin(). */
    bool setup();

    /** Atualiza leitura bruta, heading e velocidade angular estimada. */
    void loop();

    /** Escaneia o I²C e imprime no Serial os dispositivos encontrados. */
    void scanI2C(Stream& out = Serial);

    /** Define manualmente o offset raw do magnetômetro. */
    void setRawOffset(int16_t x, int16_t y, int16_t z);

    /** Define manualmente qual heading será considerado Z=0 rad. X/Y são ignorados por compatibilidade. */
    void setAngleZeroOffset(float xRad, float yRad, float zRad);

    /** Define manualmente qual heading será considerado Z=0 rad. */
    void setHeadingZeroOffset(float zRad);

    /** Usa o heading atual como novo zero relativo do robô. */
    void setCurrentAnglesAsZero();

    /** Reseta o offset angular para o zero absoluto calculado diretamente do sensor. */
    void clearAngleZeroOffset();

    bool isInitialized() const { return initialized; }
    uint8_t getAddress() const { return i2cAddress; }
    uint8_t getChipId() const { return chipId; }
    uint8_t getStatus() const { return status; }
    bool isDataReady() const { return status & 0x01; }
    bool hasOverflow() const { return status & 0x02; }
    bool hasDataSkip() const { return status & 0x04; }

    int16_t getRawX() const { return rawX; }
    int16_t getRawY() const { return rawY; }
    int16_t getRawZ() const { return rawZ; }

    float getXGauss() const { return xGauss; }
    float getYGauss() const { return yGauss; }
    float getZGauss() const { return zGauss; }

    float getAngleZRad() const { return headingZRad; }
    float getHeadingRad() const { return headingZRad; }
    float getHeadingDeg() const { return headingZRad * 180.0f / PI; }
    float getAngularVelocityZRadS() const { return angularVelocityZRadS; }
    unsigned long getLastReadMs() const { return lastReadMs; }
};
