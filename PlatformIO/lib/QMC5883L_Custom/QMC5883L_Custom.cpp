#include "QMC5883L_Custom.h"

float QMC5883LCompass::normalizeSignedAngle(float angleRad) {
    while (angleRad > PI) angleRad -= 2.0f * PI;
    while (angleRad < -PI) angleRad += 2.0f * PI;
    return angleRad;
}

float QMC5883LCompass::normalizeUnsignedAngle(float angleRad) {
    while (angleRad < 0.0f) angleRad += 2.0f * PI;
    while (angleRad >= 2.0f * PI) angleRad -= 2.0f * PI;
    return angleRad;
}

bool QMC5883LCompass::addressAvailable(uint8_t address) {
    Wire.beginTransmission(address);
    return Wire.endTransmission() == 0;
}

bool QMC5883LCompass::writeRegister(uint8_t reg, uint8_t value) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.write(value);
    return Wire.endTransmission() == 0;
}

uint8_t QMC5883LCompass::readRegister(uint8_t reg) {
    Wire.beginTransmission(i2cAddress);
    Wire.write(reg);
    Wire.endTransmission(false);

    Wire.requestFrom(i2cAddress, (uint8_t)1);

    if (Wire.available()) {
        return Wire.read();
    }

    return 0;
}

int16_t QMC5883LCompass::read16LEFromWire() {
    const uint8_t lsb = Wire.read();
    const uint8_t msb = Wire.read();
    return (int16_t)((msb << 8) | lsb);
}

bool QMC5883LCompass::setup() {
    // O código de referência usa 0x2C. O QMC5883L comum usa 0x0D.
    // Para manter o projeto simples de editar e robusto, tentamos o endereço
    // configurado primeiro e, se não responder, tentamos o fallback 0x0D.
    if (!addressAvailable(i2cAddress)) {
        if (i2cAddress != FALLBACK_I2C_ADDRESS && addressAvailable(FALLBACK_I2C_ADDRESS)) {
            i2cAddress = FALLBACK_I2C_ADDRESS;
        } else {
            initialized = false;
            return false;
        }
    }

    // Soft reset.
    writeRegister(REG_CONTROL_2, 0x80);
    delay(10);

    // SET/RESET Period Register.
    // Datasheet recomenda escrever 0x01 no registrador 0x0B.
    writeRegister(REG_SET_RESET, 0x01);
    delay(10);

    // Configura modo contínuo: OSR=512, RNG=8G, ODR=200Hz, Continuous.
    writeRegister(REG_CONTROL_1, CONFIG_CONTINUOUS_200HZ_8G);
    delay(10);

    // Control 2: INT desabilitado para teste básico sem DRDY/INT.
    writeRegister(REG_CONTROL_2, 0x01);
    delay(10);

    chipId = readRegister(REG_CHIP_ID);
    status = readRegister(REG_STATUS);

    initialized = true;
    lastReadMs = millis();

    // Primeira leitura para preencher cache e médias móveis.
    loop();
    angularVelocityZRadS = 0.0f;
    return true;
}

bool QMC5883LCompass::readRawMagnetometer() {
    if (!initialized) return false;

    Wire.beginTransmission(i2cAddress);
    Wire.write(REG_X_LSB);

    if (Wire.endTransmission(false) != 0) {
        return false;
    }

    Wire.requestFrom(i2cAddress, (uint8_t)6);

    if (Wire.available() < 6) {
        return false;
    }

    // Ordem no QMC5883L:
    // 0x00 X_LSB, 0x01 X_MSB
    // 0x02 Y_LSB, 0x03 Y_MSB
    // 0x04 Z_LSB, 0x05 Z_MSB
    rawX = read16LEFromWire() - offsetX;
    rawY = read16LEFromWire() - offsetY;
    rawZ = read16LEFromWire() - offsetZ;

    return true;
}

void QMC5883LCompass::updateHeadingFromRaw() {
    xGauss = rawX / lsbPerGauss;
    yGauss = rawY / lsbPerGauss;
    zGauss = rawZ / lsbPerGauss;

    // Heading principal do robô: plano X/Y.
    // O QMC5883L não mede roll/pitch; portanto, só o heading Z é tratado como ângulo.
    float rawHeadingZ = atan2f(yGauss, xGauss);
    rawHeadingZ += declinationDeg * PI / 180.0f;
    rawHeadingZ = normalizeUnsignedAngle(rawHeadingZ);

    const float relativeHeading = normalizeSignedAngle(rawHeadingZ - angleZeroOffsetZRad);

    // Filtro circular simples: filtra o menor delta angular, evitando a média
    // linear de ângulos perto de +180/-180 graus, que causava saltos falsos.
    if (!hasHeadingSample) {
        headingZRad = relativeHeading;
        hasHeadingSample = true;
    } else {
        const float delta = normalizeSignedAngle(relativeHeading - headingZRad);
        headingZRad = normalizeSignedAngle(headingZRad + HEADING_LPF_ALPHA * delta);
    }
}

void QMC5883LCompass::loop() {
    if (!initialized && !setup()) return;

    const unsigned long now = millis();
    const float dt = (lastReadMs == 0 || now <= lastReadMs) ? 0.0f : (now - lastReadMs) / 1000.0f;

    status = readRegister(REG_STATUS);

    // Só tenta atualizar quando há dado novo. Se o status não vier confiável em
    // algum módulo, a próxima chamada continuará tentando sem travar o controle.
    if (!(status & 0x01)) {
        return;
    }

    if (!readRawMagnetometer()) return;

    const float oldHeadingZ = headingZRad;
    updateHeadingFromRaw();

    if (dt > 0.001f) {
        const float deltaHeading = normalizeSignedAngle(headingZRad - oldHeadingZ);
        filteredRateZ.add(deltaHeading / dt);
        angularVelocityZRadS = filteredRateZ.get();
    } else {
        angularVelocityZRadS = 0.0f;
    }

    lastReadMs = now;
}

void QMC5883LCompass::scanI2C(Stream& out) {
    out.println("Escaneando I2C...");

    for (uint8_t addr = 1; addr < 127; addr++) {
        Wire.beginTransmission(addr);

        if (Wire.endTransmission() == 0) {
            out.print("Dispositivo encontrado em 0x");

            if (addr < 16) {
                out.print("0");
            }

            out.println(addr, HEX);
        }
    }

    out.println("Scan finalizado.");
    out.println();
}

void QMC5883LCompass::setRawOffset(int16_t x, int16_t y, int16_t z) {
    offsetX = x;
    offsetY = y;
    offsetZ = z;
}

void QMC5883LCompass::setAngleZeroOffset(float xRad, float yRad, float zRad) {
    (void)xRad;
    (void)yRad;
    setHeadingZeroOffset(zRad);
}

void QMC5883LCompass::setHeadingZeroOffset(float zRad) {
    angleZeroOffsetZRad = normalizeUnsignedAngle(zRad);
}

void QMC5883LCompass::setCurrentAnglesAsZero() {
    // Usa o vetor bruto atual, e não a média móvel já offsetada, para evitar
    // que a média inicial gere um offset parcial.
    const float currentXGauss = rawX / lsbPerGauss;
    const float currentYGauss = rawY / lsbPerGauss;

    float currentZ = atan2f(currentYGauss, currentXGauss);
    currentZ += declinationDeg * PI / 180.0f;
    currentZ = normalizeUnsignedAngle(currentZ);

    setHeadingZeroOffset(currentZ);

    // Depois de mudar o referencial, força as saídas relativas para zero.
    for (int i = 0; i < 5; ++i) {
        filteredRateZ.add(0.0f);
    }

    headingZRad = 0.0f;
    angularVelocityZRadS = 0.0f;
    hasHeadingSample = true;
    lastReadMs = millis();
}

void QMC5883LCompass::clearAngleZeroOffset() {
    setHeadingZeroOffset(0.0f);
    hasHeadingSample = false;
    headingZRad = 0.0f;
    angularVelocityZRadS = 0.0f;
}
