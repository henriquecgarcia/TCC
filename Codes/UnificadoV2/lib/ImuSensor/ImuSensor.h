#pragma once
#include <Arduino.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "../../src/SharedTypes.h"

class ImuSensor {
public:
  /**
   * Inicializa o IMU e executa a calibracao inicial do gyro.
   * @return true quando o sensor foi inicializado com sucesso; false caso contrario.
   */
  bool begin();
  /**
   * Faz a leitura do sensor e integra o yaw estimado.
   * @param distanceMm Distancia medida pelo ToF em milimetros.
   * @param distanceValid Indica se a distancia recebida e valida.
   * @param dt Intervalo de tempo em segundos.
   * @return Amostra do sensor com valores lidos e estimados.
   */
  SensorSample read(uint16_t distanceMm, bool distanceValid, float dt);
  /**
   * Retorna o yaw atual estimado.
   * @return Yaw em radianos.
   */
  float getYawRad() const;

private:
  Adafruit_MPU6050 _mpu;
  float _gyroBiasZ;
  float _yawRad;
  /**
   * Executa a calibracao do eixo Z do giroscopio.
   * @return true quando a calibracao foi concluida; false caso contrario.
   */
  bool calibrateGyroZ();
};
