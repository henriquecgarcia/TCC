#pragma once
#include <Arduino.h>
#include <Adafruit_VL53L0X.h>

class ToFSensor {
public:
  /**
   * Inicializa o sensor de distancia ToF.
   * @return true quando o sensor foi inicializado com sucesso; false caso contrario.
   */
  bool begin();
  /**
   * Realiza uma leitura unica da distancia.
   * @param mm Saida com a distancia em milimetros.
   * @return true quando a leitura e valida; false em falha de medicao.
   */
  bool read(uint16_t& mm);

private:
  Adafruit_VL53L0X _sensor;
};
