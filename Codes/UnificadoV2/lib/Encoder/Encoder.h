#pragma once
#include <Arduino.h>

class Encoder {
public:
  /**
   * Cria um encoder quadratura com os pinos informados.
   * @param pinA Pino do canal A.
   * @param pinB Pino do canal B.
   * @return Nao retorna valor.
   */
  Encoder(uint8_t pinA, uint8_t pinB);
  /**
   * Configura os pinos e registra as interrupcoes do encoder.
   * @return Nao retorna valor.
   */
  void begin();
  /**
   * Retorna a contagem acumulada de pulsos.
   * @return Total de pulsos registrados desde o ultimo reset.
   */
  long getTicks() const;
  /**
   * Retorna a variacao desde a ultima leitura e zera o delta.
   * @return Delta de pulsos acumulado desde a ultima chamada.
   */
  long readAndResetDelta();
  /**
   * Zera a contagem total e o delta acumulado.
   * @return Nao retorna valor.
   */
  void reset();

private:
  uint8_t _pinA;
  uint8_t _pinB;
  volatile long _ticks;
  volatile long _delta;

  /**
   * Trata a interrupcao do canal A.
   * @return Nao retorna valor.
   */
  void IRAM_ATTR handleA();
  /**
   * Trata a interrupcao do canal B.
   * @return Nao retorna valor.
   */
  void IRAM_ATTR handleB();
  /**
   * Ponteiro de ISR para o canal A.
   * @param arg Ponteiro para a instancia do encoder.
   * @return Nao retorna valor.
   */
  static void IRAM_ATTR isrA(void* arg);
  /**
   * Ponteiro de ISR para o canal B.
   * @param arg Ponteiro para a instancia do encoder.
   * @return Nao retorna valor.
   */
  static void IRAM_ATTR isrB(void* arg);
};
