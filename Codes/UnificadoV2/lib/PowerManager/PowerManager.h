#pragma once
#include <Arduino.h>
#include <esp_wifi.h>
#include "../../constants/RobotConfig.h"

class PowerManager {
public:
  enum Mode : uint8_t { NORMAL, ECONOMY };
  /**
   * Cria o gerenciador de energia com estado padrao.
   * @return Nao retorna valor.
   */
  PowerManager();
  /**
   * Associa os indicadores de atividade usados na politica de energia.
   * @param lastCommandMs Ponteiro para o timestamp da ultima ordem recebida.
   * @param robotActive Ponteiro para o estado atual do robo.
   * @return Nao retorna valor.
   */
  void begin(uint32_t* lastCommandMs, bool* robotActive);
  /**
   * Executa a maquina de estados de economia de energia.
   * @return Nao retorna valor.
   */
  void loop();
  /**
   * Forca o modo normal de operacao.
   * @return Nao retorna valor.
   */
  void forceNormal();
  /**
   * Retorna o modo atual de operacao.
   * @return Modo ativo no momento.
   */
  Mode mode() const;

private:
  uint32_t* _lastCommandMs;
  bool* _robotActive;
  uint32_t _lastCheckMs;
  Mode _mode;
  /**
   * Define internamente o modo de operacao.
   * @param mode Novo modo desejado.
   * @return Nao retorna valor.
   */
  void setMode(Mode mode);
};
