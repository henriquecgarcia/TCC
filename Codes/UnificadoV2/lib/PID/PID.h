#pragma once
#include <Arduino.h>

class PID {
public:
  /**
   * Cria um controlador PID com ganhos e intervalo fixos.
   * @param kp Ganho proporcional.
   * @param ki Ganho integral.
   * @param kd Ganho derivativo.
   * @param dt Intervalo de amostragem em segundos.
   * @return Nao retorna valor.
   */
  PID(float kp, float ki, float kd, float dt);
  /**
   * Atualiza os ganhos do controlador.
   * @param kp Novo ganho proporcional.
   * @param ki Novo ganho integral.
   * @param kd Novo ganho derivativo.
   * @return Nao retorna valor.
   */
  void setTunings(float kp, float ki, float kd);
  /**
   * Define os limites de saida do controle.
   * @param minOut Valor minimo permitido na saida.
   * @param maxOut Valor maximo permitido na saida.
   * @return Nao retorna valor.
   */
  void setLimits(float minOut, float maxOut);
  /**
   * Calcula a nova saida do PID.
   * @param setpoint Valor desejado.
   * @param measurement Valor medido.
   * @return Comando de controle calculado e limitado.
   */
  float compute(float setpoint, float measurement);
  /**
   * Reinicia o estado interno do controlador.
   * @return Nao retorna valor.
   */
  void reset();

private:
  float _kp;
  float _ki;
  float _kd;
  float _dt;
  float _integral;
  float _prevError;
  float _minOut;
  float _maxOut;
  /**
   * Limita um valor entre dois extremos.
   * @param v Valor de entrada.
   * @param lo Limite inferior.
   * @param hi Limite superior.
   * @return Valor ajustado ao intervalo informado.
   */
  static float clamp(float v, float lo, float hi);
};
