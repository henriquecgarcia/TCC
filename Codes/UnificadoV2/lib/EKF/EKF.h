#pragma once
#include <Arduino.h>
#include "../../src/SharedTypes.h"

class EKF {
public:
  /**
   * Cria o filtro com estados e covariancias padrao.
   * @return Nao retorna valor.
   */
  EKF();
  /**
   * Reinicia o estado estimado do filtro.
   * @param x Posicao X inicial.
   * @param y Posicao Y inicial.
   * @param theta Orientacao inicial em radianos.
   * @return Nao retorna valor.
   */
  void reset(float x, float y, float theta);
  /**
   * Executa a etapa de previsao do estado.
   * @param vLeft Velocidade da roda esquerda em m/s.
   * @param vRight Velocidade da roda direita em m/s.
   * @param dt Intervalo de tempo em segundos.
   * @return Nao retorna valor.
   */
  void predict(float vLeft, float vRight, float dt);
  /**
   * Corrige apenas a orientacao com uma medida externa.
   * @param measuredTheta Orientacao medida em radianos.
   * @return Nao retorna valor.
   */
  void updateTheta(float measuredTheta);
  /**
   * Retorna a pose atual estimada pelo filtro.
   * @return Pose2D com posicao e orientacao atuais.
   */
  Pose2D pose() const;

private:
  Pose2D _x;
  float _p[3][3];
  float _q[3];
  float _rTheta;
};
