#pragma once
#include <Arduino.h>
#include "../../src/SharedTypes.h"

class PurePursuit {
public:
  /**
   * Cria o seguidor de rota com lookahead padrao.
   * @return Nao retorna valor.
   */
  PurePursuit();
  /**
   * Define a distancia de lookahead usada na busca do alvo.
   * @param meters Distancia de lookahead em metros.
   * @return Nao retorna valor.
   */
  void setLookahead(float meters);
  /**
   * Calcula a referencia de velocidade para seguir a rota.
   * @param pose Pose atual do robo.
   * @param path Caminho alvo a ser seguido.
   * @param desiredSpeed Velocidade linear desejada em m/s.
   * @param vLeft Saida com a velocidade da roda esquerda em m/s.
   * @param vRight Saida com a velocidade da roda direita em m/s.
   * @param goalReached Saida que indica se o alvo final foi alcancado.
   * @return true quando existe comando valido para seguir a rota; false caso contrario.
   */
  bool compute(const Pose2D& pose, const PathBuffer& path, float desiredSpeed, float& vLeft, float& vRight, bool& goalReached);

private:
  float _lookahead;
  uint16_t _lastIndex;
  /**
   * Busca o proximo ponto alvo da rota para o controle.
   * @param pose Pose atual do robo.
   * @param path Caminho alvo a ser seguido.
   * @param target Saida com o ponto alvo selecionado.
   * @return true quando um alvo valido foi encontrado; false caso contrario.
   */
  bool findTarget(const Pose2D& pose, const PathBuffer& path, PathPoint& target);
};
