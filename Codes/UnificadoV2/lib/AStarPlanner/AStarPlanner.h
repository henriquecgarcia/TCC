#pragma once
#include <Arduino.h>
#include "../GridMap/GridMap.h"
#include "../../src/SharedTypes.h"

class AStarPlanner {
public:
  /**
   * Cria um planejador A* usando um mapa compartilhado.
   * @param map Referencia para o mapa usado na validacao das celulas.
   * @return Nao retorna valor.
   */
  explicit AStarPlanner(GridMap& map);
  /**
   * Calcula uma rota entre a origem e o destino informados.
   * @param sx Coordenada X da origem em celulas.
   * @param sy Coordenada Y da origem em celulas.
   * @param gx Coordenada X do destino em celulas.
   * @param gy Coordenada Y do destino em celulas.
   * @param outPath Buffer de saida que recebe a rota gerada.
   * @return true quando a rota foi encontrada; false em falha ou sem caminho.
   */
  bool plan(uint16_t sx, uint16_t sy, uint16_t gx, uint16_t gy, PathBuffer& outPath);

private:
  struct Node {
    int16_t parent;
    uint16_t g;
    uint16_t f;
    int8_t dir;
    bool open;
    bool closed;
  };

  GridMap& _map;
  Node _nodes[RobotConfig::MAP_CELLS];
  /**
   * Converte coordenadas de celula em indice linear do mapa.
   * @param x Coordenada X em celulas.
   * @param y Coordenada Y em celulas.
   * @return Indice linear correspondente.
   */
  static uint16_t idx(uint16_t x, uint16_t y);
  /**
   * Calcula a heuristica Manhattan entre dois pontos.
   * @param ax Coordenada X de origem.
   * @param ay Coordenada Y de origem.
   * @param bx Coordenada X de destino.
   * @param by Coordenada Y de destino.
   * @return Custo heuristico estimado entre os pontos.
   */
  static uint16_t heuristic(uint16_t ax, uint16_t ay, uint16_t bx, uint16_t by);
  /**
   * Remove da fila aberta o melhor nodo candidato.
   * @return Indice do nodo escolhido, ou -1 se nao houver candidatos.
   */
  int16_t popBestOpen();
  /**
   * Monta o caminho final a partir dos nodos visitados.
   * @param startIdx Indice linear do ponto inicial.
   * @param goalIdx Indice linear do ponto final.
   * @param outPath Buffer de saida que recebe o caminho reconstruido.
   * @return true quando o caminho foi montado com sucesso; false caso contrario.
   */
  bool buildPath(uint16_t startIdx, uint16_t goalIdx, PathBuffer& outPath);
};
