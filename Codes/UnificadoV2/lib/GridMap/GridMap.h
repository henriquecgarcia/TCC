#pragma once
#include <Arduino.h>
#include <FS.h>
#include <SPIFFS.h>
#include "../../constants/RobotConfig.h"

class GridMap {
public:
  /**
   * Inicializa o mapa com o caminho do arquivo estatico.
   * @param path Caminho do arquivo no SPIFFS.
   * @return true quando o mapa foi inicializado com sucesso; false caso contrario.
   */
  bool begin(const char* path);
  /**
   * Verifica se uma celula esta ocupada.
   * @param x Coordenada X em celulas.
   * @param y Coordenada Y em celulas.
   * @return true se a celula estiver ocupada; false se estiver livre ou em erro.
   */
  bool isOccupied(uint16_t x, uint16_t y);
  /**
   * Sobrescreve dinamicamente uma celula do mapa.
   * @param x Coordenada X em celulas.
   * @param y Coordenada Y em celulas.
   * @param occupied true para marcar como ocupada, false para liberar.
   * @return true quando a atualizacao foi aplicada; false em coordenada invalida.
   */
  bool setDynamic(uint16_t x, uint16_t y, bool occupied);
  /**
   * Limpa todas as alteracoes dinamicas do mapa.
   * @return Nao retorna valor.
   */
  void clearDynamic();
  /**
   * Retorna o valor de uma celula para uso na telemetria.
   * @param x Coordenada X em celulas.
   * @param y Coordenada Y em celulas.
   * @return Valor codificado da celula para envio ao portal.
   */
  uint8_t cellForTelemetry(uint16_t x, uint16_t y);

private:
  const char* _path;
  uint8_t _dynamic[RobotConfig::MAP_CELLS];
  /**
   * Lera a celula estatica correspondente no arquivo base.
   * @param x Coordenada X em celulas.
   * @param y Coordenada Y em celulas.
   * @param value Valor lido da celula.
   * @return true quando a leitura foi bem sucedida; false caso contrario.
   */
  bool readStatic(uint16_t x, uint16_t y, uint8_t& value);
  /**
   * Verifica se as coordenadas estao dentro dos limites do mapa.
   * @param x Coordenada X em celulas.
   * @param y Coordenada Y em celulas.
   * @return true se a coordenada for valida; false caso contrario.
   */
  bool valid(uint16_t x, uint16_t y) const;
  /**
   * Calcula o indice linear correspondente a uma celula.
   * @param x Coordenada X em celulas.
   * @param y Coordenada Y em celulas.
   * @return Indice linear da celula no mapa.
   */
  uint32_t index(uint16_t x, uint16_t y) const;
};
