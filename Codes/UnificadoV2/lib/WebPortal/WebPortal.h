#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "../GridMap/GridMap.h"
#include "../../constants/RobotConfig.h"
#include "../../src/SharedTypes.h"

class WebPortal {
public:
  typedef bool (*CommandHandler)(const char* payload, size_t len);
  /**
   * Cria o portal web com acesso ao mapa, pose e caminho atuais.
   * @param map Mapa compartilhado para telemetria.
   * @param pose Ponteiro para a pose atual do robo.
   * @param path Ponteiro para o caminho atual de navegacao.
   * @param mutex Ponteiro para o mutex de sincronizacao dos dados compartilhados.
   * @return Nao retorna valor.
   */
  WebPortal(GridMap& map, Pose2D* pose, PathBuffer* path, SemaphoreHandle_t* mutex);
  /**
   * Sobe o servidor web e registra o handler de comandos.
   * @param handler Funcao chamada quando um comando chega pelo websocket.
   * @return Nao retorna valor.
   */
  void begin(CommandHandler handler);
  /**
   * Processa eventos periodicos do portal e do websocket.
   * @return Nao retorna valor.
   */
  void loop();
  /**
   * Envia telemetria do estado atual para os clientes conectados.
   * @param navActive Indica se a navegacao esta ativa.
   * @param tofMm Distancia atual do ToF em milimetros.
   * @return Nao retorna valor.
   */
  void broadcastTelemetry(bool navActive, uint16_t tofMm);
  /**
   * Exibe uma mensagem curta para o usuario no portal.
   * @param message Texto da notificacao.
   * @return Nao retorna valor.
   */
  void showToast(const char* message);
  /**
   * Marca o mapa para reenviar aos clientes.
   * @return Nao retorna valor.
   */
  void markMapDirty();
  /**
   * Marca o caminho para reenviar aos clientes.
   * @return Nao retorna valor.
   */
  void markPathDirty();

private:
  AsyncWebServer _server;
  AsyncWebSocket _ws;
  GridMap& _map;
  Pose2D* _pose;
  PathBuffer* _path;
  SemaphoreHandle_t* _mutex;
  CommandHandler _handler;
  uint32_t _lastTelemetryMs;
  uint32_t _lastMapTelemetryMs;
  uint32_t _lastPathTelemetryMs;
  char _toast[128];
  bool _toastPending;
  bool _mapDirty;
  bool _pathDirty;

  /**
   * Trata eventos do websocket e repassa comandos ao handler.
   * @param server Servidor websocket associado.
   * @param client Cliente que gerou o evento.
   * @param type Tipo do evento recebido.
   * @param arg Argumento complementar do evento.
   * @param data Buffer com os dados recebidos.
   * @param len Tamanho do buffer de dados.
   * @return Nao retorna valor.
   */
  void handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
  /**
   * Envia o estado consolidado do robo pela websocket.
   * @param navActive Indica se a navegacao esta ativa.
   * @param tofMm Distancia atual do ToF em milimetros.
   * @return Nao retorna valor.
   */
  void sendState(bool navActive, uint16_t tofMm);
  /**
   * Envia o mapa atualizado aos clientes conectados.
   * @return Nao retorna valor.
   */
  void sendMap();
  /**
   * Envia o caminho atualizado aos clientes conectados.
   * @return Nao retorna valor.
   */
  void sendPath();
};
