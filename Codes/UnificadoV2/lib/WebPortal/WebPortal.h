#pragma once
#include <Arduino.h>
#include <ESPAsyncWebServer.h>
#include "../GridMap/GridMap.h"
#include "../../constants/RobotConfig.h"
#include "../../src/SharedTypes.h"

class WebPortal {
public:
  typedef bool (*CommandHandler)(const char* payload, size_t len);
  WebPortal(GridMap& map, Pose2D* pose, PathBuffer* path, SemaphoreHandle_t* mutex);
  void begin(CommandHandler handler);
  void loop();
  void broadcastTelemetry(bool navActive, uint16_t tofMm);
  void showToast(const char* message);
  void markMapDirty();
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

  void handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len);
  void sendState(bool navActive, uint16_t tofMm);
  void sendMap();
  void sendPath();
};
