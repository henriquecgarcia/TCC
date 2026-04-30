#include "WebPortal.h"
#include <SPIFFS.h>
#include <math.h>

WebPortal::WebPortal(GridMap& map, Pose2D* pose, PathBuffer* path, SemaphoreHandle_t* mutex)
  : _server(80), _ws("/ws"), _map(map), _pose(pose), _path(path), _mutex(mutex), _handler(nullptr),
    _lastTelemetryMs(0), _lastMapTelemetryMs(0), _lastPathTelemetryMs(0), _toastPending(false), _mapDirty(true), _pathDirty(true) {
  _toast[0] = '\0';
}

void WebPortal::begin(CommandHandler handler) {
  _handler = handler;
  _ws.onEvent([this](AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
    this->handleWsEvent(server, client, type, arg, data, len);
  });
  _server.addHandler(&_ws);
  _server.serveStatic("/", SPIFFS, "/").setDefaultFile("index.html");
  _server.begin();
}

void WebPortal::loop() {
  // Remove clientes desconectados e evita acumulo de buffers internos.
  _ws.cleanupClients();
}

void WebPortal::broadcastTelemetry(bool navActive, uint16_t tofMm) {
  // Se nao existe cliente conectado, nao monta JSON e nao usa CPU/RAM atoa.
  if (_ws.count() == 0) return;

  const uint32_t now = millis();

  // Telemetria de pose fica pequena e em baixa frequencia para nao derrubar o AP do ESP32.
  if ((now - _lastTelemetryMs) >= RobotConfig::TELEMETRY_PERIOD_MS) {
    _lastTelemetryMs = now;
    sendState(navActive, tofMm);
  }

  // Mapa e caminho sao enviados separadamente. Antes eram enviados junto da pose,
  // o que gerava pacotes grandes e podia saturar o WebSocket/Wi-Fi.
  if (_mapDirty || (now - _lastMapTelemetryMs) >= RobotConfig::MAP_TELEMETRY_PERIOD_MS) {
    _lastMapTelemetryMs = now;
    _mapDirty = false;
    sendMap();
  }

  if (_pathDirty || (now - _lastPathTelemetryMs) >= RobotConfig::PATH_TELEMETRY_PERIOD_MS) {
    _lastPathTelemetryMs = now;
    _pathDirty = false;
    sendPath();
  }
}

void WebPortal::showToast(const char* message) {
  if (message == nullptr || message[0] == '\0') return;
  strncpy(_toast, message, sizeof(_toast) - 1);
  _toast[sizeof(_toast) - 1] = '\0';
  _toastPending = true;
}

void WebPortal::markMapDirty() {
  _mapDirty = true;
}

void WebPortal::markPathDirty() {
  _pathDirty = true;
}

void WebPortal::handleWsEvent(AsyncWebSocket* server, AsyncWebSocketClient* client, AwsEventType type, void* arg, uint8_t* data, size_t len) {
  (void)server;
  (void)arg;

  if (type == WS_EVT_CONNECT) {
    // Cliente novo precisa receber mapa e caminho, mas isso sera feito no ciclo normal,
    // sem tentar empilhar varios pacotes dentro do callback de conexao.
    _mapDirty = true;
    _pathDirty = true;
    return;
  }

  if (type != WS_EVT_DATA || _handler == nullptr || len == 0 || len > 384) return;

  char payload[385];
  memcpy(payload, data, len);
  payload[len] = '\0';

  const bool ok = _handler(payload, len);
  if (client != nullptr) {
    client->text(ok ? "{\"type\":\"ack\",\"ok\":true}" : "{\"type\":\"ack\",\"ok\":false}");
  }
}

void WebPortal::sendState(bool navActive, uint16_t tofMm) {
  Pose2D poseCopy = {0.0f, 0.0f, 0.0f};
  if (_mutex != nullptr && xSemaphoreTake(*_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    poseCopy = *_pose;
    xSemaphoreGive(*_mutex);
  }

  char toastCopy[128];
  toastCopy[0] = '\0';
  if (_toastPending) {
    strncpy(toastCopy, _toast, sizeof(toastCopy) - 1);
    toastCopy[sizeof(toastCopy) - 1] = '\0';
    _toastPending = false;
    _toast[0] = '\0';
  }

  char out[384];
  if (toastCopy[0] != '\0') {
    // Mensagens de toast sao controladas no firmware; nao recebem aspas internas.
    snprintf(out, sizeof(out),
             "{\"type\":\"telemetry\",\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,\"tof\":%u,\"active\":%s,\"toast\":\"%s\"}",
             poseCopy.x, poseCopy.y, poseCopy.theta, tofMm, navActive ? "true" : "false", toastCopy);
  } else {
    snprintf(out, sizeof(out),
             "{\"type\":\"telemetry\",\"x\":%.4f,\"y\":%.4f,\"theta\":%.4f,\"tof\":%u,\"active\":%s}",
             poseCopy.x, poseCopy.y, poseCopy.theta, tofMm, navActive ? "true" : "false");
  }
  _ws.textAll(out);
}

void WebPortal::sendMap() {
  // Mapa compacto: 1 caractere por celula. 0=livre, 1=parede estatica, 2=obstaculo dinamico.
  char out[1280];
  size_t n = snprintf(out, sizeof(out), "{\"type\":\"map\",\"w\":%u,\"h\":%u,\"map\":\"",
                      RobotConfig::MAP_WIDTH, RobotConfig::MAP_HEIGHT);
  for (uint16_t y = 0; y < RobotConfig::MAP_HEIGHT && n < sizeof(out) - 4; y++) {
    for (uint16_t x = 0; x < RobotConfig::MAP_WIDTH && n < sizeof(out) - 4; x++) {
      out[n++] = static_cast<char>('0' + _map.cellForTelemetry(x, y));
    }
  }
  if (n < sizeof(out) - 3) {
    out[n++] = '"';
    out[n++] = '}';
    out[n] = '\0';
    _ws.textAll(out);
  }
}

void WebPortal::sendPath() {
  PathBuffer pathCopy;
  pathCopy.count = 0;
  if (_mutex != nullptr && xSemaphoreTake(*_mutex, pdMS_TO_TICKS(5)) == pdTRUE) {
    pathCopy = *_path;
    xSemaphoreGive(*_mutex);
  }

  char out[4096];
  size_t n = snprintf(out, sizeof(out), "{\"type\":\"path\",\"path\":[");
  for (uint16_t i = 0; i < pathCopy.count && n < sizeof(out) - 64; i++) {
    n += snprintf(out + n, sizeof(out) - n, "%s{\"x\":%.3f,\"y\":%.3f}",
                  i == 0 ? "" : ",", pathCopy.points[i].x, pathCopy.points[i].y);
  }
  if (n < sizeof(out) - 3) {
    out[n++] = ']';
    out[n++] = '}';
    out[n] = '\0';
    _ws.textAll(out);
  }
}
