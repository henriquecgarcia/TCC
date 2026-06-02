#pragma once

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <functional>

class WebInterface {
private:
	AsyncWebServer* server;
	AsyncWebSocket* carSocket;
	AsyncWebSocket* webLogSocket;
	std::function<void(const String&, AsyncWebSocketClient*)> carCommandHandler;
	std::function<void(const String&, AsyncWebSocketClient*)> webCommandHandler;

	// Construtor privado para forçar Singleton
	WebInterface();

	// Evita cópia e movimentação
	WebInterface(const WebInterface&) = delete;
	WebInterface(WebInterface&&) = delete;
	WebInterface& operator=(const WebInterface&) = delete;
	WebInterface& operator=(WebInterface&&) = delete;

public:
	// Acesso único à instância
	static WebInterface& getInstance();

	void begin();
	void handleCommand(const String& cmd);
	void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len);
	// Set handlers called when a text message is received on the respective websocket
	void setCarCommandHandler(std::function<void(const String&, AsyncWebSocketClient*)> handler) { carCommandHandler = handler; }
	void setWebCommandHandler(std::function<void(const String&, AsyncWebSocketClient*)> handler) { webCommandHandler = handler; }

	void sendCarStatus(AsyncWebSocketClient* client = nullptr, const String& carStatusJson = String());
	void sendMapSnapshot(AsyncWebSocketClient* client = nullptr, const String& mapData = String());
	void log(const String& msg);
	void log(const char* msg);
	AsyncWebServer* getServer() { return server; }
	AsyncWebSocket* getCarSocket() { return carSocket; }
	AsyncWebSocket* getWebLogSocket() { return webLogSocket; }
};