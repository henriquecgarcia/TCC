
#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include "WebInterface.h"

// Meyers singleton
WebInterface& WebInterface::getInstance() {
	static WebInterface instance;
	return instance;
}

WebInterface::WebInterface()
	: server(nullptr), carSocket(nullptr), webLogSocket(nullptr) {
}

void WebInterface::begin() {
	if (!SPIFFS.begin(true)) {
		Serial.println("SPIFFS mount failed in WebInterface");
	}

	server = new AsyncWebServer(80);
	carSocket = new AsyncWebSocket("/car");
	webLogSocket = new AsyncWebSocket("/ws");

	carSocket->onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
		this->onWebSocketEvent(server, client, type, arg, data, len);
	});

	webLogSocket->onEvent([this](AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len){
		// logs currently don't expect messages from client, but keep handler for completeness
		this->onWebSocketEvent(server, client, type, arg, data, len);
	});

	server->addHandler(carSocket);
	server->addHandler(webLogSocket);

	// Serve files from SPIFFS root
	server->serveStatic("/", SPIFFS, "/").setDefaultFile("dashboard.html");

	server->begin();
	Serial.println("WebInterface: server started");
}

void WebInterface::onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
	if (type == WS_EVT_CONNECT) {
		Serial.printf("WebSocket client connected: %u\n", client->id());
		// Send initial state if needed
		sendCarStatus(client);
		sendMapSnapshot(client);
	} else if (type == WS_EVT_DISCONNECT) {
		Serial.printf("WebSocket client disconnected: %u\n", client->id());
	} else if (type == WS_EVT_DATA) {
		AwsFrameInfo *info = (AwsFrameInfo*)arg;
		if (info->final && info->len == len && info->opcode == WS_TEXT) {
			String msg((char*)data, len);
			// Dispatch to per-socket handlers if configured
			if (server == carSocket) {
				if (carCommandHandler) carCommandHandler(msg, client);
				else handleCommand(msg);
			} else if (server == webLogSocket) {
				if (webCommandHandler) webCommandHandler(msg, client);
				else handleCommand(msg);
			} else {
				handleCommand(msg);
			}
		}
	}
}

void WebInterface::handleCommand(const String& cmd) {
	// Placeholder: apenas loga por enquanto. O projeto pode expandir parsing JSON/commands.
	Serial.print("WebInterface received command: ");
	Serial.println(cmd);
}

void WebInterface::sendCarStatus(AsyncWebSocketClient* client, const String& carStatusJson) {
	String status = carStatusJson;
	if (carSocket == nullptr) return;
	if (client) {
		client->text(status);
	} else {
		carSocket->textAll(status);
	}
}

void WebInterface::sendMapSnapshot(AsyncWebSocketClient* client, const String& mapData) {
	String mapJson = "{\"map\": " + mapData + "}"; // placeholder
	if (carSocket == nullptr) return;
	if (client) {
		client->text(mapJson);
	} else {
		carSocket->textAll(mapJson);
	}
}

void WebInterface::log(const String& msg) {
	Serial.print(msg);
	Serial.print('\n');
	if (webLogSocket) webLogSocket->textAll(msg);
}

void WebInterface::log(const char* msg) {
	Serial.print(msg);
	Serial.print('\n');
	if (webLogSocket) webLogSocket->textAll(String(msg));
}


