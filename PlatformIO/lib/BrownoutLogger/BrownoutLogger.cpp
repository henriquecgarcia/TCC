#include "BrownoutLogger.h"
#include <ArduinoJson.h>

static constexpr const char* BROWNOUT_NAMESPACE = "brownout";
static constexpr const char* KEY_BOOT_COUNT = "boots";
static constexpr const char* KEY_BROWNOUT_COUNT = "bouts";
static constexpr const char* KEY_EVENTS = "events";
static constexpr size_t MAX_EVENT_STORE_LEN = 1700;
static constexpr size_t MAX_EVENTS_IN_JSON = 24;

BrownoutLogger::BrownoutLogger()
	: lastResetReason(ESP_RST_UNKNOWN), bootCount(0), brownoutCount(0), cachedEvents("") {
}

BrownoutLogger& BrownoutLogger::getInstance() {
	static BrownoutLogger instance;
	return instance;
}

void BrownoutLogger::begin() {
	lastResetReason = esp_reset_reason();

	if (!preferences.begin(BROWNOUT_NAMESPACE, false)) {
		Serial.println("[BrownoutLogger] Falha ao abrir Preferences/NVS.");
		return;
	}

	bootCount = preferences.getUInt(KEY_BOOT_COUNT, 0) + 1;
	brownoutCount = preferences.getUInt(KEY_BROWNOUT_COUNT, 0);
	cachedEvents = preferences.getString(KEY_EVENTS, "");

	preferences.putUInt(KEY_BOOT_COUNT, bootCount);

	if (wasBrownoutReset()) {
		appendBrownoutEvent();
	}

	Serial.printf("[BrownoutLogger] Boot #%lu | Ultimo reset: %s | Brownouts: %lu\n",
		static_cast<unsigned long>(bootCount),
		resetReasonToText(lastResetReason),
		static_cast<unsigned long>(brownoutCount));
}

void BrownoutLogger::appendBrownoutEvent() {
	brownoutCount += 1;
	String line = buildEventLine(brownoutCount);

	if (cachedEvents.length() > 0 && !cachedEvents.endsWith("\n")) {
		cachedEvents += "\n";
	}
	cachedEvents += line;
	cachedEvents = trimStoredEvents(cachedEvents, MAX_EVENT_STORE_LEN);

	preferences.putUInt(KEY_BROWNOUT_COUNT, brownoutCount);
	preferences.putString(KEY_EVENTS, cachedEvents);
}

String BrownoutLogger::buildEventLine(uint32_t eventNumber) const {
	String line;
	line.reserve(96);
	line += "event=";
	line += eventNumber;
	line += ";boot=";
	line += bootCount;
	line += ";uptime_ms=";
	line += millis();
	line += ";reason=";
	line += resetReasonToText(lastResetReason);
	return line;
}

String BrownoutLogger::trimStoredEvents(const String& events, size_t maxLen) {
	if (events.length() <= maxLen) {
		return events;
	}

	String trimmed = events.substring(events.length() - maxLen);
	int firstNewLine = trimmed.indexOf('\n');
	if (firstNewLine >= 0 && firstNewLine + 1 < static_cast<int>(trimmed.length())) {
		trimmed = trimmed.substring(firstNewLine + 1);
	}
	return trimmed;
}

void BrownoutLogger::clear() {
	brownoutCount = 0;
	cachedEvents = "";
	preferences.putUInt(KEY_BROWNOUT_COUNT, brownoutCount);
	preferences.putString(KEY_EVENTS, cachedEvents);
}

String BrownoutLogger::toJson() const {
	DynamicJsonDocument doc(4096);
	doc["bootCount"] = bootCount;
	doc["brownoutCount"] = brownoutCount;
	doc["lastResetReason"] = resetReasonToText(lastResetReason);
	doc["lastResetWasBrownout"] = wasBrownoutReset();
	doc["note"] = "Brownout e registrado apos o ESP32 reiniciar com reset reason ESP_RST_BROWNOUT.";

	JsonArray events = doc["events"].to<JsonArray>();
	int start = 0;
	size_t added = 0;

	while (start < static_cast<int>(cachedEvents.length()) && added < MAX_EVENTS_IN_JSON) {
		int end = cachedEvents.indexOf('\n', start);
		if (end < 0) {
			end = cachedEvents.length();
		}

		String line = cachedEvents.substring(start, end);
		line.trim();
		if (line.length() > 0) {
			JsonObject event = events.createNestedObject();
			int cursor = 0;
			while (cursor < static_cast<int>(line.length())) {
				int sep = line.indexOf(';', cursor);
				if (sep < 0) sep = line.length();
				String part = line.substring(cursor, sep);
				int eq = part.indexOf('=');
				if (eq > 0) {
					String key = part.substring(0, eq);
					String value = part.substring(eq + 1);
					if (key == "event" || key == "boot" || key == "uptime_ms") {
						event[key] = value.toInt();
					} else {
						event[key] = value;
					}
				}
				cursor = sep + 1;
			}
			added++;
		}
		start = end + 1;
	}

	String output;
	output.reserve(2048);
	serializeJson(doc, output);
	return output;
}

bool BrownoutLogger::wasBrownoutReset() const {
	return lastResetReason == ESP_RST_BROWNOUT;
}

uint32_t BrownoutLogger::getBootCount() const {
	return bootCount;
}

uint32_t BrownoutLogger::getBrownoutCount() const {
	return brownoutCount;
}

String BrownoutLogger::getLastResetReasonText() const {
	return String(resetReasonToText(lastResetReason));
}

const char* BrownoutLogger::resetReasonToText(esp_reset_reason_t reason) {
	switch (reason) {
		case ESP_RST_POWERON: return "POWERON";
		case ESP_RST_EXT: return "EXT";
		case ESP_RST_SW: return "SW";
		case ESP_RST_PANIC: return "PANIC";
		case ESP_RST_INT_WDT: return "INT_WDT";
		case ESP_RST_TASK_WDT: return "TASK_WDT";
		case ESP_RST_WDT: return "WDT";
		case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
		case ESP_RST_BROWNOUT: return "BROWNOUT";
		case ESP_RST_SDIO: return "SDIO";
		default: return "UNKNOWN";
	}
}
