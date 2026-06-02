#pragma once

#include <Arduino.h>
#include <Preferences.h>
#include <esp_system.h>

class BrownoutLogger {
private:
	Preferences preferences;
	esp_reset_reason_t lastResetReason;
	uint32_t bootCount;
	uint32_t brownoutCount;
	String cachedEvents;

	BrownoutLogger();
	BrownoutLogger(const BrownoutLogger&) = delete;
	BrownoutLogger& operator=(const BrownoutLogger&) = delete;

	void appendBrownoutEvent();
	String buildEventLine(uint32_t eventNumber) const;
	static String trimStoredEvents(const String& events, size_t maxLen);

public:
	static BrownoutLogger& getInstance();

	void begin();
	void clear();
	String toJson() const;
	bool wasBrownoutReset() const;
	uint32_t getBootCount() const;
	uint32_t getBrownoutCount() const;
	String getLastResetReasonText() const;
	static const char* resetReasonToText(esp_reset_reason_t reason);
};
