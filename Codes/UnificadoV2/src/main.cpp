#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <ESPmDNS.h>
#include <SPIFFS.h>
#include <ArduinoJson.h>
#include "SharedTypes.h"
#include "../constants/RobotConfig.h"
#include <MotorDriver.h>
#include <Encoder.h>
#include <DriveBase.h>
#include <ImuSensor.h>
#include <ToFSensor.h>
#include <GridMap.h>
#include <AStarPlanner.h>
#include <PurePursuit.h>
#include <EKF.h>
#include <WebPortal.h>
#include <PowerManager.h>

// Objetos de hardware e algoritmos globais, alocados estaticamente para evitar fragmentacao.
MotorDriver leftMotor(RobotConfig::LEFT_IN1, RobotConfig::LEFT_IN2, RobotConfig::LEFT_PWM, RobotConfig::LEFT_PWM_CH);
MotorDriver rightMotor(RobotConfig::RIGHT_IN1, RobotConfig::RIGHT_IN2, RobotConfig::RIGHT_PWM, RobotConfig::RIGHT_PWM_CH);
Encoder leftEncoder(RobotConfig::LEFT_ENC_A, RobotConfig::LEFT_ENC_B);
Encoder rightEncoder(RobotConfig::RIGHT_ENC_A, RobotConfig::RIGHT_ENC_B);
DriveBase drive(leftMotor, rightMotor, leftEncoder, rightEncoder);
ImuSensor imu;
ToFSensor tof;
GridMap gridMap;
AStarPlanner planner(gridMap);
PurePursuit purePursuit;
EKF ekf;

SemaphoreHandle_t stateMutex;
Pose2D sharedPose;
SensorSample sharedSensors;
WheelSample sharedWheels;
PathBuffer sharedPath;
NavGoal sharedGoal;

volatile bool replanRequested = false;
bool robotActive = false;
uint32_t lastCommandMs = 0;
PowerManager powerManager;
WebPortal webPortal(gridMap, &sharedPose, &sharedPath, &stateMutex);

static float targetLeftMps = 0.0f;
static float targetRightMps = 0.0f;

// Estado dos comandos manuais simples enviados pela interface web.
enum ManualMode : uint8_t {
	MANUAL_NONE = 0,
	MANUAL_LINEAR = 1,
	MANUAL_TURN = 2
};

static volatile ManualMode manualMode = MANUAL_NONE;
static float manualLinearSpeedMps = 0.0f;
static float manualTurnTargetRad = 0.0f;
static uint32_t manualEndMs = 0;

void stopMotionOutputs() {
	manualMode = MANUAL_NONE;
	targetLeftMps = 0.0f;
	targetRightMps = 0.0f;
	drive.stop();
}

void clearAutonomousNavigation() {
	sharedGoal.active = false;
	sharedPath.count = 0;
	replanRequested = false;
}

void startManualLinear(float speedMps) {
	if (speedMps > RobotConfig::MAX_LINEAR_SPEED_MPS) speedMps = RobotConfig::MAX_LINEAR_SPEED_MPS;
	if (speedMps < -RobotConfig::MAX_LINEAR_SPEED_MPS) speedMps = -RobotConfig::MAX_LINEAR_SPEED_MPS;
	clearAutonomousNavigation();
	manualLinearSpeedMps = speedMps;
	manualEndMs = millis() + RobotConfig::MANUAL_LINEAR_DURATION_MS;
	manualMode = MANUAL_LINEAR;
	robotActive = true;
}

void startManualTurn(float deltaRad) {
	clearAutonomousNavigation();
	manualTurnTargetRad = normalizeAngle(ekf.pose().theta + deltaRad);
	manualEndMs = millis() + RobotConfig::MANUAL_TURN_TIMEOUT_MS;
	manualMode = MANUAL_TURN;
	robotActive = true;
}

// Atualiza destino e comandos recebidos via WebSocket.
bool handleWebCommand(const char* payload, size_t len) {
	if (payload == nullptr || len == 0 || len > 512) return false;
	JsonDocument doc;
	if (deserializeJson(doc, payload, len) != DeserializationError::Ok) return false;

	const char* cmd = doc["cmd"] | "";
	lastCommandMs = millis();
	powerManager.forceNormal();

	if (strcmp(cmd, "goal") == 0) {
		const int gx = doc["gx"] | -1;
		const int gy = doc["gy"] | -1;
		if (gx < 0 || gy < 0 || gx >= RobotConfig::MAP_WIDTH || gy >= RobotConfig::MAP_HEIGHT) {
			webPortal.showToast("Destino fora dos limites do mapa.");
			return false;
		}
		if (gridMap.isOccupied(gx, gy)) {
			webPortal.showToast("Destino ocupado por parede ou obstaculo.");
			return false;
		}
		if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
			manualMode = MANUAL_NONE;
			sharedGoal.gx = gx;
			sharedGoal.gy = gy;
			sharedGoal.active = true;
			robotActive = true;
			replanRequested = true;
			xSemaphoreGive(stateMutex);
		}
		return true;
	}

	if (strcmp(cmd, "pose") == 0) {
		const float x = doc["x"] | 0.05f;
		const float y = doc["y"] | 0.05f;
		const float theta = doc["theta"] | 0.0f;
		ekf.reset(x, y, theta);
		if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
			sharedPose = ekf.pose();
			xSemaphoreGive(stateMutex);
		}
		replanRequested = true;
		return true;
	}

	if (strcmp(cmd, "stop") == 0) {
		robotActive = false;
		clearAutonomousNavigation();
		stopMotionOutputs();
		return true;
	}

	if (strcmp(cmd, "forward") == 0) {
		startManualLinear(RobotConfig::MANUAL_LINEAR_SPEED_MPS);
		webPortal.showToast("Movendo para frente. Use PARADA para interromper.");
		return true;
	}

	if (strcmp(cmd, "backward") == 0) {
		startManualLinear(-RobotConfig::MANUAL_LINEAR_SPEED_MPS);
		webPortal.showToast("Movendo para tras. Use PARADA para interromper.");
		return true;
	}

	if (strcmp(cmd, "turnRight90") == 0) {
		startManualTurn(-HALF_PI);
		return true;
	}

	if (strcmp(cmd, "turnLeft90") == 0) {
		startManualTurn(HALF_PI);
		return true;
	}

	if (strcmp(cmd, "clearDynamic") == 0) {
		gridMap.clearDynamic();
		webPortal.markMapDirty();
		replanRequested = true;
		return true;
	}

	return false;
}

// Injeta obstaculo dinamico a frente do robo e solicita replanejamento.
void injectObstacleIfNeeded(const Pose2D& pose, const SensorSample& sensors) {
	if (!sensors.distanceValid || sensors.distanceMm > RobotConfig::OBSTACLE_MM) return;

	const float distM = sensors.distanceMm / 1000.0f;
	const float ox = pose.x + cosf(pose.theta) * distM;
	const float oy = pose.y + sinf(pose.theta) * distM;
	uint16_t gx, gy;
	if (!worldToGrid(ox, oy, gx, gy)) return;

	bool changed = false;
	for (int8_t yy = -1; yy <= 1; yy++) {
		for (int8_t xx = -1; xx <= 1; xx++) {
			const int nx = static_cast<int>(gx) + xx;
			const int ny = static_cast<int>(gy) + yy;
			if (nx >= 0 && ny >= 0 && nx < RobotConfig::MAP_WIDTH && ny < RobotConfig::MAP_HEIGHT) {
				changed |= gridMap.setDynamic(nx, ny, true);
			}
		}
	}
	if (changed) {
		webPortal.markMapDirty();
		targetLeftMps = 0.0f;
		targetRightMps = 0.0f;
		replanRequested = true;
	}
}

void sensorsTask(void* parameter) {
	(void)parameter;
	TickType_t lastWake = xTaskGetTickCount();
	uint32_t lastMs = millis();
	while (true) {
		const uint32_t now = millis();
		const float dt = (now - lastMs) / 1000.0f;
		lastMs = now;

		uint16_t mm = 8191;
		const bool valid = tof.read(mm);
		const SensorSample s = imu.read(mm, valid, dt);

		if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
			sharedSensors = s;
			xSemaphoreGive(stateMutex);
		}
		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(RobotConfig::SENSOR_PERIOD_MS));
	}
}

void navigationTask(void* parameter) {
	(void)parameter;
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		Pose2D pose;
		SensorSample sensors;
		NavGoal goal;

		if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
			pose = sharedPose;
			sensors = sharedSensors;
			goal = sharedGoal;
			xSemaphoreGive(stateMutex);
		}

		injectObstacleIfNeeded(pose, sensors);

		if (goal.active && replanRequested) {
			uint16_t sx, sy;
			if (worldToGrid(pose.x, pose.y, sx, sy)) {
				PathBuffer newPath;
				if (planner.plan(sx, sy, goal.gx, goal.gy, newPath)) {
					if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
						sharedPath = newPath;
						replanRequested = false;
						webPortal.markPathDirty();
						xSemaphoreGive(stateMutex);
					}
				} else {
					robotActive = false;
					clearAutonomousNavigation();
					stopMotionOutputs();
					webPortal.showToast("Nao foi possivel gerar um caminho ate o destino.");
				}
			}
		}

		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(RobotConfig::NAV_PERIOD_MS));
	}
}

void controlTask(void* parameter) {
	(void)parameter;
	TickType_t lastWake = xTaskGetTickCount();
	uint32_t lastMs = millis();
	float imuYawRad = 0.0f;
	while (true) {
		const uint32_t now = millis();
		float dt = (now - lastMs) / 1000.0f;
		lastMs = now;
		if (dt <= 0.0f || dt > 0.2f) dt = RobotConfig::CONTROL_PERIOD_MS / 1000.0f;

		SensorSample sensors;
		PathBuffer path;
		Pose2D pose = ekf.pose();
		bool active = robotActive;

		if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
			sensors = sharedSensors;
			path = sharedPath;
			xSemaphoreGive(stateMutex);
		}

		if (manualMode == MANUAL_LINEAR) {
			// Movimento manual simples por tempo fixo. Mantem o motor ativo mesmo sem rota A*.
			// if (now >= manualEndMs) {
			//	 manualMode = MANUAL_NONE;
			//	 robotActive = false;
			//	 targetLeftMps = 0.0f;
			//	 targetRightMps = 0.0f;
			//	 drive.stop();
			// } else {
				targetLeftMps = manualLinearSpeedMps;
				targetRightMps = manualLinearSpeedMps;
			// }
			imuYawRad = sensors.yawRad;
		} else if (manualMode == MANUAL_TURN) {
			// Giro manual de 90 graus usando o theta estimado pelo EKF/MPU6050, com timeout de seguranca.
			const float error = normalizeAngle(manualTurnTargetRad - pose.theta);
			if (fabsf(error) <= RobotConfig::MANUAL_TURN_TOLERANCE_RAD || now >= manualEndMs) {
				manualMode = MANUAL_NONE;
				robotActive = false;
				targetLeftMps = 0.0f;
				targetRightMps = 0.0f;
			} else {
				const float sign = error > 0.0f ? 1.0f : -1.0f;
				targetLeftMps = -sign * RobotConfig::MANUAL_TURN_SPEED_MPS;
				targetRightMps = sign * RobotConfig::MANUAL_TURN_SPEED_MPS;
			}
			imuYawRad = sensors.yawRad;
		} else if (active && path.count > 0) {
			bool goalReached = false;
			if (!purePursuit.compute(pose, path, RobotConfig::MAX_LINEAR_SPEED_MPS, targetLeftMps, targetRightMps, goalReached) || goalReached) {
				targetLeftMps = 0.0f;
				targetRightMps = 0.0f;
				robotActive = false;
				sharedGoal.active = false;
			}
			imuYawRad = sensors.yawRad;
		} else {
			targetLeftMps = 0.0f;
			targetRightMps = 0.0f;
		}

		sharedWheels = drive.update(targetLeftMps, targetRightMps, dt, imuYawRad);
		ekf.predict(sharedWheels.leftSpeedMps, sharedWheels.rightSpeedMps, dt);
		ekf.updateTheta(imuYawRad);
		sharedPose = ekf.pose();

		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(RobotConfig::CONTROL_PERIOD_MS));
	}
}

void webTask(void* parameter) {
	(void)parameter;
	TickType_t lastWake = xTaskGetTickCount();
	while (true) {
		powerManager.loop();
		{
			webPortal.loop();
			uint16_t tofMm = 0;
			if (xSemaphoreTake(stateMutex, pdMS_TO_TICKS(5)) == pdTRUE) {
				tofMm = sharedSensors.distanceMm;
				xSemaphoreGive(stateMutex);
			}
			webPortal.broadcastTelemetry(robotActive, tofMm);
		}
		vTaskDelayUntil(&lastWake, pdMS_TO_TICKS(RobotConfig::TELEMETRY_PERIOD_MS));
	}
}

void setupWiFi() {
	WiFi.mode(WIFI_AP);
	WiFi.setSleep(false);
	WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
	WiFi.softAP(RobotConfig::WIFI_SSID, RobotConfig::WIFI_PASSWORD, 6, false, 2);
	if (MDNS.begin(RobotConfig::MDNS_NAME)) {
		MDNS.addService("http", "tcp", 80);
	}
}

void setup() {
	Serial.begin(115200);
	delay(300);

	stateMutex = xSemaphoreCreateMutex();
	if (stateMutex == nullptr) {
		Serial.println("ERRO: falha ao criar mutex.");
		while (true) delay(1000);
	}

	Wire.begin(RobotConfig::I2C_SDA, RobotConfig::I2C_SCL);

	if (!SPIFFS.begin(true)) {
		Serial.println("ERRO: falha ao montar SPIFFS.");
		while (true) delay(1000);
	}

	drive.begin();
	if (!imu.begin()) Serial.println("AVISO: IMU indisponivel; navegacao degradada.");
	if (!tof.begin()) Serial.println("AVISO: ToF indisponivel; avoidance degradado.");
	if (!gridMap.begin(RobotConfig::MAP_FILE)) Serial.println("AVISO: mapa estatico invalido.");

	sharedPose = ekf.pose();
	sharedSensors = {0.0f, 0.0f, 8191, false};
	sharedPath.count = 0;
	sharedGoal = {0, 0, false};
	lastCommandMs = millis();

	setupWiFi();
	powerManager.begin(&lastCommandMs, &robotActive);

	webPortal.begin(handleWebCommand);

	xTaskCreatePinnedToCore(sensorsTask, "sensores", 4096, nullptr, 3, nullptr, 0);
	xTaskCreatePinnedToCore(navigationTask, "navegacao", 8192, nullptr, 2, nullptr, 0);
	xTaskCreatePinnedToCore(controlTask, "controle", 6144, nullptr, 4, nullptr, 1);
	xTaskCreatePinnedToCore(webTask, "websocket", 12288, nullptr, 1, nullptr, 1);

	Serial.println("Robo ESP32 autonomo iniciado.");
	Serial.print("AP: ");
	Serial.println(RobotConfig::WIFI_SSID);
}

void loop() {
	// O loop principal fica livre; as tarefas FreeRTOS executam o sistema em dois nucleos.
	vTaskDelay(pdMS_TO_TICKS(1000));
}
