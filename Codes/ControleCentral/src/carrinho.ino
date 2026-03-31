#define DEBUG_PRINTS
#include "Constants.h"

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <ArduinoJson.h>

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

#include "LED.h"
#include "MediaMovel.h"
#include "Map.h"
#include "odometry.h"
#include "PowerManager.h"

const char* ssid = "Canguru";
const char* password = "VamoPula";
const char* otaHostname = "espcarrinho";
const char* otaPassword = "VamoPula";

const double intentKp = 1.0;
const double intentKi = 0.5;
const double intentKd = 0.0;

static const unsigned int GRID_WIDTH = 120;
static const unsigned int GRID_HEIGHT = 120;
static const int ENCODER_TEETH = 10;

static const size_t PATH_BUFFER_SIZE = 512;
static const unsigned int MAP_VIEW_WIDTH = 40;
static const unsigned int MAP_VIEW_HEIGHT = 24;

AsyncWebServer server(80);
AsyncWebSocket carSocket("/car");
AsyncWebSocket ws("/ws");

#pragma region Funções Auxiliares

static double clamp(double value, double min, double max) {
	return (value < min) ? min : (value > max) ? max : value;
}

static void webLog(const String& msg) {
	#ifdef DEBUG_PRINTS
		Serial.print(msg);
	#endif
	if (ws.count() > 0) {
		ws.textAll(msg);
	}
}

static void webLog(const char* msg) {
	#ifdef DEBUG_PRINTS
		Serial.print(msg);
	#endif
	if (ws.count() > 0) {
		ws.textAll(msg);
	}
}

void sendTelemetryFrame();
String getMapSnapshotJson();

#include "VL53L0X.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"

#pragma region "Classe MPU6050/Gryoscopio"

class MPU6050 {
private:
	Adafruit_MPU6050 sensor;  // sem new/delete
	unsigned long lastRead = 0;
	static const unsigned long readInterval = 20; // ms entre leituras

	MediaMovel gyroX{1};
	MediaMovel gyroY{1};
	MediaMovel gyroZ{1};

	// previne cópia acidental (dois objetos lutando pelo mesmo hardware)
	MPU6050(const MPU6050&) = delete;
	MPU6050& operator=(const MPU6050&) = delete;

	bool firstRead = true; // para evitar leituras iniciais erradas
	// offsets calibrados para evitar drift
	float offsetX = 0.06; // ajuste fino do giroscópio X
	float offsetY = -0.03; // ajuste fino do giroscópio Y
	float offsetZ = -0.03; // ajuste fino do giroscópio Z

	MediaMovel accelX{1};
	MediaMovel accelY{1};
	MediaMovel accelZ{1};
	float cachedTempC = 0.0f;

	float offsetAccelX = 0.0; // ajuste fino do acelerômetro X
	float offsetAccelY = 0.0; // ajuste fino do acelerômetro Y
	float offsetAccelZ = 0.0; // ajuste fino do acelerômetro Z

	// unsigned long lastWSUpdate = 0;

public:
	MPU6050() = default;  // construtor padrão

	void setup() {
		if (!sensor.begin()) {
			webLog("Erro ao iniciar o MPU6050! Verifique as conexões.\n Reiniciando ESP32 em 1 segundo...\n");
			delay(1000);
			ESP.restart();
			return;
		}
		sensor.setAccelerometerRange(MPU6050_RANGE_2_G);
		sensor.setGyroRange(MPU6050_RANGE_500_DEG);
		sensor.setFilterBandwidth(MPU6050_BAND_21_HZ);
	}

	sensors_event_t getAccelerometer() {
		sensors_event_t a, g, temp;
		sensor.getEvent(&a, &g, &temp);
		return a;
	}
	sensors_event_t getGyroscope() {
		sensors_event_t a, g, temp;
		sensor.getEvent(&a, &g, &temp);
		return g;
	}
	sensors_event_t getTemperature() {
		sensors_event_t a, g, temp;
		sensor.getEvent(&a, &g, &temp);
		return temp;
	}

	float getAccelerometerX() {
		return accelX.get();
	}
	float getAccelerometerY() {
		return accelY.get();
	}
	float getAccelerometerZ() {
		return accelZ.get();
	}

	float getGyroscopeX() {
		return gyroX.get();
	}
	float getGyroscopeY() {
		return gyroY.get();
	}
	float getGyroscopeZ() {
		return gyroZ.get();
	}
	float getTemperatureC() {
		return cachedTempC;
	}

	void loop() {
		unsigned long now = millis();
		if (now - lastRead < readInterval) {
			return;  // evita leituras muito frequentes
		}
		lastRead = now;

		// faz só uma chamada ao sensor por ciclo
		sensors_event_t a, g, temp;
		sensor.getEvent(&a, &g, &temp);
		cachedTempC = temp.temperature;

		if (firstRead) {
			// calibra offsets na primeira leitura
			offsetX = g.gyro.x;
			offsetY = g.gyro.y;
			offsetZ = g.gyro.z;
			firstRead = false;

			offsetAccelX = a.acceleration.x;
			offsetAccelY = a.acceleration.y;
			offsetAccelZ = a.acceleration.z;
			webLog("MPU6050 calibrado com offsets iniciais.\n");
		}

		// Atualiza as médias móveis com offsets calibrados
		gyroX.add(g.gyro.x - offsetX);
		gyroY.add(g.gyro.y - offsetY);
		gyroZ.add(g.gyro.z - offsetZ);
		accelX.add(a.acceleration.x - offsetAccelX);
		accelY.add(a.acceleration.y - offsetAccelY);
		accelZ.add(a.acceleration.z - offsetAccelZ);

		// if (now - lastWSUpdate >= 1000) { // atualiza a cada segundo
		// 	lastWSUpdate = now;
		// 	carSocket.textAll("{\"gyroX\": " + String(g.gyro.x - offsetX) + ", \"gyroY\": " + String(g.gyro.y - offsetY) + ", \"gyroZ\": " + String(g.gyro.z - offsetZ) + ", \"accelX\": " + String(a.acceleration.x - offsetAccelX) + ", \"accelY\": " + String(a.acceleration.y - offsetAccelY) + ", \"accelZ\": " + String(a.acceleration.z - offsetAccelZ) + ", \"tempC\": " + String(temp.temperature) + "}");
		// }
	}
};

#include "PID.h"

#pragma region "Ponte H e controle"

class PonteH {
private:
	// TODO: REFATORAR PARA A LOGICA SEPARADA DE CONTROLE DOS MOTORES.
	Movement currentMove = MOVEMENT_STOPPED;
	Movement lastTurnDirection = MOVEMENT_STOPPED;


	static constexpr double targetVelocityForward = (150.0 * 2.0 * M_PI) / 60.0; // 150 RPM convertido para rad/s
	static constexpr double targetVelocityTurn = (75.0 * 2.0 * M_PI) / 60.0; // 75 RPM convertido para rad/s

	bool isMoving = false;
	double turningAngleZ = 0.0; // acumulado em radianos
	double gyroZAtTurnStart = 0.0; // para calcular o quanto já virou
	unsigned long lastUpdate = 0;
	unsigned long lastSocketUpdate = 0;

	static const unsigned long controlInterval = 100; // ms entre controles

	double turnLimitRad = 0.0; // Sempre pra fente!
	unsigned long turnStartedAt = 0;
	unsigned long turnTimeoutMs = 4000;

	static constexpr double turnToleranceRad = 2.0 * (M_PI / 180.0);      // alvo: erro < 2 graus
	static constexpr double turnFineThresholdRad = 25.0 * (M_PI / 180.0); // troca coarse -> fine

	PID pidGyro = PID(0.1, 0.0, 0.0, controlInterval / 1000.0); // Kp, Ki, Kd para giroscópio

	PID pidTurn = PID(0.08, 0.00, 0.00, controlInterval / 1000.0); // controlador fino de curva (fase final)

	static double normalizeAngle(double angle) {
		angle = fmod(angle, 2 * M_PI);
		if (angle < 0.0) {
			angle += 2 * M_PI;
		}
		return angle;
	}

public:
	PonteH() {
		pidGyro.setTunning(0.1, 0.0, 0.0); // Kp, Ki, Kd
		pidGyro.setMaxMin(0.5, -0.5); // limites de correção
		pidTurn.setMaxMin(M_PI, 0.0); // limites de correcao na fase fina
	}

	void sendCommand(double targetRadSLeft, double targetRadSRight) {
		int16_t scaledLeft = static_cast<int16_t>( floor(targetRadSLeft * 1000.0) );
		int16_t scaledRight = static_cast<int16_t>( floor(targetRadSRight * 1000.0) );
		Wire.beginTransmission(MOTOR_CONTROLER_ESP32_ADDR);
		MotorCommand cmd;
		cmd.targetRadSLeft = scaledLeft;
		cmd.targetRadSRight = scaledRight;
		Wire.write((uint8_t*)&cmd, sizeof(MotorCommand));
		Wire.endTransmission();
	}

	void setup() {
		isMoving = false;
		currentMove = MOVEMENT_STOPPED;
		turningAngleZ = 0.0;
		turnLimitRad = 0.0;
		lastUpdate = millis();
		pidGyro.reset();
		pidTurn.reset();
		webLog("[PonteH] Configuração completa!\n");
	}

	void forward() {
		if (!isMoving || currentMove != MOVEMENT_FORWARD) {
			currentMove = MOVEMENT_FORWARD;
			sendCommand(targetVelocityForward, targetVelocityForward);
			isMoving = true;
			lastUpdate = millis();
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"forward\"}");
		}
	}

	void backward() {
		if (!isMoving || currentMove != MOVEMENT_BACKWARDS) {
			currentMove = MOVEMENT_BACKWARDS;
			sendCommand(-targetVelocityForward, -targetVelocityForward);
			isMoving = true;
			lastUpdate = millis();
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"backward\"}");
		}
	}

	void turnLeft(double degs = 90.0) {
		degs = fabs(degs);
		if (degs > 180.0)
			return turnRight(360.0 - degs); // "vire 270* para esquerda" = "vire 90* para direita"
		if (!isMoving || currentMove != MOVEMENT_TURN_LEFT) {
			sendCommand(targetVelocityTurn, -targetVelocityTurn);

			currentMove = MOVEMENT_TURN_LEFT;
			isMoving = true;
			lastUpdate = millis();

			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"left\"}");

			double turnRads = degs * (M_PI / 180.0);

			// Curva relativa: alvo desta manobra apenas (nao acumula alvo anterior)
			turnLimitRad = normalizeAngle(turnRads) - gyroZAtTurnStart * (lastTurnDirection == MOVEMENT_TURN_LEFT ? 1 : -1);
			turningAngleZ = 0.0;
			pidTurn.reset();

			double turnLimitDeg = turnLimitRad * (180.0 / M_PI);
			turnStartedAt = millis();
			webLog("[PonteH] Turn limit set to " + String(turnLimitDeg) + " degrees (" + String(turnLimitRad) + " radians)\n");
		}
	}

	void turnRight(double degs = 90.0) {
		degs = fabs(degs);
		if (degs > 180.0)
			return turnLeft(360.0 - degs); // "vire 270* para direita" = "vire 90* para esquerda"
		if (!isMoving || currentMove != MOVEMENT_TURN_RIGHT) {
			sendCommand(-targetVelocityTurn, targetVelocityTurn);

			currentMove = MOVEMENT_TURN_RIGHT;
			isMoving = true;
			lastUpdate = millis();

			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"right\"}");

			double turnRads = degs * (M_PI / 180.0);

			// Curva relativa: alvo desta manobra apenas (nao acumula alvo anterior)
			turnLimitRad = normalizeAngle(turnRads) - gyroZAtTurnStart * (lastTurnDirection == MOVEMENT_TURN_LEFT ? 1 : -1);
			turningAngleZ = 0.0;
			pidTurn.reset();

			double turnLimitDeg = turnLimitRad * (180.0 / M_PI);
			turnStartedAt = millis();
			webLog("[PonteH] Turn limit set to " + String(turnLimitDeg) + " degrees (" + String(turnLimitRad) + " radians)\n");
		}
	}

	void stop() {
		sendCommand(0.0, 0.0); // comando de parada imediato

		pidGyro.reset();
		// pidTurn.reset();
		turningAngleZ = 0.0;
		turnLimitRad = 0.0;

		isMoving = false;
		currentMove = MOVEMENT_STOPPED;
	}

	MotorStatus fetchMotorStatus() {
		MotorStatus status;

		Wire.requestFrom(MOTOR_CONTROLER_ESP32_ADDR, sizeof(MotorStatus));

		if (Wire.available() == sizeof(MotorStatus)) {
			uint8_t* ptr = (uint8_t*)&status;
			for (int i = 0; i < sizeof(MotorStatus); i++) {
				ptr[i] = Wire.read();
			}
		} else {
			webLog("[PonteH] Erro ao ler status dos motores: dados insuficientes recebidos\n");
			// Preenche o status com valores de erro
			status.radSLeft = -1.0;
			status.radSRight = -1.0;
			status.pwmLeft = -1.0;
			status.pwmRight = -1.0;
		}
		return status;
	}

	bool isStopped() const {
		return !isMoving || (currentMove == MOVEMENT_STOPPED); // Vai que eu esqueci de setar como parado em uma das duas, ai... Agora ta seguro :)
	}

	void loop(VL53L0X* frontDistSensor, MPU6050* mpuSensor) {
		if (!isMoving || !frontDistSensor || !mpuSensor) {
			return;
		}

		unsigned long now = millis();
		if (now - lastUpdate < controlInterval) {
			return;
		}
		double deltaTime = (now - lastUpdate) / 1000.0;  // em segundos
		lastUpdate = now;

		double gyroRead = mpuSensor->getGyroscopeZ();
		switch (currentMove) {
			case MOVEMENT_FORWARD: {
				// NTS: Com o pensamento descentralizado, isso continua basicamente igual, só adicionar o "SENDCOMMAND" com vel = 0;
				int frontDistance = frontDistSensor->loop();
				if (frontDistance < 100) {
					stop();
					#ifdef DEBUG_PRINTS
						Serial.println("Parando por obstáculo!");
					#endif
					webLog("[PonteH] Parando por obstáculo!\n");
					carSocket.textAll("{\"movement\": \"stopped\", \"reason\": \"obstacle\"}");
					break;
				}
				// Como é a mesma lógica para controle de "Frente" e "Trás", não colocamos BREAK, caindo direto para o proximo
			}
			case MOVEMENT_BACKWARDS: {
				const double gyroFixPid = pidGyro.compute(0.0, gyroRead); // calcula correção do giroscópio
				// PID direto com o targetVelocity aqui?
				sendCommand(
					(currentMove == MOVEMENT_BACKWARDS ? -targetVelocityForward : targetVelocityForward) + gyroFixPid,
					(currentMove == MOVEMENT_BACKWARDS ? -targetVelocityForward : targetVelocityForward) - gyroFixPid
				);
				break;
			}
			case MOVEMENT_TURN_LEFT:
			case MOVEMENT_TURN_RIGHT: {
				// TODO: REFAZER TODA ESSA PARTE, codigo merda que tem que ser modificado para descentralizar o controle dos motores
				double gz = gyroRead;

				if (currentMove == MOVEMENT_TURN_RIGHT) gz = -gz; // inverte para esquerda
				// anguloGirado = anguloGirado + giroInercialZ * deltaT
				turningAngleZ = turningAngleZ + (gz * deltaTime);
				const double turningAngleDeg = turningAngleZ * (180.0 / M_PI);

				double angleError = turnLimitRad - turningAngleZ;
				double absError = fabs(angleError);
				bool isFinePhase = (absError <= turnFineThresholdRad);

				double turnCorrection = pidTurn.compute(turnLimitRad, turningAngleZ);
				if (isFinePhase) {
					// // turnCorrection = pidTurn.compute(turnLimitRad, turningAngleZ);
					// motorRight->setTargetRPM(50.0); // reduz RPM para fase fina
					// motorLeft->setTargetRPM(50.0);
				}

				turnCorrection = turnCorrection * deltaTime;

				#ifdef DEBUG_PRINTS
					Serial.print("[PonteH] Turning Angle Z: ");
					Serial.print(turningAngleZ);
					Serial.print(" rad (");
					Serial.print(turningAngleDeg);
					Serial.printf(" deg) on deltaTime: %f\n", deltaTime);
					Serial.print("Angle Error (deg): ");
					Serial.println(angleError * (180.0 / M_PI));
					Serial.print("Turn phase: ");
					Serial.println(isFinePhase ? "fine" : "coarse");
					Serial.print("Turn correction: ");
					Serial.println(turnCorrection);
				#endif
				webLog("[PonteH] Turning Angle Z: " + String(turningAngleZ) +
					" rad (" + String(turningAngleDeg) + " deg) on deltaTime: " + String(deltaTime) +
					"\n[PonteH] Angle error: " + String(angleError * (180.0 / M_PI)) + " deg" +
					" | phase: " + String(isFinePhase ? "fine" : "coarse") +
					" | correction: " + String(turnCorrection) + "\n");
				carSocket.textAll("{\"turning_angleZ\": " + String(turningAngleZ) + ", \"turningAngleZ_deg\": " + String(turningAngleDeg) + ", \"delta_time\": " + String(deltaTime) + "}");

				if (absError <= turnToleranceRad || fabs(turnLimitRad) < turningAngleZ) {
					stop();
					lastTurnDirection = currentMove;
					gyroZAtTurnStart = angleError; // Acumula o erro para a próxima curva
					#ifdef DEBUG_PRINTS
						Serial.println("Parando por ângulo de giro alcançado!");
					#endif
					carSocket.textAll("{\"movement\": \"stopped\", \"reason\": \"target angle reached\"}");
					return;
				}

				if (now - turnStartedAt > turnTimeoutMs) {
					stop();
					lastTurnDirection = currentMove;
					gyroZAtTurnStart = angleError; // Acumula o erro para a próxima curva
					webLog("[PonteH] Curva interrompida por timeout de seguranca.\n");
					carSocket.textAll("{\"movement\": \"stopped\", \"reason\": \"turn timeout\"}");
					return;
				}

				// mantém direção definida e alterna update
				sendCommand(
					(currentMove == MOVEMENT_TURN_LEFT ? -targetVelocityTurn : targetVelocityTurn) + turnCorrection,
					(currentMove == MOVEMENT_TURN_LEFT ? -targetVelocityTurn : targetVelocityTurn) - turnCorrection
				);
				break;
			}
			default:
				break;
		}

		if (carSocket.count() > 0) {
			if (isMoving && now - lastSocketUpdate >= 100) {
				lastSocketUpdate = now;
				sendTelemetryFrame();
			}
		} else {
			#ifdef DEBUG_PRINTS
				Serial.println("[PonteH] Nenhum cliente conectado, não enviando dados.");
			#endif
		}

		// alterna para a próxima chamada
	}

	Movement getCurrentMove() const {
		return currentMove;
	}
};

bool ConnectToWiFi(unsigned long timeoutMs = 15000) {
	if (WiFi.status() == WL_CONNECTED) {
		return true;
	}

	WiFi.begin(ssid, password);
	unsigned long startedAt = millis();
	#ifdef DEBUG_PRINTS
		Serial.print("Conectando ao WiFi -> ");
		Serial.print(ssid);
	#endif
	while (WiFi.status() != WL_CONNECTED && (millis() - startedAt) < timeoutMs) {
		#ifdef DEBUG_PRINTS
			Serial.print(".");
		#endif
		delay(100);
	}

	const bool connected = (WiFi.status() == WL_CONNECTED);
	#ifdef DEBUG_PRINTS
		if (connected) {
			Serial.println(" WiFi conectado!");
			Serial.print("Endereco IP: ");
			Serial.println(WiFi.localIP());
		} else {
			Serial.println(" Falha ao conectar dentro do timeout.");
		}
	#endif
	return connected;
}

#pragma region "Global Objects"

// ——————— Sensores ———————
VL53L0X *sensor = new VL53L0X();	// VL53L0X no I²C (SDA=21, SCL=22)
MPU6050 *sensorMPU = new MPU6050();	// MPU6050 no I²C (SDA=21, SCL=22)

Map *robotMap = new Map(GRID_WIDTH, GRID_HEIGHT);
Odometry *robotOdom = new Odometry(robotMap);
Map::Position plannedPath[PATH_BUFFER_SIZE];
size_t plannedPathLen = 0;
bool hasPlannedPath = false;

// ——————— Ponte H (drive de 2 motores) ———————
PonteH *ponte = new PonteH();

// ——————— Outros ———————
// LED da carroceria (MQTT)
LED led_carro(2);
// Gerenciador de energia
PowerManager powerManager;

void sendTelemetryFrame() {
	if (carSocket.count() == 0) {
		return;
	}

	StaticJsonDocument<768> doc;
	doc["led"] = led_carro.isOn();
	if (ponte->isStopped()) {
		doc["status"] = "stopped";
	} else {
		doc["status"] = "moving";
		switch (ponte->getCurrentMove()) {
			case MOVEMENT_FORWARD:
				doc["direction"] = "forward";
				break;
			case MOVEMENT_BACKWARDS:
				doc["direction"] = "backward";
				break;
			case MOVEMENT_TURN_LEFT:
				doc["direction"] = "left";
				break;
			case MOVEMENT_TURN_RIGHT:
				doc["direction"] = "right";
				break;
			default:
				doc["direction"] = "unknown";
		}
	}

	JsonObject gyro = doc["gyro"].to<JsonObject>();
	gyro["x"] = sensorMPU->getGyroscopeX();
	gyro["y"] = sensorMPU->getGyroscopeY();
	gyro["z"] = sensorMPU->getGyroscopeZ();

	JsonObject accel = doc["accel"].to<JsonObject>();
	accel["x"] = sensorMPU->getAccelerometerX();
	accel["y"] = sensorMPU->getAccelerometerY();
	accel["z"] = sensorMPU->getAccelerometerZ();

	doc["temperature"] = sensorMPU->getTemperatureC();
	doc["distance"] = sensor->loop();

	// TODO: Dados dos motores tem que ser modificado e pedido para o ESCRAVO!
	MotorStatus motorStatus = ponte->fetchMotorStatus();
	JsonObject motorD = doc["motorD"].to<JsonObject>();
	motorD["rpm"] = motorStatus.radSRight * (60.0 / (2.0 * M_PI)); // converte de rad/s para RPM
	motorD["pwm"] = motorStatus.pwmRight;

	JsonObject motorE = doc["motorE"].to<JsonObject>();
	motorE["rpm"] = motorStatus.radSLeft * (60.0 / (2.0 * M_PI)); // converte de rad/s para RPM
	motorE["pwm"] = motorStatus.pwmLeft;

	JsonObject odom = doc["odometry"].to<JsonObject>();
	odom["x"] = robotOdom->getX();
	odom["y"] = robotOdom->getY();
	odom["theta"] = robotOdom->getTheta();
	odom["mapX"] = robotOdom->getMapX();
	odom["mapY"] = robotOdom->getMapY();

	String output;
	output.reserve(768);
	serializeJson(doc, output);
	carSocket.textAll(output);
}

String getMapSnapshotJson() {
	const Map::Position robotPos = robotMap->getPosition();
	const unsigned int mapWidth = robotMap->getWidth();
	const unsigned int mapHeight = robotMap->getHeight();

	if (mapWidth == 0 || mapHeight == 0) {
		return "{\"type\":\"map\",\"error\":\"empty_map\"}";
	}

	const unsigned int viewWidth = (MAP_VIEW_WIDTH < mapWidth) ? MAP_VIEW_WIDTH : mapWidth;
	const unsigned int viewHeight = (MAP_VIEW_HEIGHT < mapHeight) ? MAP_VIEW_HEIGHT : mapHeight;

	unsigned int startX = 0;
	unsigned int startY = 0;

	if (robotPos.x > (viewWidth / 2U)) {
		startX = robotPos.x - (viewWidth / 2U);
	}
	if (robotPos.y > (viewHeight / 2U)) {
		startY = robotPos.y - (viewHeight / 2U);
	}
	if (startX + viewWidth > mapWidth) {
		startX = mapWidth - viewWidth;
	}
	if (startY + viewHeight > mapHeight) {
		startY = mapHeight - viewHeight;
	}

	DynamicJsonDocument doc(16384);
	doc["type"] = "map";
	JsonObject mapObj = doc["map"].to<JsonObject>();
	mapObj["width"] = mapWidth;
	mapObj["height"] = mapHeight;
	mapObj["viewWidth"] = viewWidth;
	mapObj["viewHeight"] = viewHeight;
	mapObj["viewStartX"] = startX;
	mapObj["viewStartY"] = startY;

	JsonObject robot = mapObj["robot"].to<JsonObject>();
	robot["x"] = robotPos.x;
	robot["y"] = robotPos.y;

	if (robotMap->hasTarget()) {
		const Map::Position target = robotMap->getTarget();
		JsonObject targetObj = mapObj["target"].to<JsonObject>();
		targetObj["x"] = target.x;
		targetObj["y"] = target.y;
	}

	JsonArray rows = mapObj["rows"].to<JsonArray>();
	for (unsigned int y = 0; y < viewHeight; ++y) {
		String row;
		row.reserve(viewWidth);
		const unsigned int ay = startY + y;
		for (unsigned int x = 0; x < viewWidth; ++x) {
			const unsigned int ax = startX + x;
			row += (robotMap->getCell(ax, ay) == 1U) ? '1' : '0';
		}
		rows.add(row);
	}

	JsonArray path = mapObj["path"].to<JsonArray>();
	if (hasPlannedPath) {
		for (size_t i = 0; i < plannedPathLen; ++i) {
			JsonObject step = path.add<JsonObject>();
			step["x"] = plannedPath[i].x;
			step["y"] = plannedPath[i].y;
		}
	}

	String output;
	output.reserve(8192);
	serializeJson(doc, output);
	return output;
}

// ——————— WebSocket ———————
// ——————— Car Command Handler ———————
String getCarStatus() {
	StaticJsonDocument<192> doc;
	doc["status"] = ponte->isStopped() ? "stopped" : "moving";
	if (!ponte->isStopped()) {
		switch (ponte->getCurrentMove()) {
			case MOVEMENT_FORWARD: doc["direction"] = "forward"; break;
			case MOVEMENT_BACKWARDS: doc["direction"] = "backward"; break;
			case MOVEMENT_TURN_LEFT: doc["direction"] = "left"; break;
			case MOVEMENT_TURN_RIGHT: doc["direction"] = "right"; break;
			default: doc["direction"] = "unknown"; break;
		}
	}

	doc["odometry"]["x"] = robotOdom->getX();
	doc["odometry"]["y"] = robotOdom->getY();
	doc["odometry"]["theta"] = robotOdom->getTheta();
	doc["odometry"]["mapX"] = robotOdom->getMapX();
	doc["odometry"]["mapY"] = robotOdom->getMapY();

	String output;
	output.reserve(192);
	serializeJson(doc, output);
	return output;
}
void handleCar(const String& cmd, AsyncWebSocketClient* client = nullptr) {
	if (cmd.length() > 0 && cmd[0] == '{') {
		StaticJsonDocument<256> request;
		DeserializationError error = deserializeJson(request, cmd);
		if (!error) {
			const char* action = request["action"] | "";
			if (strcmp(action, "path_to") == 0) {
				const long txRaw = request["x"] | -1;
				const long tyRaw = request["y"] | -1;

				if (txRaw < 0 || tyRaw < 0) {
					if (client) {
						client->text("{\"type\":\"path\",\"ok\":false,\"reason\":\"invalid_target\"}");
					}
					return;
				}

				const unsigned int tx = static_cast<unsigned int>(txRaw);
				const unsigned int ty = static_cast<unsigned int>(tyRaw);

				if (!robotMap->setTarget(tx, ty)) {
					if (client) {
						client->text("{\"type\":\"path\",\"ok\":false,\"reason\":\"target_out_of_bounds\"}");
					}
					return;
				}

				size_t outLen = 0;
				hasPlannedPath = robotMap->findPathAStar(tx, ty, plannedPath, PATH_BUFFER_SIZE, outLen);
				plannedPathLen = hasPlannedPath ? outLen : 0;

				DynamicJsonDocument response(16384);
				response["type"] = "path";
				response["ok"] = hasPlannedPath;
				if (!hasPlannedPath) {
					response["reason"] = "path_not_found";
				}

				JsonObject targetObj = response["target"].to<JsonObject>();
				targetObj["x"] = tx;
				targetObj["y"] = ty;

				JsonArray path = response["cells"].to<JsonArray>();
				if (hasPlannedPath) {
					for (size_t i = 0; i < plannedPathLen; ++i) {
						JsonObject step = path.add<JsonObject>();
						step["x"] = plannedPath[i].x;
						step["y"] = plannedPath[i].y;
					}
				}

				String payload;
				payload.reserve(12288);
				serializeJson(response, payload);
				if (client) {
					client->text(payload);
					client->text(getMapSnapshotJson());
				} else if (carSocket.count() > 0) {
					carSocket.textAll(payload);
					carSocket.textAll(getMapSnapshotJson());
				}
				return;
			}
		}
	}

	String normalized = cmd;
	normalized.toLowerCase();
	const String& command = normalized;
	if (command == "stop") {
		ponte->stop();
		webLog("Carro parado.\n");
	} else if (command == "right") {
		ponte->turnRight();
		webLog("Carro virando para a direita.\n");
	} else if (command == "left") {
		ponte->turnLeft();
		webLog("Carro virando para a esquerda.\n");
	} else if (command == "forward") {
		ponte->forward();
		webLog("Carro indo para frente.\n");
	} else if (command == "backward") {
		ponte->backward();
		webLog("Carro indo para trás.\n");
	} else if (command == "status") {
		String status = getCarStatus();
		if (carSocket.count() > 0) {
			// carSocket.binaryAll(status.c_str(), status.length());
			if (client) {
				client->text(status);
			} else {
				carSocket.textAll(status);
			}
		}
	} else if (command == "map") {
		String mapPayload = getMapSnapshotJson();
		if (client) {
			client->text(mapPayload);
		} else if (carSocket.count() > 0) {
			carSocket.textAll(mapPayload);
		}
	} else {
		webLog("Comando desconhecido: " + command + "\n");
	}
}
void handleCommand(const String& cmd) {
	String normalized = cmd;
	normalized.toLowerCase();
	const String& command = normalized;
	if (command == "led") {
		led_carro.toggle();
		webLog("LED Alterado, agora está: " + String(led_carro.isOn() ? "ON" : "OFF") + "\n");
	} else if (command == "reboot" || command == "restart") {
		webLog("Reiniciando o ESP...\n");
		delay(100);
		Serial.println("Reiniciando o ESP...\n");
		ESP.restart();
	} else if (command == "ip") {
		String ip = WiFi.localIP().toString();
		webLog("IP atual: " + ip + "\n");
	} else if (command == "status") {
		String status = getCarStatus();
		webLog("Status do carro: " + status + "\n");
	} else if (command == "data") {
		sendTelemetryFrame();
	} else {
		webLog("Comando desconhecido: " + command + "\n");
	}
}
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
	const char* sockName = "Unknown Socket";
	if (server == &carSocket) {
		sockName = "Car Socket";
	} else if (server == &ws) {
		sockName = "WebSocket";
	}
	switch (type) {
		case WS_EVT_CONNECT:
			Serial.printf("[%s] WebSocket client #%u connected from %s\n", sockName, client->id(), client->remoteIP().toString().c_str());
			webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Conectado de " + client->remoteIP().toString() + "\n");
			if (server == &carSocket) {
				Serial.printf("WebSocket [server #%s] client #%u is ready\n", sockName, client->id());
				client->text("{\"ready\":\"true\"}");
				client->text(getMapSnapshotJson());
			}
			break;
		case WS_EVT_DISCONNECT:
			if (server == &carSocket) {
				if (server->count() == 0) {
					ponte->stop();
				}
			}
			webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Desconectado\n");
			Serial.printf("WebSocket [server #%s] client #%u disconnected\n", sockName, client->id());
			break;
		case WS_EVT_DATA:
			AwsFrameInfo *info;
			info = (AwsFrameInfo*)arg;
			if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
				String command((const char*)data, len);
				Serial.printf("Comando recebido do cliente #%u: %s\n", client->id(), command.c_str());
				if (server == &carSocket) {
					handleCar(command, client);
					// webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Comando recebido: " + command + "\n");
				} else if (server == &ws) {
					webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Comando recebido: " + command + "\n");
					handleCommand(command);
				} else {
					webLog("["+ String(sockName) + "] Comando desconhecido: " + command + "\n");
				}
			}
			break;
		case WS_EVT_PONG:
		case WS_EVT_ERROR:
			break;
	}
}

bool isIdle() {
	return ponte->isStopped();
}

void prepare_http_server() {
	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (!SPIFFS.exists("/dashboard.html")) {
			request->send(404, "text/plain", "dashboard.html não encontrado");
			webLog("dashboard.html não encontrado\n");
			return;
		}
		request->send(SPIFFS, "/dashboard.html", String(), false);
	});
	server.on("/monitor", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (!SPIFFS.exists("/monitor.html")) {
			request->send(404, "text/plain", "monitor.html não encontrado");
			webLog("monitor.html não encontrado\n");
			return;
		}
		request->send(SPIFFS, "/monitor.html", String(), false);
	});
	server.onNotFound([](AsyncWebServerRequest *request) {
		webLog("Requisição não encontrada: " + request->url());
		request->send(404, "text/plain", "Not Found");
	});

	ws.onEvent(onWebSocketEvent);
	carSocket.onEvent(onWebSocketEvent);
	server.addHandler(&ws);
	server.addHandler(&carSocket);

	// Inicia servidor
	server.begin();
	#ifdef DEBUG_PRINTS
		Serial.println("Servidor HTTP iniciado");
	#endif
}

int last_sent_percent = 0;
void init_ota() {
	// Inicia o OTA
	ArduinoOTA.setHostname(otaHostname);
	ArduinoOTA.setPassword(otaPassword);

	ArduinoOTA.onStart([]() {
		ponte->stop();
		String type;
		if (ArduinoOTA.getCommand() == U_FLASH) {
			type = "sketch";
		} else { // U_SPIFFS
			type = "filesystem";
		}

		SPIFFS.end();
		webLog("Iniciando OTA para " + type + "\n");
	});
	ArduinoOTA.onEnd([]() {
		webLog("OTA concluída!\n");
	});
	ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
		int current_percent = (progress / (total / 100));
		String logMessage = "Recebendo atualização [";
		for (int i = 0 ; i < current_percent; i++) {
			logMessage += "=";
		}
		for (int i = current_percent; i < 100; i++) {
			logMessage += " ";
		}
		logMessage += "] " + String(current_percent) + "%\n";
		if (current_percent == last_sent_percent)
			Serial.print(logMessage);
		else
			webLog(logMessage);
		last_sent_percent = current_percent;
	});
	ArduinoOTA.onError([](ota_error_t error) {
		Serial.printf("Error[%u]: ", error);
		if (error == OTA_AUTH_ERROR) {
			webLog("Erro de autenticação!\n");
			Serial.println("Auth Failed");
		} else if (error == OTA_BEGIN_ERROR) {
			webLog("Erro ao iniciar atualização!\n");
			Serial.println("Begin Failed");
		} else if (error == OTA_CONNECT_ERROR) {
			webLog("Erro de conexão!\n");
			Serial.println("Connect Failed");
		} else if (error == OTA_RECEIVE_ERROR) {
			webLog("Erro ao receber dados!\n");
			Serial.println("Receive Failed");
		} else if (error == OTA_END_ERROR) {
			webLog("Erro ao finalizar atualização!\n");
			Serial.println("End Failed");
		}
	});

	ArduinoOTA.begin();
}

void setup() {
	Serial.begin(115200);

	#ifdef DEBUG_PRINTS
		Serial.println("Iniciando...");
	#endif
	powerManager.setPowerMode(POWER_NORMAL);

	led_carro.setup();

	if (!SPIFFS.begin(true)) {
		#ifdef DEBUG_PRINTS
			Serial.println("Falha ao montar SPIFFS");
		#endif
		LED inLed(2);
		inLed.setup();
		while (true) {
			inLed.toggle();
			delay(3000); 
		}
	}
	#ifdef DEBUG_PRINTS
		Serial.println("SPIFFS Mounted!");
	#endif

	ConnectToWiFi();

	if (!MDNS.begin(otaHostname)) {
		Serial.println("Erro ao iniciar mDNS");
	} else {
		Serial.println("mDNS iniciado!");
	}
	prepare_http_server();
	init_ota();

	ponte->setup();
	// TODO: Repensar odometria
	robotMap->generateStraightLineTest(MAP_ORIGIN_Y);
	robotMap->setPosition(MAP_ORIGIN_X, MAP_ORIGIN_Y);
	robotOdom->reset(0.0f, 0.0f, 0.0f);

	Wire.begin();
	sensor->setup();
	sensorMPU->setup();

	#ifdef DEBUG_PRINTS
		Serial.println("Setup completo!");
	#endif
}

unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 20;  // ms

void loop() {
	ArduinoOTA.handle();
	ws.cleanupClients();
	carSocket.cleanupClients();

	if (WiFi.status() != WL_CONNECTED) {
		#ifdef DEBUG_PRINTS
			Serial.println("WiFi desconectado, tentando reconectar...");
		#endif
		ponte->stop();
		ConnectToWiFi(5000);
		return;
	}

	powerManager.loop();

	unsigned long now = millis();
	if (now - lastLoopTime >= loopInterval) {
		lastLoopTime = now;

		sensorMPU->loop();
		// TODO: Repensar como fazer a odometria com os encoders no "Slave"
		// robotOdom->update(encoderE->getTotalTicks(), encoderD->getTotalTicks());

		ponte->loop(sensor, sensorMPU);
	}
}
