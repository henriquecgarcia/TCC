/*
	Trabalho de conclusão de Curso - UNIFESP - Campus São José dos Campos
	Alunos: Henrique Campanha Garcia
	Professor Orientador: André Marcorin
	Universidade: UNIFESP - Campus São José dos Campos
	Itens utilizados:
	- ESP32 --> Controlador do carrinho, que controla tudo.
	- Ponte H --> Utilizado para controlar os motores do carrinho.
	- 4 motores CC --> Utilizado para movimentar o carrinho.
		* Motor 1: Motor direito.
			| Pino 12 --> Ponte H.
			| Pino 13 --> Ponte H.
			| Pino 14 --> PWM.
		* Motor 2: Motor esquerdo.
			| Pino 33 --> Ponte H.
			| Pino 25 --> Ponte H.
			| Pino 32 --> PWM.
	- 2 encoders --> Utilizado para medir a velocidade do carrinho.
		* Encoder 1: Motor direito.
			| Pino 26 --> Encoder.
			| Pino 27 --> Encoder.
		* Encoder 2: Motor esquerdo.
			| Pino 35 --> Encoder.
			| Pino 34 --> Encoder.
	- 1 sensor de distância VL53L0X --> Utilizado para medir a distância do carrinho em relação a um obstáculo.
		* Pino SDA --> 21
		* Pino SCL --> 22
	- 1 sensor MPU6050 --> Utilizado para medir a aceleração e Giroscópio do carrinho.
		* Pino SDA --> 21
		* Pino SCL --> 22
	- 1 bateria de 9V --> Utilizado para alimentar a ponte H que alimenta os motores e o ESP32 (via 5V).
		* Externo ao ESP32, ligado na ponte H.
*/

#define DEBUG_PRINTS
#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <ArduinoJson.h>

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>
#include <ArduinoOTA.h>
#include <ESPmDNS.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

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

#include "Encoder.h"
#include "PID.h"

unsigned long lastReading = 0; // para evitar leituras excessivas
class Motor {
private:
	unsigned long lastDebug = 0;
	unsigned long lastThink = 0;
	const uint8_t in1Pin, in2Pin, pwmPin;
	Encoder* encoder;

	double targetRPM = 100.0;
	double targetRadS = rpmToRadS(targetRPM);
	double currentRPM = 0.0;
	double pidOutput = 0.0;

	PID pidRPM; // PID original para controle de RPM

	static constexpr unsigned long thinkInterval = 500; // ms entre updates

	double rpmToRadS(double rpm) {
		return rpm * (M_PI / 30.0f); // converte RPM para rad/s
	}

public:
	// Construtor: inicializa ambos PIDs
	Motor(int in1, int in2, int pwm, Encoder* enc, float kp_rpm = 1.0, float ki_rpm = 5.0, float kd_rpm = 0.0) : in1Pin(in1), in2Pin(in2), pwmPin(pwm), encoder(enc),
		pidRPM(kp_rpm, ki_rpm, kd_rpm, thinkInterval/1000.0f) {}

	void begin() {
		pinMode(in1Pin, OUTPUT);
		pinMode(in2Pin, OUTPUT);
		pinMode(pwmPin, OUTPUT);

		encoder->begin();
		encoder->reset();

		stop();
		// limites RPM em rad/s
		pidRPM.setMaxMin(rpmToRadS(200.0), 0.0);
	}

	void setTunings(float kp, float ki, float kd) {
		pidRPM.setTunning(kp, ki, kd);
	}
	void setTargetRPM(double rpm) {
		targetRadS = rpmToRadS(rpm);
	}

	void forward() {
		encoder->reset();
		stop();

		digitalWrite(in1Pin, HIGH);
		digitalWrite(in2Pin, LOW);

		targetRadS = rpmToRadS(100.0);

		webLog("[Motor " + String(pwmPin) + "] Direction is now forwards.\n");
	}
	void backward() {
		encoder->reset();
		stop();

		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, HIGH);

		targetRadS = rpmToRadS(100.0);

		webLog("[Motor " + String(pwmPin) + "] Direction is now backwards.\n");
	}
	void stop() {
		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, LOW);
		encoder->reset();

		targetRadS = 0.0;
		pidRPM.reset();
		currentRPM = 0.0;
		pidOutput = 0.0;
		analogWrite(pwmPin, 0);
		lastReading = millis();

		webLog("[Motor " + String(pwmPin) + "] Stopped.\n");
	}

	void update(double gyroError = 0.0) {
		unsigned long now = millis();
		if (now - lastThink < thinkInterval) return;
		lastThink = now;

		currentRPM = encoder->getRPM(10, thinkInterval/1000.0);

		// controle de RPM com compensação de giro (se houver)
		float targetRadS  = this->targetRadS + gyroError; 
		float currentRadS = rpmToRadS(currentRPM);
		float rpmControl  = pidRPM.compute(targetRadS, currentRadS);
		rpmControl = pidRPM.scaleToPWM(rpmControl); // converte para valor de PWM

		pidOutput = rpmControl;
		pidOutput = clamp(pidOutput, 0.0, 255.0);

		analogWrite(pwmPin, int(pidOutput));
		encoder->reset();

		if (now - lastDebug >= thinkInterval) {
			webLog("[Motor " + String(pwmPin) + 
					" (" + (pwmPin == 32 ? "Right" : "Left") + ") " +
					"] RPM out: " + String(rpmControl) +
					" -> PWM: " + String(pidOutput) +
					" || Encoder RPM Read: " + String(currentRPM) +
					" || Gyro Read: " + String(gyroError) + "\n");
			lastDebug = now;
		}

		float effectiveTargetRadS = targetRadS + gyroError;

		if (effectiveTargetRadS < 0.01) {
			webLog("[Motor " + String(pwmPin) + "] Target RPM is too low, resetting to 100 RPM\n");
			this->targetRadS = rpmToRadS(100.0);
		}
	}


	bool isMoving() {
		return (pidOutput > 1.0) && (targetRadS > 0.0);
	}

	void manualAddPID(int value) {
		pidOutput += value;
		analogWrite(pwmPin, int(pidOutput));
		#ifdef DEBUG_PRINTS
			Serial.print("[Motor "); Serial.print(pwmPin);
			Serial.print("] PID manual adicionado: "); Serial.println(value);
			Serial.print("Novo PID Output: "); Serial.println(pidOutput);
		#endif
		lastReading = millis();
	}

	double getRPM() const {
		return currentRPM;
	}
	double getTargetRPM() const {
		return targetRPM;
	}
	double getPIDOutput() const {
		return pidOutput;
	}
	bool isClockwise() const {
		return encoder->isClockwise();
	}
};

enum Movement {
	MOVEMENT_FORWARD,
	MOVEMENT_BACKWARDS,
	MOVEMENT_TURN_LEFT,
	MOVEMENT_TURN_RIGHT,
	MOVEMENT_STOPPED
};
class PonteH {
private:
	Motor*	  motorRight;
	Motor*	  motorLeft;
	Movement	currentMove	= MOVEMENT_STOPPED;
	bool		isMoving	   = false;
	double	  turningAngleZ = 0.0; // acumulado em radianos
	unsigned long lastUpdate   = 0;
	unsigned long lastSocketUpdate = 0;
	static const unsigned long controlInterval = 100; // ms entre controles
	double turnLimitRad = 0.0; // Sempre pra fente!

	// flag para alternar quais motores atualizar
	bool		nextRight	  = true;
	double lastGyroZ = 0.0; // último valor do giroscópio Z

	PID pidGyro = PID(0.1, 0.0, 0.0, controlInterval / 1000.0); // Kp, Ki, Kd para giroscópio
	PID pidTurn = PID(10.0, 5.0, 0.5, controlInterval / 1000.0); // Controlador PID para as curvas (ajusta RPM)

	static double normalizeAngle(double angle) {
		angle = fmod(angle, 2 * M_PI);
		if (angle < -M_PI) {
			angle += 2 * M_PI;
		} else if (angle > M_PI) {
			angle -= 2 * M_PI;
		}
		return angle;
	}

	void _fixTurning() {
		turnLimitRad = normalizeAngle(turnLimitRad);
		turningAngleZ = normalizeAngle(turningAngleZ);
	}

public:
	PonteH(Motor* right, Motor* left) : motorRight(right), motorLeft(left) {
		pidGyro.setTunning(0.1, 0.0, 0.0); // Kp, Ki, Kd
		pidGyro.setMaxMin(0.5, -0.5); // limites de correção
		pidTurn.setMaxMin(M_PI, -M_PI); // limites de correção para curvas
	}

	void setup() {
		if (!motorRight || !motorLeft) {
			webLog("[PonteH] Erro: Motores não configurados corretamente!\n");
			return;
		}
		motorRight->begin();
		motorLeft->begin();
		isMoving = false;
		currentMove = MOVEMENT_STOPPED;
		turningAngleZ = 0.0;
		turnLimitRad = 0.0;
		lastUpdate = millis();
		nextRight = true;  // reinicia alternância
		pidGyro.reset();
		webLog("[PonteH] Configuração completa!\n");
	}

	void forward() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_FORWARD) {
			stop();
			currentMove = MOVEMENT_FORWARD;
			motorRight->forward();
			motorLeft->forward();
			isMoving = true;
			lastUpdate = millis();
			nextRight = true;  // reinicia alternância
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"forward\"}");
		}
	}

	void backward() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_BACKWARDS) {
			stop();
			currentMove = MOVEMENT_BACKWARDS;
			motorRight->backward();
			motorLeft->backward();
			isMoving = true;
			lastUpdate = millis();
			nextRight = true;
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"backward\"}");
		}
	}

	void turnLeft(double degs = 90.0) {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_TURN_LEFT) {
			stop();
			currentMove = MOVEMENT_TURN_LEFT;
			motorRight->backward();
			motorLeft->forward();
			isMoving = true;
			lastUpdate = millis();
			nextRight = true;
			pidTurn.reset(); // Zera o PID na nova curva
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"left\"}");

			double turnRads = degs * (M_PI / 180.0);

			turnLimitRad -= normalizeAngle(turnRads); // converte limite de giro para radianos e normaliza
			webLog("[PonteH] Turn limit set to " + String(degs) + " degrees (" + String(turnLimitRad) + " radians)\n");
		}
	}

	void turnRight(double degs = 90.0) {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_TURN_RIGHT) {
			stop();
			currentMove = MOVEMENT_TURN_RIGHT;
			motorRight->forward();
			motorLeft->backward();
			isMoving = true;
			lastUpdate = millis();
			nextRight = true;
			pidTurn.reset(); // Zera o PID na nova curva
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"right\"}");

			double turnRads = degs * (M_PI / 180.0);

			turnLimitRad += normalizeAngle(turnRads); // converte limite de giro para radianos e normaliza
			webLog("[PonteH] Turn limit set to " + String(degs) + " degrees (" + String(turnLimitRad) + " radians)\n");
		}
	}

	void stop() {
		if (!motorRight || !motorLeft) return;
		motorRight->stop();
		motorLeft->stop();
		isMoving = false;
		nextRight = true;

		pidGyro.reset();
		pidTurn.reset();
		lastGyroZ = 0.0; // reseta o último valor do giroscópio Z

		currentMove = MOVEMENT_STOPPED;
	}

	bool isStopped() const {
		return !isMoving || (currentMove == MOVEMENT_STOPPED); // Vai que eu esqueci de setar como parado em uma das duas, ai... Agora ta seguro :)
	}

	// Deve ser chamado dentro de loop()
	void loop(VL53L0X* frontDistSensor, MPU6050* mpuSensor) {
		if (!isMoving || !motorRight || !motorLeft ||
			!frontDistSensor || !mpuSensor) {
			return;
		}

		unsigned long now = millis();
		if (now - lastUpdate < controlInterval) {
			return;
		}
		double deltaTime = (now - lastUpdate) / 1000.0;  // em segundos
		lastUpdate = now;

		// Função auxiliar para chamar update() alternado
		auto doUpdate = [&](Motor* m, double gyroCorr = 0.0) {
			if (nextRight && m == motorRight)	  m->update(gyroCorr);
			else if (!nextRight && m == motorLeft) m->update(gyroCorr);
		};

		switch (currentMove) {
			case MOVEMENT_FORWARD: {
				int d = frontDistSensor->loop();
				if (d < 100) {
					stop();
					#ifdef DEBUG_PRINTS
						Serial.println("Parando por obstáculo!");
					#endif
					webLog("[PonteH] Parando por obstáculo!\n");
					carSocket.textAll("{\"movement\": \"stopped\", \"reason\": \"obstacle\"}");
					break;
				}
			}
			case MOVEMENT_BACKWARDS: {
				double gz = lastGyroZ;
				if (nextRight) {
					double read = mpuSensor->getGyroscopeZ();
					lastGyroZ = pidGyro.compute(read, 0.0); // calcula correção do giroscópio
				}
				doUpdate(motorRight,  gz);
				doUpdate(motorLeft,  -gz);
				break;
			}
			case MOVEMENT_TURN_LEFT:
			case MOVEMENT_TURN_RIGHT: {
				double gz = mpuSensor->getGyroscopeZ();

				if (currentMove == MOVEMENT_TURN_LEFT) gz = -gz; // inverte para esquerda
				turningAngleZ += gz * deltaTime;
				const double turningAngleDeg = turningAngleZ * (180.0 / M_PI);

				double deltaToLimit = turnLimitRad - turningAngleZ;
				double turnCorrection = pidTurn.compute(0.0, deltaToLimit);

				#ifdef DEBUG_PRINTS
					Serial.print("[PonteH] Turning Angle Z: ");
					Serial.print(turningAngleZ);
					Serial.print(" rad (");
					Serial.print(turningAngleDeg);
					Serial.printf(" deg) on deltaTime: %f\n", deltaTime);
					Serial.print("Turn correction: ");
					Serial.println(turnCorrection);
				#endif
				webLog("[PonteH] Turning Angle Z: " + String(turningAngleZ) + " rad (" + String(turningAngleDeg) + " deg) on deltaTime: " + String(deltaTime) + "\nTurn correction: " + String(turnCorrection) + "\n");
				carSocket.textAll("{\"turning_angleZ\": " + String(turningAngleZ) + ", \"turningAngleZ_deg\": " + String(turningAngleDeg) + ", \"delta_time\": " + String(deltaTime) + "}");

				if (fabs(deltaToLimit) < (5.0 * M_PI / 180.0)) { // se estiver a menos de ~3 graus do limite, para o carrinho
					stop();
					#ifdef DEBUG_PRINTS
						Serial.println("Parando por ângulo de giro alcançado!");
					#endif
					carSocket.textAll("{\"movement\": \"stopped\", \"reason\": \"target angle reached\"}");
					_fixTurning(); // garante que os ângulos estejam normalizados
					return;
				}

				// mantém direção definida e alterna update
				doUpdate(motorRight, turnCorrection);
				doUpdate(motorLeft, -turnCorrection);
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
		nextRight = !nextRight;
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

// ——————— Sensores ———————
VL53L0X *sensor = new VL53L0X();	// VL53L0X no I²C (SDA=21, SCL=22)
MPU6050 *sensorMPU = new MPU6050();	// MPU6050 no I²C (SDA=21, SCL=22)

// ——————— Encoders ———————
// Motor Direito
Encoder *encoderD = new Encoder(15, 4);  // CH A=4, CH B=15
// Motor Esquerdo
Encoder *encoderE = new Encoder(16, 17);  // CH A=17, CH B=16

Map *robotMap = new Map(GRID_WIDTH, GRID_HEIGHT);
Odometry *robotOdom = new Odometry(robotMap);
Map::Position plannedPath[PATH_BUFFER_SIZE];
size_t plannedPathLen = 0;
bool hasPlannedPath = false;

// ——————— Motores com PID ———————
// Motor Direito  → IN1=12, IN2=13, PWM=32
Motor *motorDireito  = new Motor( 12, 13, 32, encoderD, intentKp, intentKi, intentKd );
// Motor Esquerdo → IN1=26, IN2=25, PWM=33
Motor *motorEsquerdo = new Motor( 26, 25, 33, encoderE, intentKp, intentKi, intentKd );

// ——————— Ponte H (drive de 2 motores) ———————
PonteH *ponte = new PonteH(motorDireito, motorEsquerdo);

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

	JsonObject motorD = doc["motorD"].to<JsonObject>();
	motorD["rpm"] = motorDireito->getRPM();
	motorD["pidOutput"] = motorDireito->getPIDOutput();
	motorD["targetRPM"] = motorDireito->getTargetRPM();
	motorD["isClockwise"] = motorDireito->isClockwise();
	motorD["ticks"] = encoderD->getTotalTicks();
	motorD["turns"] = encoderD->getTotalRevolutions(ENCODER_TEETH);
	motorD["fullTurn"] = encoderD->hasCompletedFullTurn(ENCODER_TEETH);

	JsonObject motorE = doc["motorE"].to<JsonObject>();
	motorE["rpm"] = motorEsquerdo->getRPM();
	motorE["pidOutput"] = motorEsquerdo->getPIDOutput();
	motorE["targetRPM"] = motorEsquerdo->getTargetRPM();
	motorE["isClockwise"] = motorEsquerdo->isClockwise();
	motorE["ticks"] = encoderE->getTotalTicks();
	motorE["turns"] = encoderE->getTotalRevolutions(ENCODER_TEETH);
	motorE["fullTurn"] = encoderE->hasCompletedFullTurn(ENCODER_TEETH);

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
					lastReading = millis();
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

void handlePower(AsyncWebServerRequest *request) {
	if (request->hasParam("mode")) {
		String mode = request->getParam("mode")->value();
		if (mode == "normal") {
			powerManager.setPowerMode(POWER_NORMAL);
		} else if (mode == "saving") {
			powerManager.setPowerMode(POWER_SAVING);
		}
	}
	String json = "{\"power_mode\":\"" + String(powerManager.isPowerSaving() ? "saving" : "normal") + "\"}";
	request->send(200, "application/json", json);
	lastReading = millis();
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
	// Configura o gerenciador de energia
	powerManager.isIdleCheck = isIdle;
	powerManager.lastCommandReceived = &lastReading;

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
	robotMap->generateStraightLineTest(MAP_ORIGIN_Y);
	robotMap->setPosition(MAP_ORIGIN_X, MAP_ORIGIN_Y);
	robotOdom->reset(0.0f, 0.0f, 0.0f, encoderE->getTotalTicks(), encoderD->getTotalTicks());

	Wire.begin();
	sensor->setup();
	sensorMPU->setup();

	#ifdef DEBUG_PRINTS
		Serial.println("Setup completo!");
	#endif

	lastReading = millis();
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
		robotOdom->update(encoderE->getTotalTicks(), encoderD->getTotalTicks());

		ponte->loop(sensor, sensorMPU);
	}
}
