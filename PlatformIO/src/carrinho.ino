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

#include "LED.h"
#include "MediaMovel.h"
#include "PowerManager.h"

const char* ssid = "Canguru";
const char* password = "VamoPula";
const char* otaHostname = "espcarrinho";
const char* otaPassword = "VamoPula";

const double intentKp = 1.0;
const double intentKi = 0.5;
const double intentKd = 0.0;

AsyncWebServer server(80);
AsyncWebSocket carSocket("/car");
AsyncWebSocket ws("/ws");

static double clamp(double value, double min, double max) {
	return (value < min) ? min : (value > max) ? max : value;
}

static void webLog(String msg) {
	#ifdef DEBUG_PRINTS
		Serial.print(msg);
	#endif
	ws.textAll(msg);
}

#include "VL53L0X.h"
#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"

class MPU6050 {
private:
	Adafruit_MPU6050 sensor;  // sem new/delete
	unsigned long lastRead = 0;
	static const unsigned long readInterval = 20; // ms entre leituras

	MediaMovel gyroX{10};
	MediaMovel gyroY{10};
	MediaMovel gyroZ{10};

	// previne cópia acidental (dois objetos lutando pelo mesmo hardware)
	MPU6050(const MPU6050&) = delete;
	MPU6050& operator=(const MPU6050&) = delete;

	bool firstRead = true; // para evitar leituras iniciais erradas
	// offsets calibrados para evitar drift
	float offsetX = 0.06; // ajuste fino do giroscópio X
	float offsetY = -0.03; // ajuste fino do giroscópio Y
	float offsetZ = -0.03; // ajuste fino do giroscópio Z

	MediaMovel accelX{10};
	MediaMovel accelY{10};
	MediaMovel accelZ{10};

	float offsetAccelX = 0.0; // ajuste fino do acelerômetro X
	float offsetAccelY = 0.0; // ajuste fino do acelerômetro Y
	float offsetAccelZ = 0.0; // ajuste fino do acelerômetro Z

	// unsigned long lastWSUpdate = 0;

public:
	MPU6050() = default;  // construtor padrão

	void setup() {
		if (!sensor.begin()) {
			webLog("Erro ao iniciar o MPU6050! Verifique as conexões.\n Reiniciando ESP32 em 1 segundo...\n");
			unsigned long restartDelay = 1000; // 1 segundo
			unsigned long startTime = millis();
			while (millis() - startTime < restartDelay) {} // espera 1 segundo, ocupado
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
		return getTemperature().temperature;
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
	unsigned long last_think = 0;
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

	void update(double gyro_error = 0.0) {
		unsigned long now = millis();
		if (now - last_think < thinkInterval) return;
		last_think = now;

		double lastPidOutput = pidOutput;
		currentRPM = encoder->getRPM(10, thinkInterval/1000.0);

		// controle de RPM
		float targetRadS  = this->targetRadS;
		float currentRadS = rpmToRadS(currentRPM);
		float rpmControl  = pidRPM.compute(targetRadS, currentRadS);

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
					" || Gyro Read: " + String(gyro_error) + "\n");
			lastDebug = now;
		}

		if (targetRadS < 0.01) {
			webLog("[Motor " + String(pwmPin) + "] Target RPM is too low, resetting to 100 RPM\n");
			targetRadS = rpmToRadS(100.0);
		}
	}


	bool isMoving() {
		return (pidOutput > 1.0) && (targetRPM > 0.0);
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
	double	  turning_angleZ = 0.0;
	unsigned long lastUpdate   = 0;
	unsigned long lastSocketUpdate = 0;
	static const unsigned long controlInterval = 100; // ms entre controles

	// flag para alternar quais motores atualizar
	bool		nextRight	  = true;
	double lastGyroZ = 0.0; // último valor do giroscópio Z

	PID pid_gyro = PID(0.1, 0.0, 0.0, controlInterval / 1000.0); // Kp, Ki, Kd para giroscópio

public:
	PonteH(Motor* right, Motor* left) : motorRight(right), motorLeft(left) {
		pid_gyro.setTunning(0.1, 0.0, 0.0); // Kp, Ki, Kd
		pid_gyro.setMaxMin(0.5, -0.5); // limites de correção
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
		turning_angleZ = 0.0;
		lastUpdate = millis();
		nextRight = true;  // reinicia alternância
		pid_gyro.reset();
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
			turning_angleZ = 0.0;
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
			turning_angleZ = 0.0;
			lastUpdate = millis();
			nextRight = true;
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"backward\"}");
		}
	}

	void turnLeft() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_TURN_LEFT) {
			stop();
			currentMove = MOVEMENT_TURN_LEFT;
			motorRight->backward();
			motorLeft->forward();
			isMoving = true;
			turning_angleZ = 0.0;
			lastUpdate = millis();
			nextRight = true;
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"left\"}");
		}
	}

	void turnRight() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_TURN_RIGHT) {
			stop();
			currentMove = MOVEMENT_TURN_RIGHT;
			motorRight->forward();
			motorLeft->backward();
			isMoving = true;
			turning_angleZ = 0.0;
			lastUpdate = millis();
			nextRight = true;
			carSocket.textAll("{\"status\": \"moving\", \"direction\": \"right\"}");
		}
	}

	void stop() {
		if (!motorRight || !motorLeft) return;
		motorRight->stop();
		motorLeft->stop();
		isMoving = false;
		nextRight = true;

		pid_gyro.reset();
		turning_angleZ = 0.0; // reseta o ângulo de giro
		lastGyroZ = 0.0; // reseta o último valor do giroscópio Z

		currentMove = MOVEMENT_STOPPED;
	}

	bool isStopped() const {
		return !isMoving || (currentMove == MOVEMENT_STOPPED); // Vai que eu esqueci de setar como parado em uma das duas, ai... Agora ta seguro :)
	}

	// Deve ser chamado dentro de loop()
	void loop(VL53L0X* front_dist_sensor, MPU6050* mpu_sensor) {
		if (!isMoving || !motorRight || !motorLeft ||
			!front_dist_sensor || !mpu_sensor) {
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
				int d = front_dist_sensor->loop();
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
				double gz = mpu_sensor->getGyroscopeZ();
				if (nextRight) {
					lastGyroZ = pid_gyro.compute(gz, 0.0); // calcula correção do giroscópio
					lastGyroZ = map(lastGyroZ, 0, 255, -0.5, 0.5); // mapeia para -0.5 a 0.5, considerando a faixa do PID
				} else {
					gz = lastGyroZ;  // usa o último valor salvo, para usar o mesmo valor do giroscópio
				}
				doUpdate(motorRight,  gz);
				doUpdate(motorLeft,  -gz);
				break;
			}
			case MOVEMENT_TURN_LEFT:
			case MOVEMENT_TURN_RIGHT: {
				double gz = mpu_sensor->getGyroscopeZ();
				// acumula ângulo em graus
				turning_angleZ += gz * deltaTime;
				#ifdef DEBUG_PRINTS
					Serial.print("[PonteH] Turning Angle Z: ");
					Serial.print(turning_angleZ); Serial.printf(" on deltaTime: %f\n", deltaTime);
				#endif
				carSocket.textAll("{\"turning_angleZ\": " + String(turning_angleZ) + ", \"deltaTime\": " + String(deltaTime) + "}");
				if (fabs(turning_angleZ) > 90.0) {
					stop();
					#ifdef DEBUG_PRINTS
						Serial.println("Parando por ângulo de giro excessivo!");
					#endif
					carSocket.textAll("{\"movement\": \"stopped\", \"reason\": \"Excessive turning angle\"}");
					return;
				}
				// mantém direção definida e alterna update
				doUpdate(motorRight);
				doUpdate(motorLeft);
				break;
			}
			default:
				break;
		}

		if (carSocket.count() > 0) {
			if (isMoving && now - lastSocketUpdate >= 100) {
				lastSocketUpdate = now;
				handleCommand("data");
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

void ConnectToWiFi(){
	WiFi.begin(ssid, password);
	#ifdef DEBUG_PRINTS
		Serial.print("Conectando ao WiFi -> ");
		Serial.print(ssid);
	#endif
	while (WiFi.status() != WL_CONNECTED) {
		#ifdef DEBUG_PRINTS
			Serial.print(".");
		#endif
		delay(500);
	}
	#ifdef DEBUG_PRINTS
		Serial.println(" WiFi conectado!");
		Serial.print("Endereco IP: ");
		Serial.println(WiFi.localIP());
	#endif
}
// ——————— Sensores ———————
VL53L0X *sensor = new VL53L0X();	// VL53L0X no I²C (SDA=21, SCL=22)
MPU6050 *sensorMPU = new MPU6050();	// MPU6050 no I²C (SDA=21, SCL=22)

// ——————— Encoders ———————
// Motor Direito
Encoder *encoderD = new Encoder(15, 4);  // CH A=4, CH B=15
// Motor Esquerdo
Encoder *encoderE = new Encoder(16, 17);  // CH A=17, CH B=16

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

// ——————— WebSocket ———————
// ——————— Car Command Handler ———————
String getCarStatus() {
	JsonDocument doc;
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

	String output;
	serializeJson(doc, output);
	return output;
}
void handleCar(String cmd) {
	cmd.toLowerCase();
	if (cmd == "stop") {
		ponte->stop();
		webLog("Carro parado.\n");
	} else if (cmd == "right") {
		ponte->turnRight();
		webLog("Carro virando para a direita.\n");
	} else if (cmd == "left") {
		ponte->turnLeft();
		webLog("Carro virando para a esquerda.\n");
	} else if (cmd == "forward") {
		ponte->forward();
		webLog("Carro indo para frente.\n");
	} else if (cmd == "backward") {
		ponte->backward();
		webLog("Carro indo para trás.\n");
	} else if (cmd == "status") {
		String status = getCarStatus();
		if (carSocket.count() > 0) {
			// carSocket.binaryAll(status.c_str(), status.length());
			carSocket.textAll(status);
		}
	} else {
		webLog("Comando desconhecido: " + cmd + "\n");
	}
}
void handleCommand(String cmd) {
	cmd.toLowerCase();
	if (cmd == "led") {
		led_carro.toggle();
		webLog("LED Alterado, agora está: " + String(led_carro.isOn() ? "ON" : "OFF") + "\n");
	} else if (cmd == "reboot" || cmd == "restart") {
		webLog("Reiniciando o ESP...\n");
		unsigned long startTime = millis();
		while (millis() - startTime < 100) {} // n faz porra, é busy wait
		Serial.println("Reiniciando o ESP...\n");
		ESP.restart();
	} else if (cmd == "ip") {
		String ip = WiFi.localIP().toString();
		webLog("IP atual: " + ip + "\n");
	} else if (cmd == "status") {
		String status = getCarStatus();
		webLog("Status do carro: " + status + "\n");
	} else if (cmd == "data") {
		JsonDocument doc;
		doc["led"] = led_carro.isOn();
		doc["ip"] = WiFi.localIP().toString();
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

		// Gyro
		JsonObject gyro = doc["gyro"].to<JsonObject>();
		gyro["x"] = sensorMPU->getGyroscopeX();
		gyro["y"] = sensorMPU->getGyroscopeY();
		gyro["z"] = sensorMPU->getGyroscopeZ();

		JsonObject accel = doc["accel"].to<JsonObject>();
		accel["x"] = sensorMPU->getAccelerometerX();
		accel["y"] = sensorMPU->getAccelerometerY();
		accel["z"] = sensorMPU->getAccelerometerZ();

		// Temperatura
		doc["temperature"] = sensorMPU->getTemperatureC();

		// Distância
		doc["distance"] = sensor->loop();

		// Motor Direito
		JsonObject motorD = doc["motorD"].to<JsonObject>();
		motorD["rpm"] = motorDireito->getRPM();
		motorD["pidOutput"] = motorDireito->getPIDOutput();
		motorD["targetRPM"] = motorDireito->getTargetRPM();
		motorD["isClockwise"] = motorDireito->isClockwise();

		// Motor Esquerdo
		JsonObject motorE = doc["motorE"].to<JsonObject>();
		motorE["rpm"] = motorEsquerdo->getRPM();
		motorE["pidOutput"] = motorEsquerdo->getPIDOutput();
		motorE["targetRPM"] = motorEsquerdo->getTargetRPM();
		motorE["isClockwise"] = motorEsquerdo->isClockwise();

		String output;
		serializeJson(doc, output);
		webLog("Dados do carro: " + output + "\n");
		if (carSocket.count() > 0) {
			// carSocket.binaryAll(output.c_str(), output.length());
			carSocket.textAll(output);
		}
	} else {
		webLog("Comando desconhecido: " + cmd + "\n");
	}
}
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
	String sockName = "Unknown Socket";
	if (server == &carSocket) {
		sockName = "Car Socket";
	} else if (server == &ws) {
		sockName = "WebSocket";
	}
	switch (type) {
		case WS_EVT_CONNECT:
			Serial.printf("[%s] WebSocket client #%u connected from %s\n", sockName.c_str(), client->id(), client->remoteIP().toString().c_str());
			webLog("["+ sockName + "] Cliente ["+ String(client->id()) +"]: Conectado de " + client->remoteIP().toString() + "\n");
			if (server == &carSocket) {
				Serial.printf("WebSocket [server #%s] client #%u is ready\n", sockName.c_str(), client->id());
				client->text("{\"ready\":\"true\"}");
			}
			break;
		case WS_EVT_DISCONNECT:
			if (server == &carSocket) {
				if (server->count() == 0) {
					ponte->stop();
					lastReading = millis();
					delay(100); // Aguarda 100ms para garantir que o cliente esteja pronto
				}
			}
			webLog("["+ sockName + "] Cliente ["+ String(client->id()) +"]: Desconectado\n");
			Serial.printf("WebSocket [server #%s] client #%u disconnected\n", sockName.c_str(), client->id());
			break;
		case WS_EVT_DATA:
			AwsFrameInfo *info;
			info = (AwsFrameInfo*)arg;
			if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
				data[len] = 0;
				String command = (char*)data;
				Serial.printf("Comando recebido do cliente #%u: %s\n", client->id(), command.c_str());
				if (server == &carSocket) {
					handleCar(command);
					delay(100); // Aguarda 100ms para garantir que o cliente esteja pronto
					webLog("["+ sockName + "] Cliente ["+ String(client->id()) +"]: Comando recebido: " + command + "\n");
				} else if (server == &ws) {
					webLog("["+ sockName + "] Cliente ["+ String(client->id()) +"]: Comando recebido: " + command + "\n");
					handleCommand(command);
				} else {
					webLog("["+ sockName + "] Comando desconhecido: " + command + "\n");
				}
			}
			break;
		case WS_EVT_PONG:
		case WS_EVT_ERROR:
			break;
	}
}

void http_stop_carro(AsyncWebServerRequest *request) {
	ponte->stop();
	lastReading = millis();
	request->send(200, "application/json", "{\"status\":\"stopped\"}");
}

void http_handle_forward(AsyncWebServerRequest *request) {
	ponte->forward();
	lastReading = millis();
	request->send(200, "application/json", "{\"status\":\"moving forward\"}");
}

void http_handle_backward(AsyncWebServerRequest *request) {
	ponte->backward();
	lastReading = millis();
	request->send(200, "application/json", "{\"status\":\"moving backward\"}");
}

void http_handle_turn_left(AsyncWebServerRequest *request) {
	ponte->turnLeft();
	lastReading = millis();
	request->send(200, "application/json", "{\"status\":\"turning left\"}");
}

void http_handle_turn_right(AsyncWebServerRequest *request) {
	ponte->turnRight();
	lastReading = millis();
	request->send(200, "application/json", "{\"status\":\"turning right\"}");
}

void http_handle_test(AsyncWebServerRequest *request) {
	String html = "<html><head>";
	html += "<meta name=\"viewport\" content=\"width=device-width, initial-scale=1\">";
	html += "<style>";
	html += "body { font-family: Arial, sans-serif; }";
	html += "h1 { color: #333; }";
	html += "button { padding: 10px 20px; font-size: 16px; margin: 5px; }";
	html += "button:hover { background-color: #ddd; }";
	html += "div { margin-top: 20px; }";
	html += "</style>";
	html += "<title>Manual Control</title>";
	html += "</head><body>";
	html += "<h1>Manual Control</h1>";
	html += "<button onclick=\"fetchURL('/forward')\">Forward</button>";
	html += "<button onclick=\"fetchURL('/backward')\">Backward</button>";
	html += "<button onclick=\"fetchURL('/turn_left')\">Turn Left</button>";
	html += "<button onclick=\"fetchURL('/turn_right')\">Turn Right</button>";
	html += "<button onclick=\"fetchURL('/stop')\">Stop</button>";
	html += "<button onclick=\"fetchURL('/data')\">Data</button>";
	html += "<div id=\"data\"></div>";
	html += "<script>";
	html += "function fetchURL(url) {";
	html += " fetch(url).then(response => response.json()).then(data => {";
	html += "  console.log(data);";
	html += "  document.getElementById('data').innerText = JSON.stringify(data);";
	html += " });";
	html += "}";
	html += "</script>";
	html += "</body></html>";
	lastReading = millis();
	request->send(200, "text/html", html);
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
	server.on("/forward", HTTP_GET, http_handle_forward);
	server.on("/backward", HTTP_GET, http_handle_backward);
	server.on("/turn_left", HTTP_GET, http_handle_turn_left);
	server.on("/turn_right", HTTP_GET, http_handle_turn_right);
	server.on("/stop", HTTP_GET, http_stop_carro);
	server.on("/test", HTTP_GET, http_handle_test);
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

	ConnectToWiFi();

	if (!MDNS.begin(otaHostname)) {
		Serial.println("Erro ao iniciar mDNS");
	} else {
		Serial.println("mDNS iniciado!");
	}
	prepare_http_server();
	init_ota();

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
	ponte->setup();

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
		ConnectToWiFi();
		return;
	}

	powerManager.loop();

	unsigned long now = millis();
	if (now - lastLoopTime >= loopInterval) {
		lastLoopTime = now;

		sensorMPU->loop();

		ponte->loop(sensor, sensorMPU);

		lastReading = millis();
	}
}
