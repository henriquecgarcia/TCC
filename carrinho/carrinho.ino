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

#include <WiFi.h>
// #include <ESPAsyncWebServer.h>
// #include <SPIFFS.h>
#include <WebServer.h>

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"
#include "Adafruit_VL53L0X.h"

#include <Arduino.h>
#include <math.h>

#include "LED.h"
#include "MediaMovel.h"
#include "PowerManager.h"

const char* ssid = "Canguru";
const char* password = "VamoPula";

const double intentKp = 1.0;
const double intentKi = 0.5;
const double intentKd = 0.0;

// AsyncWebServer server(80);
WebServer server(80);

template <typename T>
T clamp(T value, T min, T max) {
	if (value < min) return min;
	if (value > max) return max;
	return value;
}

class VL53L0X {
private:
	Adafruit_VL53L0X sensor; // sem alocação dinâmica
	unsigned long lastRead = 0; // inicialização inline
	int last_reading = 0;
	static const unsigned long readInterval = 100; // ms entre leituras

	// previne cópia acidental (dois objetos no mesmo hardware)
	VL53L0X(const VL53L0X&) = delete;
	VL53L0X& operator=(const VL53L0X&) = delete;

public:
	VL53L0X() = default;  // construtor padrão

	void setup() {
		if (!sensor.begin()) {
			#ifdef DEBUG_PRINTS
				Serial.println("Falha ao encontrar o sensor VL53L0X");
			#endif
			LED inLed = LED(2);
			inLed.setup();
			while (true) {
				inLed.toggle();
				delay(10);  // mantém o watchdog feliz
			}
		}
		#ifdef DEBUG_PRINTS
			Serial.println("Sensor VL53L0X encontrado!");
		#endif
	}

	int loop() {
		unsigned long now = millis();
		if (now - lastRead < readInterval) {
			return last_reading;
		}
		lastRead = now;

		VL53L0X_RangingMeasurementData_t measure;
		sensor.rangingTest(&measure, false);

		#ifdef DEBUG_PRINTS
			Serial.print("Distancia: ");
		#endif
		if (measure.RangeStatus != 4) { // se não estiver fora de alcance
			last_reading = measure.RangeMilliMeter;
			#ifdef DEBUG_PRINTS
				Serial.print(last_reading);
				Serial.println(" mm");
			#endif
		} else {
			#ifdef DEBUG_PRINTS
				Serial.println("Fora do alcance");
			#endif
			last_reading = 99999;
		}
		return last_reading;
	}
};

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

public:
	MPU6050() = default;  // construtor padrão

	void setup() {
		if (!sensor.begin()) {
			#ifdef DEBUG_PRINTS
				Serial.println("Falha ao encontrar o sensor MPU6050");
			#endif
			LED inLed = LED(2);
			inLed.setup();
			while (true) {
				inLed.toggle();
				delay(20);  // mantém o watchdog feliz
			}
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
		return getAccelerometer().acceleration.x;
	}
	float getAccelerometerY() {
		return getAccelerometer().acceleration.y;
	}
	float getAccelerometerZ() {
		return getAccelerometer().acceleration.z;
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

		// Atualiza as médias móveis com offsets calibrados
		gyroX.add(g.gyro.x + 0.06);
		gyroY.add(g.gyro.y - 0.03);
		gyroZ.add(g.gyro.z - 0.03);
	}
};

class Encoder {
private:
	volatile int countA = 0, countB = 0;
	uint8_t pinA, pinB;
	int lastStateB = LOW;
	bool clockwise = true;

	static void isrA_arg(void* arg) {
		static_cast<Encoder*>(arg)->onPulseA();
	}
	static void isrB_arg(void* arg) {
		static_cast<Encoder*>(arg)->onPulseB();
	}

	// callbacks de pulso sem IRAM_ATTR
	void onPulseA() {
		countA++;
	}
	void onPulseB() {
		countB++;
		int current = digitalRead(pinB);
		if (lastStateB == LOW && current == HIGH) {
			clockwise = (digitalRead(pinA) == LOW);
		}
		lastStateB = current;
	}

public:
	Encoder(uint8_t pinA, uint8_t pinB) : pinA(pinA), pinB(pinB) {
		countA = countB = 0;
		lastStateB = LOW;
		clockwise = true;
	}

	void begin() {
		pinMode(pinA, INPUT_PULLUP);
		pinMode(pinB, INPUT_PULLUP);
		// passa 'this' para o ISR correto
		attachInterruptArg(pinA, isrA_arg, this, CHANGE);
		attachInterruptArg(pinB, isrB_arg, this, CHANGE);
	}

	void reset() {
		countA = countB = 0;
	}

	double getRPM(int teeth = 10, double intervalSec = 0.1) {
		int pulses = (countA + countB) / 2;
		double revs = pulses / double(teeth * 2);
		return (revs / intervalSec) * 60.0;
		// return revs;
	}

	double getSpeed(int wheelDiameter = 65, int teeth = 10, double intervalSec = 0.1) {
		double rpm = getRPM(teeth, intervalSec);
		return (rpm * wheelDiameter * M_PI) / 1000.0; // mm/s
	}

	bool isClockwise() const {
		return clockwise;
	}
};

#include "PID.h"

unsigned long lastReading = 0; // para evitar leituras excessivas
class Motor {
private:
	unsigned long lastDebug = 0;
	unsigned long last_think = 0;
	const uint8_t in1Pin, in2Pin, pwmPin;
	Encoder* encoder;

	double targetRPM = 100.0;
	double currentRPM = 0.0;
	double pidOutput = 0.0;

	PID pidRPM; // PID original para controle de RPM
	PID pidGyro; // Novo PID dedicado ao erro de giroscópio

	static constexpr unsigned long thinkInterval = 500; // ms entre updates

public:
	// Construtor: inicializa ambos PIDs
	Motor(int in1, int in2, int pwm, Encoder* enc, float kp_rpm = 1.0, float ki_rpm = 5.0, float kd_rpm = 0.0, float kp_gyro = 0.1, float ki_gyro = 0.0, float kd_gyro = 0.0) : in1Pin(in1), in2Pin(in2), pwmPin(pwm), encoder(enc),
		pidRPM(kp_rpm, ki_rpm, kd_rpm, thinkInterval/1000.0f), pidGyro(kp_gyro, ki_gyro, kd_gyro, thinkInterval/1000.0f) {}

	void begin() {
		pinMode(in1Pin, OUTPUT);
		pinMode(in2Pin, OUTPUT);
		pinMode(pwmPin, OUTPUT);

		encoder->begin();
		encoder->reset();

		stop();
		// limites RPM em rad/s
		pidRPM.setMaxMin(200.0 * M_PI / 30.0, 0.0);
		// limites para correção de giroscópio (em rad/s)
		pidGyro.setMaxMin(0.5, -0.5);
	}

	void setTunings(float kp, float ki, float kd) {
		pidRPM.setTunning(kp, ki, kd);
		pidGyro.setTunning(kp/10, ki/10, kd/10);
	}
	void setTargetRPM(double rpm) {
		targetRPM = rpm;
	}

	void forward() {
		encoder->reset();
		stop();

		digitalWrite(in1Pin, HIGH);
		digitalWrite(in2Pin, LOW);

		targetRPM = 100.0;

		#ifdef DEBUG_PRINTS
			Serial.print("[Motor "); Serial.print(pwmPin);
			Serial.println("] Direction is now forwards");
		#endif
	}
	void backward() {
		encoder->reset();
		stop();

		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, HIGH);

		targetRPM = 100.0;
		#ifdef DEBUG_PRINTS
			Serial.print("[Motor "); Serial.print(pwmPin);
			Serial.println("] Direction is now backwards");
		#endif
	}
	void stop() {
		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, LOW);
		encoder->reset();

		targetRPM = 0.0;
		pidRPM.reset();
		pidGyro.reset();
		currentRPM = 0.0;
		pidOutput = 0.0;
		analogWrite(pwmPin, 0);
		lastReading = millis();

		#ifdef DEBUG_PRINTS
			Serial.print("[Motor "); Serial.print(pwmPin);
			Serial.println("] Stopped");
		#endif
	}

	void update(double gyro_error = 0.0) {
		unsigned long now = millis();
		if (now - last_think < thinkInterval) return;
		last_think = now;

		double lastPidOutput = pidOutput;
		currentRPM = encoder->getRPM(10, thinkInterval/1000.0);

		// controle de RPM
		float targetRadS  = targetRPM * (M_PI / 30.0f);
		float currentRadS = currentRPM * (M_PI / 30.0f);
		float rpmControl  = pidRPM.compute(targetRadS, currentRadS);

		// controle de alinhamento usando erro do giroscópio
		float gyroControl = pidGyro.compute(0.0f, static_cast<float>(gyro_error));
		gyroControl -= 255.0f / 2.0f; // centraliza em torno de 0

		// soma dos dois controles para sinal PWM
		pidOutput = rpmControl + gyroControl;
		pidOutput = clamp(pidOutput, 0.0, 255.0);

		analogWrite(pwmPin, int(pidOutput));
		encoder->reset();

		if (now - lastDebug >= thinkInterval) {
			#ifdef DEBUG_PRINTS
				Serial.print("[Motor "); Serial.print(pwmPin);
				Serial.print("] RPM out: "); Serial.print(rpmControl);
				Serial.print(" | Gyro out: "); Serial.print(gyroControl);
				Serial.print(" -> PWM: "); Serial.println(pidOutput);

				Serial.println("\n"); Serial.print("Encoder RPM Read: ");
				Serial.print(currentRPM); Serial.print("RPM | ");
				Serial.print(currentRadS); Serial.println("rad/s");

				Serial.print("Gyro Read: "); Serial.println(gyro_error);
			#endif
			lastDebug = now;
		}

		// if (pidOutput < 10.0) {
		// 	stop();
		// 	#ifdef DEBUG_PRINTS
		// 	Serial.println("Motor desligado por PID baixo");
		// #endif
		// }
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
	static const unsigned long controlInterval = 100; // ms entre controles

	// flag para alternar quais motores atualizar
	bool		nextRight	  = true;

public:
	PonteH(Motor* right, Motor* left) : motorRight(right), motorLeft(left)
	{}

	void setup() {
		if (motorRight) motorRight->begin();
		if (motorLeft)  motorLeft->begin();
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
		}
	}

	void turnLeft() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_TURN_LEFT) {
			stop();
			currentMove = MOVEMENT_TURN_LEFT;
			motorRight->forward();
			motorLeft->backward();
			isMoving = true;
			turning_angleZ = 0.0;
			lastUpdate = millis();
			nextRight = true;
		}
	}

	void turnRight() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_TURN_RIGHT) {
			stop();
			currentMove = MOVEMENT_TURN_RIGHT;
			motorRight->backward();
			motorLeft->forward();
			isMoving = true;
			turning_angleZ = 0.0;
			lastUpdate = millis();
			nextRight = true;
		}
	}

	void stop() {
		if (!motorRight || !motorLeft) return;
		motorRight->stop();
		motorLeft->stop();
		isMoving = false;
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
					return;
				}
				double gz = mpu_sensor->getGyroscopeZ();
				// alterna update entre Right e Left
				doUpdate(motorRight, -gz);
				doUpdate(motorLeft,   gz);
				break;
			}
			case MOVEMENT_BACKWARDS: {
				double gz = mpu_sensor->getGyroscopeZ();
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
				if (fabs(turning_angleZ) > 90.0) {
					stop();
					#ifdef DEBUG_PRINTS
						Serial.println("Parando por ângulo de giro excessivo!");
					#endif
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

		// alterna para a próxima chamada
		nextRight = !nextRight;
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
MPU6050 *sensorMPU = new MPU6050();	// MPU6050 no mesmo barramento I²C

// ——————— Encoders ———————
// Motor Direito
// Encoder *encoderD = new Encoder(27, 26);  // CH A=26, CH B=27
Encoder *encoderD = new Encoder(15, 4);  // CH A=17, CH B=16
// Motor Esquerdo
// Encoder *encoderE = new Encoder(35, 34);  // CH A=35, CH B=34
Encoder *encoderE = new Encoder(16, 17);  // CH A=19, CH B=18

// ——————— Motores com PID ———————
// Motor Direito  → IN1=12, IN2=13, PWM=14
Motor *motorDireito  = new Motor( 12, 13, 32, encoderD, intentKp, intentKi, intentKd );
// Motor Esquerdo → IN1=33, IN2=25, PWM=32
Motor *motorEsquerdo = new Motor( 26, 25, 33, encoderE, intentKp, intentKi, intentKd );

// ——————— Ponte H (drive de 2 motores) ———————
PonteH *ponte = new PonteH(motorDireito, motorEsquerdo);

// ——————— Outros ———————
// LED da carroceria (MQTT)
LED led_carro(2);
// Gerenciador de energia
PowerManager powerManager;

// void http_stop_carro(AsyncWebServerRequest *request) {
void http_stop_carro() {
	ponte->stop();
	lastReading = millis();
	// request->send(200, "application/json", "{\"status\":\"stopped\"}");
	server.send(200, "application/json", "{\"status\":\"stopped\"}");
}

// void http_handle_forward(AsyncWebServerRequest *request) {
void http_handle_forward() {
	ponte->forward();
	lastReading = millis();
	// request->send(200, "application/json", "{\"status\":\"moving forward\"}");
	server.send(200, "application/json", "{\"status\":\"moving forward\"}");
}

// void http_handle_backward(AsyncWebServerRequest *request) {
void http_handle_backward() {
	ponte->backward();
	lastReading = millis();
	// request->send(200, "application/json", "{\"status\":\"moving backward\"}");
	server.send(200, "application/json", "{\"status\":\"moving backward\"}");
}

// void http_handle_turn_left(AsyncWebServerRequest *request) {
void http_handle_turn_left() {
	ponte->turnLeft();
	lastReading = millis();
	// request->send(200, "application/json", "{\"status\":\"turning left\"}");
	server.send(200, "application/json", "{\"status\":\"turning left\"}");
}

// void http_handle_turn_right(AsyncWebServerRequest *request) {
void http_handle_turn_right() {
	ponte->turnRight();
	lastReading = millis();
	// request->send(200, "application/json", "{\"status\":\"turning right\"}");
	server.send(200, "application/json", "{\"status\":\"turning right\"}");
}

// void http_handle_test(AsyncWebServerRequest *request) {
void http_handle_test() {
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
	html += "<button onclick=\"fetchURL('/test')\">Test</button>";
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
	// request->send(200, "text/html", html);
	server.send(200, "text/html", html);
}

// void handleData(AsyncWebServerRequest *request) {
void handleData() {
	String json = "{";

	// Dados do MPU6050
	json += "\"sensorMPU\":{";
		json += "\"accX\":" + String(sensorMPU->getAccelerometerX()) + ",";
		json += "\"accY\":" + String(sensorMPU->getAccelerometerY()) + ",";
		json += "\"accZ\":" + String(sensorMPU->getAccelerometerZ()) + ",";
		json += "\"gyroX\":" + String(sensorMPU->getGyroscopeX()) + ",";
		json += "\"gyroY\":" + String(sensorMPU->getGyroscopeY()) + ",";
		json += "\"gyroZ\":" + String(sensorMPU->getGyroscopeZ()) + ",";
		json += "\"temp\":"  + String(sensorMPU->getTemperatureC());
	json += "},";

	// Distância VL53L0X
	json += "\"sensorVL53L0X\":{";
		json += "\"distance\":" + String(sensor->loop());
	json += "},";

	// Status do LED do carrinho
	json += "\"led_carro\":{";
		json += "\"status\":\"" + String(led_carro.isOn() ? "on" : "off") + "\"";
	json += "},";

	// Modo de energia
	json += "\"power_manager\":{";
		json += "\"power_mode\":\"" + String(powerManager.isPowerSaving() ? "saving" : "normal") + "\"";
	json += "},";

	// Motor Direito
	json += "\"direcaoD\":\"" + String(encoderD->isClockwise() ? "frente" : "tras") + "\",";
	json += "\"rpmD\":" + String(encoderD->getRPM()) + ",";

	// Motor Esquerdo
	json += "\"direcaoE\":\"" + String(encoderE->isClockwise() ? "frente" : "tras") + "\",";
	json += "\"rpmE\":" + String(encoderE->getRPM());

	json += "}";

	lastReading = millis();

	// request->send(200, "application/json", json);
	server.send(200, "application/json", json);
}

// void handlePower(AsyncWebServerRequest *request) {
void handlePower() {
	// if (request->hasParam("mode")) {
	// 	String mode = request->getParam("mode")->value();
	// 	if (mode == "normal") {
	// 		powerManager.setPowerMode(POWER_NORMAL);
	// 	} else if (mode == "saving") {
	// 		powerManager.setPowerMode(POWER_SAVING);
	// 	}
	// }
	String json = "{\"power_mode\":\"" + String(powerManager.isPowerSaving() ? "saving" : "normal") + "\"}";
	// request->send(200, "application/json", json);
	server.send(200, "application/json", json);
	lastReading = millis();
}

// void handle_base(AsyncWebServerRequest *request) {
void handle_base() {
	if (true) {
		return http_handle_test();
	}
	// lastReading = millis();
	// request->send(SPIFFS, "/dashboard.html", "text/html");
}

bool isIdle() {
	// Verifica se ambos os motores estão parados e sem movimento
	return ponte->isStopped();
}

void prepare_http_server() {
	if (true) {
		server.on("/", handle_base);
		server.on("/data", handleData);
		server.on("/forward", http_handle_forward);
		server.on("/backward", http_handle_backward);
		server.on("/turn_left", http_handle_turn_left);
		server.on("/turn_right", http_handle_turn_right);
		server.on("/stop", http_stop_carro);
		server.on("/test", http_handle_test);

		server.begin();
		return;
	}
	// if (!SPIFFS.begin(true)) {
	// 	#ifdef DEBUG_PRINTS
	// 	Serial.println("SPIFFS Mount Failed");
	// #endif
	// 	return;
	// }
	// #ifdef DEBUG_PRINTS
	// 	Serial.println("SPIFFS Mounted!");
	// #endif

	// server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
	// 	if (!SPIFFS.exists("/dashboard.html")) {
	// 		request->send(404, "text/plain", "dashboard.html não encontrado");
	// 		#ifdef DEBUG_PRINTS
	// 	Serial.println("dashboard.html não encontrado");
	// #endif
	// 		return;
	// 	}
	// 	request->send(SPIFFS, "/dashboard.html", String(), false);
	// 	#ifdef DEBUG_PRINTS
	// 	Serial.println("dashboard.html enviado");
	// #endif
	// });
	// server.on("/data", HTTP_GET, handleData);
	// server.on("/forward", HTTP_GET, http_handle_forward);
	// server.on("/backward", HTTP_GET, http_handle_backward);
	// server.on("/turn_left", HTTP_GET, http_handle_turn_left);
	// server.on("/turn_right", HTTP_GET, http_handle_turn_right);
	// server.on("/stop", HTTP_GET, http_stop_carro);
	// server.on("/test", HTTP_GET, http_handle_test);

	// // Inicia servidor
	// server.begin();
	// #ifdef DEBUG_PRINTS
	// 	Serial.println("Servidor HTTP iniciado");
	// #endif

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

	Wire.begin();

	ponte->setup();
	sensor->setup();
	sensorMPU->setup();

	#ifdef DEBUG_PRINTS
		Serial.println("Setup completo!");
	#endif
	// WiFi.mode(WIFI_STA);
	ConnectToWiFi();

	prepare_http_server();

	lastReading = millis();
}

unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 20;  // ms

void loop() {
	if (WiFi.status() != WL_CONNECTED) {
		#ifdef DEBUG_PRINTS
			Serial.println("WiFi desconectado, tentando reconectar...");
		#endif
		ponte->stop();
		ConnectToWiFi();
		return;
	}

	server.handleClient();

	unsigned long now = millis();
	if (now - lastLoopTime >= loopInterval) {
		lastLoopTime = now;

		// Atualiza sensores
		sensorMPU->loop();

		// Atualiza ponte H e motores
		ponte->loop(sensor, sensorMPU);

		lastReading = millis();
	}
}
