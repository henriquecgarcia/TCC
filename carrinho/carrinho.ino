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

#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <SPIFFS.h>

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"
#include "Adafruit_VL53L0X.h"

#include <Arduino.h>
#include <math.h>

const char* ssid = "Canguru";
const char* password = "VamoPula";

AsyncWebServer server(80);

enum power_mode_t {
	POWER_NORMAL,
	POWER_SAVING
};

#include "esp_pm.h"
#include "esp_wifi.h"
class PowerManager {
private:
	power_mode_t power_mode;

	void configurePowerSettings(power_mode_t mode) {
		if (mode == POWER_SAVING) {
			esp_pm_config_esp32_t pmConfig = {
				160, // max_freq_mhz
				80,  // min_freq_mhz
				true // light_sleep_enable
			};
			esp_err_t err = esp_pm_configure(&pmConfig);
			Serial.printf("esp_pm_configure (ECONOMIA): %d\n", err);
		} else {
			esp_pm_config_esp32_t pmConfig = {
				240, // max_freq_mhz
				80,  // min_freq_mhz
				false // light_sleep_enable
			};
			esp_err_t err = esp_pm_configure(&pmConfig);
			Serial.printf("esp_pm_configure (NORMAL): %d\n", err);
		}
	}

public:
	PowerManager() : power_mode(POWER_NORMAL) {}

	bool setPowerMode(power_mode_t mode) {
		if (mode == this->power_mode) {
			Serial.println("Já está no modo solicitado.");
			return false;
		}

		configurePowerSettings(mode);
		this->power_mode = mode;

		if (mode == POWER_SAVING) {
			Serial.println("Colocando WiFi em modo de economia...");
			esp_wifi_set_ps(WIFI_PS_MAX_MODEM);
			Serial.println("WiFi em modo de economia!");

			// Serial.println("Colocando I2C em modo de economia...");
			// Wire.setClock(10); // 10kHz
			// Serial.println("I2C em modo de economia!");

			Serial.println("Modo de energia alterado para ECONOMIA");
		} else {
			Serial.println("Reiniciando WiFi...");
			esp_wifi_set_ps(WIFI_PS_NONE);
			Serial.println("WiFi reiniciado!");

			// Serial.println("Reiniciando I2C...");
			// Wire.begin();
			// Wire.setClock(100000); // 100kHz padrão
			// Serial.println("I2C reiniciado!");

			Serial.println("Modo de energia alterado para NORMAL");
		}
		return true;
	}

	power_mode_t getPowerMode() { return this->power_mode; }

	void printPowerMode() {
		if (this->power_mode == POWER_NORMAL) {
			Serial.println("Modo de energia: NORMAL");
		} else {
			Serial.println("Modo de energia: ECONOMIA");
		}
	}

	bool isPowerSaving() {
		return this->power_mode == POWER_SAVING;
	}
};

class LED {
private:
	int pin;
public:
	LED(int pin) {
		this->pin = pin;
	}

	void setup() {
		pinMode(this->pin, OUTPUT);
		digitalWrite(this->pin, LOW);
	}

	void on() {
		digitalWrite(this->pin, HIGH);
	}

	void off() {
		digitalWrite(this->pin, LOW);
	}

	bool isOn() {
		return digitalRead(this->pin) == HIGH;
	}

	void toggle() {
		Serial.print("Toggling LED ");
		Serial.println(this->pin);
		digitalWrite(this->pin, !digitalRead(this->pin));
	}
};

class VL53L0X {
private:
	Adafruit_VL53L0X *sensor;
	unsigned long lastRead;
	int last_reading;
public:
	VL53L0X() {
		this->sensor = new Adafruit_VL53L0X();
		this->lastRead = 0;
	}

	void setup() {
		if (!this->sensor->begin()) {
			Serial.println("Falha ao encontrar o sensor VL53L0X");
			while (1);
		}
		Serial.println("Sensor VL53L0X encontrado!");
	}

	int loop() {
		if (millis() - this->lastRead < 100) {
			return this->last_reading;
		}
		this->lastRead = millis();
		VL53L0X_RangingMeasurementData_t measure;
		this->sensor->rangingTest(&measure, false);
		Serial.print("Distancia: ");
		if (measure.RangeStatus != 4) { // if not out of range
			this->last_reading = measure.RangeMilliMeter;
			Serial.print(measure.RangeMilliMeter);
			Serial.println(" mm");
			return this->last_reading;
		} else {
			Serial.println("Fora do alcance");
			return 99999;
		}
	}
};

class MPU6050 {
private:
	Adafruit_MPU6050 *sensor;
public:
	MPU6050() {
		this->sensor = new Adafruit_MPU6050();
	}

	void setup() {
		if (!this->sensor->begin()) {
			Serial.println("Falha ao encontrar o sensor MPU6050");
			while (1);
		}
		this->sensor->setAccelerometerRange(MPU6050_RANGE_2_G);
		this->sensor->setGyroRange(MPU6050_RANGE_500_DEG);
		this->sensor->setFilterBandwidth(MPU6050_BAND_21_HZ);
	}

	sensors_event_t getAccelerometer() {
		sensors_event_t a, g, temp;
		this->sensor->getEvent(&a, &g, &temp);
		return a;
	}
	sensors_event_t getGyroscope() {
		sensors_event_t a, g, temp;
		this->sensor->getEvent(&a, &g, &temp);
		return g;
	}
	sensors_event_t getTemperature() {
		sensors_event_t a, g, temp;
		this->sensor->getEvent(&a, &g, &temp);
		return temp;
	}

	float getAccelerometerX() {
		sensors_event_t a = this->getAccelerometer();
		return a.acceleration.x;
	}
	float getAccelerometerY() {
		sensors_event_t a = this->getAccelerometer();
		return a.acceleration.y;
	}
	float getAccelerometerZ() {
		sensors_event_t a = this->getAccelerometer();
		return a.acceleration.z;
	}

	float getGyroscopeX() {
		sensors_event_t g = this->getGyroscope();
		return g.gyro.x * 10.0f * 1229.0f / 4096.0f + 18.0f;
	}
	float getGyroscopeY() {
		sensors_event_t g = this->getGyroscope();
		return g.gyro.y * 10.0f * 1229.0f / 4096.0f + 70.0f;
	}
	float getGyroscopeZ() {
		sensors_event_t g = this->getGyroscope();
		return g.gyro.z * 10.0f * 1229.0f / 4096.0f + 270.0f;
	}
	float getTemperatureC() {
		sensors_event_t temp = this->getTemperature();
		return temp.temperature;
	}

	void loop() {} // Não faz nada ainda, mas fará?
};

class Encoder {
private:
	volatile int countA = 0, countB = 0;
	uint8_t pinA, pinB;
	int lastStateB = LOW;
	bool clockwise = true;

	// ISRs sem IRAM_ATTR
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

#include <PID_v1_bc.h>
class Motor {
private:
	unsigned long lastDebug = 0;
	int in1Pin, in2Pin;
	int pwmPin;
	Encoder* encoder;

	double targetRPM = 100.0;
	double currentRPM = 0.0;
	double pidOutput = 0.0;
	PID* pid;

	unsigned long last_think = 0;

public:
	Motor(int in1, int in2, int pwm, Encoder* enc, double kp = 1.0, double ki = 5.0, double kd = 0.0) {
		in1Pin = in1;
		in2Pin = in2;
		pwmPin = pwm;
		encoder = enc;
		pid = new PID(&currentRPM, &pidOutput, &targetRPM, kp, ki, kd, DIRECT);
	}

	void begin() {
		pinMode(in1Pin, OUTPUT);
		pinMode(in2Pin, OUTPUT);
		pinMode(pwmPin, OUTPUT);
		encoder->begin();
		encoder->reset();
		stop();
		pid->SetMode(AUTOMATIC);
		// pid->SetOutputLimits(0, 255);
	}

	void setTunings(double kp, double ki, double kd) {
		pid->SetTunings(kp, ki, kd);
	}
	void setTargetRPM(double rpm) {
		targetRPM = rpm;
	}

	void forward() {
		encoder->reset();
		digitalWrite(in1Pin, HIGH);
		digitalWrite(in2Pin, LOW);

		analogWrite(pwmPin, int(255/2));
		last_think = millis();
	}
	void backward() {
		encoder->reset();
		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, HIGH);

		analogWrite(pwmPin, int(255/2));
		last_think = millis();
	}
	void stop() {
		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, LOW);
		// analogWrite(pwmPin, 0);
		encoder->reset();
	}

	void update(double intervalSec) {
		if (millis() - 500 < last_think)
			return;
		last_think = millis();
		double lastPidOutput = pidOutput;
		currentRPM = encoder->getRPM(10, intervalSec);
		pid->Compute();
		if (millis() - lastDebug >= 500) {
			Serial.print(pwmPin);
			Serial.print(") - ");
			Serial.print("Current RPM: ");
			Serial.print(currentRPM);
			Serial.print(" | Privous output: ");
			Serial.print(lastPidOutput);
			Serial.print(" - New Output: ");
			Serial.println(pidOutput);
			lastDebug = millis();
		}
		analogWrite(pwmPin, int(pidOutput));
		encoder->reset();
	}
};


enum Movement { FORWARD, BACKWARD, TURN_LEFT, TURN_RIGHT };
class PonteH {
private:
	Motor* motorRight;
	Motor* motorLeft;
	Movement currentMove = FORWARD;
	bool isMoving = false;

public:
	PonteH(Motor* right, Motor* left) {
		this->motorRight = right;
		this->motorLeft = left;
	}

	void setup() {
		motorRight->begin();
		motorLeft->begin();
	}

	void forward() {
		if (currentMove != FORWARD) stop();
		currentMove = FORWARD;
		motorRight->forward();
		motorLeft->forward();
		isMoving = true;
	}

	void backward() {
		if (currentMove != BACKWARD) stop();
		currentMove = BACKWARD;
		motorRight->backward();
		motorLeft->backward();
		isMoving = true;
	}

	void turnLeft() {
		if (currentMove != TURN_LEFT) stop();
		currentMove = TURN_LEFT;
		motorRight->forward();
		motorLeft->backward();
		isMoving = true;
	}

	void turnRight() {
		if (currentMove != TURN_RIGHT) stop();
		currentMove = TURN_RIGHT;
		motorRight->backward();
		motorLeft->forward();
		isMoving = true;
	}

	void stop() {
		currentMove = FORWARD;
		motorRight->stop();
		motorLeft->stop();
		isMoving = false;
	}

	void updateAll(double intervalSec) {
		if (!isMoving) return;
		motorRight->update(intervalSec);
		motorLeft->update(intervalSec);
	}
};

void ConnectToWiFi(){
	WiFi.begin(ssid, password);
	Serial.print("Conectando ao WiFi -> ");
	Serial.print(ssid);
	while (WiFi.status() != WL_CONNECTED) {
		Serial.print(".");
		delay(500);
	}
	Serial.println(" WiFi conectado!");
	Serial.print("Endereco IP: ");
	Serial.println(WiFi.localIP());
}
// ——————— Sensores ———————
VL53L0X *sensor = new VL53L0X();	// VL53L0X no I²C (SDA=21, SCL=22)
MPU6050 *sensorMPU = new MPU6050();	// MPU6050 no mesmo barramento I²C

// ——————— Encoders ———————
// Motor Direito
Encoder *encoderD = new Encoder(27, 26);  // CH A=26, CH B=27
// Motor Esquerdo
Encoder *encoderE = new Encoder(35, 34);  // CH A=35, CH B=34

// ——————— Motores com PID ———————
// Motor Direito  → IN1=12, IN2=13, PWM=14, canal LEDC=0
Motor *motorDireito  = new Motor( 12, 13, 14, encoderD, 1, 0.5, 0.0 );
// Motor Esquerdo → IN1=33, IN2=25, PWM=32, canal LEDC=1
Motor *motorEsquerdo = new Motor( 33, 25, 32, encoderE, 1, 0.5, 0.0 );

// ——————— Ponte H (drive de 2 motores) ———————
PonteH *ponte = new PonteH(motorDireito, motorEsquerdo);

// ——————— Outros ———————
// LED da carroceria (MQTT)
LED led_carro(2);
// Gerenciador de energia
PowerManager powerManager;

void http_stop_carro(AsyncWebServerRequest *request) {
	ponte->stop();
	request->send(200, "application/json", "{\"status\":\"stopped\"}");
}

void http_handle_forward(AsyncWebServerRequest *request) {
	ponte->forward();
	request->send(200, "application/json", "{\"status\":\"moving forward\"}");
}

void http_handle_backward(AsyncWebServerRequest *request) {
	ponte->backward();
	request->send(200, "application/json", "{\"status\":\"moving backward\"}");
}

void http_handle_turn_left(AsyncWebServerRequest *request) {
	ponte->turnLeft();
	request->send(200, "application/json", "{\"status\":\"turning left\"}");
}

void http_handle_turn_right(AsyncWebServerRequest *request) {
	ponte->turnRight();
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
	request->send(200, "text/html", html);
}

void handleData(AsyncWebServerRequest *request) {
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
	request->send(200, "application/json", json);
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
}

void handle_base(AsyncWebServerRequest *request) {
	request->send(SPIFFS, "/dashboard.html", "text/html");
}

unsigned long lastRead = 0;

void setup() {
	Serial.begin(115200);

	powerManager.setPowerMode(POWER_NORMAL);

	led_carro.setup();

	Wire.begin();

	ponte->setup();
	sensor->setup();
	sensorMPU->setup();

	Serial.println("Setup completo!");
	WiFi.mode(WIFI_STA);
	ConnectToWiFi();

	if (!SPIFFS.begin(true)) {
		Serial.println("SPIFFS Mount Failed");
		return;
	}
	Serial.println("SPIFFS Mounted!");

	server.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (!SPIFFS.exists("/dashboard.html")) {
			request->send(404, "text/plain", "dashboard.html não encontrado");
			Serial.println("dashboard.html não encontrado");
			return;
		}
		request->send(SPIFFS, "/dashboard.html", String(), false);
		Serial.println("dashboard.html enviado");
	});
	server.on("/data", HTTP_GET, handleData);
	server.on("/forward", HTTP_GET, http_handle_forward);
	server.on("/backward", HTTP_GET, http_handle_backward);
	server.on("/turn_left", HTTP_GET, http_handle_turn_left);
	server.on("/turn_right", HTTP_GET, http_handle_turn_right);
	server.on("/stop", HTTP_GET, http_stop_carro);
	server.on("/test", HTTP_GET, http_handle_test);

	// Inicia servidor
	server.begin();
	Serial.println("Servidor HTTP iniciado");
	lastRead = millis();
}

unsigned long prevMicros = 0;
void loop() {
	unsigned long now = micros();
	double dt = (now - prevMicros) / 1e6;
	prevMicros = now;

	ponte->updateAll(0.5);
	if (millis() - lastRead < 1000) {
		return;
	}
	lastRead = millis();

	Serial.println("====================================");

	int front_distance = sensor->loop();
	Serial.printf("Distância: ");
	if (front_distance < 100) {
		Serial.printf("Obstáculo a %d mm\n", front_distance);
	} else {
		Serial.println("Sem obstáculo");
	}

	Serial.printf("Acelerômetro: %.2f %.2f %.2f\n",
		sensorMPU->getAccelerometerX(),
		sensorMPU->getAccelerometerY(),
		sensorMPU->getAccelerometerZ());

	Serial.printf("Giroscópio: %.2f %.2f %.2f\n",
		sensorMPU->getGyroscopeX(),
		sensorMPU->getGyroscopeY(),
		sensorMPU->getGyroscopeZ());

	Serial.print("Motor Direito: ");
	Serial.print(encoderD->getRPM());
	Serial.print(" RPM, ");
	Serial.print(encoderD->isClockwise() ? "Frente" : "Tras");
	Serial.print(" | Motor Esquerdo: ");
	Serial.print(encoderE->getRPM());
	Serial.print(" RPM, ");
	Serial.print(encoderE->isClockwise() ? "Frente" : "Tras");
	Serial.println();
	Serial.println("===================================");
	Serial.println();
	Serial.println();

}
