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
			| Pino 35 --> Ponte H.
			| Pino 32 --> Ponte H.
		* Motor 2: Motor esquerdo.
			| Pino 33 --> Ponte H.
			| Pino 25 --> Ponte H.
	- 2 encoders --> Utilizado para medir a velocidade do carrinho.
		* Encoder 1: Motor direito.
			| Pino 26 --> Encoder.
			| Pino 27 --> Encoder.
		* Encoder 2: Motor esquerdo.
			| Pino 18 --> Encoder.
			| Pino 19 --> Encoder.
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

const char* ssid = "Canguru";
const char* password = "VamoPula";

AsyncWebServer server(80);

typedef enum { POWER_NORMAL = 0, POWER_SAVING = 1 } power_mode_t;

#include "esp_pm.h"
class PowerManager {
private:
	power_mode_t power_mode;
public:
	PowerManager() : power_mode(POWER_NORMAL) {}

	void setPowerMode(power_mode_t mode) {
		if (mode == POWER_SAVING) {
			esp_pm_config_esp32_t pm_config = {
				.min_freq_mhz = 80,
				.max_freq_mhz = 160,
				.light_sleep_enable = true
			};
			esp_pm_configure(pm_config);
			setPowerSaving();
		} else {
			esp_pm_config_esp32_t pm_config = {
				.min_freq_mhz = 80,
				.max_freq_mhz = 240,
				.light_sleep_enable = false
			};
			esp_pm_configure(pm_config);
			setPowerNormal();
		}
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

	bool setPowerSaving() {
		if (this->power_mode == POWER_SAVING) {
			Serial.println("Modo de energia já está em ECONOMIA");
			return false;
		}
		this->setPowerMode(POWER_SAVING);
		Serial.println("Colocando WiFi em modo de economia...");
		WiFi.setPowerSave(WIFI_PS_MAX_MODEM);
		Serial.println("WiFi em modo de economia!");
		Serial.println("Colocando I2C em modo de economia...");
		Wire.setClock(10); // 10kHz
		Serial.println("I2C em modo de economia!");
		Serial.println("Modo de energia alterado para ECONOMIA");
		return true;
	}
	bool setPowerNormal() {
		if (this->power_mode == POWER_NORMAL) {
			Serial.println("Modo de energia já está em NORMAL");
			return false;
		}
		this->setPowerMode(POWER_NORMAL);
		Serial.println("Reiniciando WiFi...");
		WiFi.setPowerSave(WIFI_PS_NONE);
		Serial.println("WiFi reiniciado!");
		Serial.println("Reiniciando I2C...");
		Wire.begin();
		Wire.setClock(100000); // 100kHz --> Clock padrão do I2C
		Serial.println("I2C reiniciado!");
		Serial.println("Modo de energia alterado para NORMAL");
		return true;
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

class Motor {
private:
	int pin1, pin2;
public:
	Motor(int pin1, int pin2) {
		this->pin1 = pin1;
		this->pin2 = pin2;
	}

	int getPin1() { return this->pin1; }
	int getPin2() { return this->pin2; }

	void setup() {
		pinMode(this->pin1, OUTPUT);
		pinMode(this->pin2, OUTPUT);
		digitalWrite(this->pin1, LOW);
		digitalWrite(this->pin2, LOW);
	}

	void forward() {
		digitalWrite(this->pin1, HIGH);
		digitalWrite(this->pin2, LOW);
	}

	void backward() {
		digitalWrite(this->pin1, LOW);
		digitalWrite(this->pin2, HIGH);
	}

	void stop() {
		digitalWrite(this->pin1, LOW);
		digitalWrite(this->pin2, LOW);
	}
};

class Encoder {
private:
	int contadorCH1 = 0, contadorCH2 = 0;
	int pinoCH1, pinoCH2;
	int ultimoEstadoCH2 = LOW;
	bool isSentidoHorario = true;

	static Encoder* instance;
	static void isrCH1() { instance->incrementarContadorCH1(); }
	static void isrCH2() { instance->incrementarContadorCH2(); }

	void incrementarContadorCH1() { contadorCH1++; }
	void incrementarContadorCH2() {
		contadorCH2++;
		int estadoAtual = digitalRead(pinoCH2);
		if (ultimoEstadoCH2 == LOW && estadoAtual == HIGH) {
			isSentidoHorario = (digitalRead(pinoCH1) == LOW);
		}
		ultimoEstadoCH2 = estadoAtual;
	}

public:
	Encoder(int ch1, int ch2) : pinoCH1(ch1), pinoCH2(ch2) {
		instance = this;
	}
	void setup() {
		pinMode(pinoCH1, INPUT_PULLUP);
		pinMode(pinoCH2, INPUT_PULLUP);
		attachInterrupt(digitalPinToInterrupt(pinoCH1), isrCH1, CHANGE);
		attachInterrupt(digitalPinToInterrupt(pinoCH2), isrCH2, CHANGE);
    }
	bool getSentidoHorario() { return isSentidoHorario; }
	int getContadorCH1() { return contadorCH1; }
	int getContadorCH2() { return contadorCH2; }
	int getVelocidadeRPM(int dentes = 6) {
		int media = (contadorCH1 + contadorCH2) / 2;
		return media / (dentes * 2);
	}

	void exibirDados() {
		Serial.print("Sentido: ");
		Serial.print(isSentidoHorario ? "horario" : "anti-horario");
		Serial.print(" | Contador CH1: ");
		Serial.print(contadorCH1);
		Serial.print(" | Contador CH2: ");
		Serial.print(contadorCH2);
		Serial.print(" | Velocidade: ");
		Serial.print(getVelocidadeRPM());
		Serial.println(" RPM");
	}
};
Encoder* Encoder::instance = nullptr;

class ponteH {
private:
	Motor *motorD, *motorE;
	Encoder *encoderD, *encoderE;

public:
	ponteH(Motor *motor_d, Motor *motor_e, Encoder *encoder_d, Encoder *encoder_e) {
		this->motorD = motor_d;
		this->motorE = motor_e;
		this->encoderD = encoder_d;
		this->encoderE = encoder_e;
	}

	void setup() {
		this->motorD->setup();
		this->motorE->setup();
		this->encoderD->setup();
		this->encoderE->setup();
	}

	void forward() {
		this->motorD->forward();
		this->motorE->forward();
	}

	void backward() {
		this->motorD->backward();
		this->motorE->backward();
	}

	void turnLeft() {
		this->motorD->forward();
		this->motorE->backward();
	}

	void turnRight() {
		this->motorD->backward();
		this->motorE->forward();
	}

	void stop() {
		this->motorD->stop();
		this->motorE->stop();
	}

	void mostrarLeiturasEncoders() {
		Serial.print("Motor D: ");
		this->encoderD->exibirDados();
		Serial.print("Motor E: ");
		this->encoderE->exibirDados();
		Serial.println();
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

VL53L0X *sensor = new VL53L0X();
MPU6050 *sensorMPU = new MPU6050();
Motor *motorD = new Motor(12, 13);
Motor *motorE = new Motor(33, 25);
Encoder *encoderD = new Encoder(35, 34);
Encoder *encoderE = new Encoder(32, 39);
ponteH *ponte = new ponteH(motorD, motorE, encoderD, encoderE);
LED led_carro = LED(2); // Led da Lavoura, controlado pelo MQTT
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
	json += "\"sensorMPU\":{\"accX\":" + String(sensorMPU->getAccelerometerX()) + ",";
	json += "\"accY\":" + String(sensorMPU->getAccelerometerY()) + ",";
	json += "\"accZ\":" + String(sensorMPU->getAccelerometerZ()) + ",";
	json += "\"gyroX\":" + String(sensorMPU->getGyroscopeX()) + ",";
	json += "\"gyroY\":" + String(sensorMPU->getGyroscopeY()) + ",";
	json += "\"gyroZ\":" + String(sensorMPU->getGyroscopeZ()) + ",";
	json += "\"temp\":" + String(sensorMPU->getTemperatureC()) + "},";
	json += "\"sensorVL53L0X\":{\"distance\":" + String(sensor->loop()) + "},";
	json += "\"led_carro\":{\"status\":\"" + String(led_carro.isOn() ? "on" : "off") + "\"},";
	json += "\"power_manager\":{\"power_mode\":\"" + String(powerManager.isPowerSaving() ? "saving" : "normal") + "\"},";
	// Motor Direito
	json += "\"direcaoD\":\"" + String( encoderD->getSentidoHorario() ? "frente" : "tras") + "\",";
	json += "\"rpmD\":"    + String( encoderD->getVelocidadeRPM()) + ",";
	json += "\"contD1\":"  + String( encoderD->getContadorCH1()) + ",";
	json += "\"contD2\":"  + String( encoderD->getContadorCH2()) + ",";
	// Motor Esquerdo
	json += "\"direcaoE\":\"" + String( encoderE->getSentidoHorario() ? "frente" : "tras") + "\",";
	json += "\"rpmE\":"    + String( encoderE->getVelocidadeRPM()) + ",";
	json += "\"contE1\":"  + String( encoderE->getContadorCH1()) + ",";
	json += "\"contE2\":"  + String( encoderE->getContadorCH2());
	json += "}";
	request->send(200, "application/json", json);
}

void handlePower(AsyncWebServerRequest *request) {
	if (request->hasParam("mode")) {
		String mode = request->getParam("mode")->value();
		if (mode == "normal") {
			powerManager.setPowerNormal();
		} else if (mode == "saving") {
			powerManager.setPowerSaving();
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

	powerManager.setPowerNormal();

	led_carro.setup();

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

void loop() {
	if (millis() - lastRead < 1000) {
		return;
	}
	lastRead = millis();

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

	ponte->mostrarLeiturasEncoders();
}
