/*
	Trabalho de conclusão de Curso - UNIFESP - Campus São José dos Campos
	Alunos: Henrique Campanha Garcia
	Professor Orientador: André Marcorin
	Universidade: UNIFESP - Campus São José dos Campos
	Ideia do projeto: Utilizar de uma carcaça de um carrinho de controle remoto para controlar o carrinho para de modo remoto, fazer leituras de sensores para monitoramento de temperatura, umidade e luminosidade, para então processar os dados para saber se a central deveria ligar ou desligar luzes, ventuinhas e etc em uma lavoura. Os dados são enviados para um servidor MQTT para monitoramento a distância.
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
	- 1 sensor MPU6050 --> Utilizado para medir a aceleração e Giroscopio do carrinho.
		* Pino SDA --> 21
		* Pino SCL --> 22
	- 1 controle de videogame --> Utilizado para controlar o carrinho.
		* Bluepad32, sem pino físico
	- 1 bateria de 9V --> Utilizado para alimentar a ponte H que alimenta os motores e o ESP32 (via 5V).
		* Externo ao ESP32, ligado na ponte H.
*/



#include <PubSubClient.h>
#include <WiFi.h>
#include <Bluepad32.h> // Soon to be removed
#include <WebServer.h>
#include <SPIFFS.h>

#include <Wire.h>
#include "Adafruit_Sensor.h"
#include "Adafruit_MPU6050.h"
#include "Adafruit_VL53L0X.h"

#define ULTRA_SONIC_READ_INTERVAL 100
#define READ_INTERVAL 500

const char* ssid = "Canguru";
const char* password = "VamoPula";

GamepadPtr myGamepads[BP32_MAX_GAMEPADS]; // Array de Gamepads, para um autonomo, não precisa, remover depois

// This callback gets called any time a new gamepad is connected.
// Up to 4 gamepads can be connected at the same time.
void onConnectedGamepad(GamepadPtr gp) {
	bool foundEmptySlot = false;

	for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
		if (myGamepads[i] == nullptr) {
			Serial.printf("CALLBACK: Gamepad is connected, index=%d\n", i);
			// Additionally, you can get certain gamepad properties like:
			// Model, VID, PID, BTAddr, flags, etc.
			GamepadProperties properties = gp->getProperties();
			Serial.printf("Gamepad model: %s, VID=0x%04x, PID=0x%04x\n",
										gp->getModelName().c_str(), properties.vendor_id,
										properties.product_id);
			myGamepads[i] = gp;
			foundEmptySlot = true;
			break;
		}
	}
	if (!foundEmptySlot) {
		Serial.println(
			"CALLBACK: Gamepad connected, but could not found empty slot");
	}
}

void onDisconnectedGamepad(GamepadPtr gp) {
	bool foundGamepad = false;

	for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
		if (myGamepads[i] == gp) {
			Serial.printf("CALLBACK: Gamepad is disconnected from index=%d\n", i);
			myGamepads[i] = nullptr;
			foundGamepad = true;
			break;
		}
	}

	if (!foundGamepad) {
		Serial.println(
			"CALLBACK: Gamepad disconnected, but not found in myGamepads");
	}
}

unsigned long lastDump = 0;
void dumpGamepad(ControllerPtr ctl) {
	if (millis() - lastDump < 100) {
		return;
	}
	lastDump = millis();
	Serial.printf(
		"idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
		"misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
		ctl->index(),		// Controller Index
		ctl->dpad(),		 // D-pad
		ctl->buttons(),	  // bitmask of pressed buttons
		ctl->axisX(),		// (-511 - 512) left X Axis
		ctl->axisY(),		// (-511 - 512) left Y axis
		ctl->axisRX(),	   // (-511 - 512) right X axis
		ctl->axisRY(),	   // (-511 - 512) right Y axis
		ctl->brake(),		// (0 - 1023): brake button
		ctl->throttle(),	 // (0 - 1023): throttle (AKA gas) button
		ctl->miscButtons(),  // bitmask of pressed "misc" buttons
		ctl->gyroX(),		// Gyro X
		ctl->gyroY(),		// Gyro Y
		ctl->gyroZ(),		// Gyro Z
		ctl->accelX(),	   // Accelerometer X
		ctl->accelY(),	   // Accelerometer Y
		ctl->accelZ()		// Accelerometer Z
	);
}

WiFiClient wifiClient;

PubSubClient mqttClient(wifiClient);
WebServer server(80);

void mqttConnect() {
	char *clientId = "CLIENTID-UNIFESP-2024/2";
	char *username = "FLESPI-API_KEY REMOVED FROM HERE (ALSO DELETED)";
	char *password = "";

	while (!mqttClient.connected()) {
		if (mqttClient.connect(clientId, username, password)) {
			Serial.println("Connected to MQTT broker.");
		}
	}
}

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
		return g.gyro.x;
	}
	float getGyroscopeY() {
		sensors_event_t g = this->getGyroscope();
		return g.gyro.y;
	}
	float getGyroscopeZ() {
		sensors_event_t g = this->getGyroscope();
		return g.gyro.z;
	}
	float getTemperatureC() {
		sensors_event_t temp = this->getTemperature();
		return temp.temperature;
	}

	void loop() {} // Não faz nada ainda, mas fará?
};

class Motor {
private:
// Status: Funcionando
	int pin1, pin2;
public:
	Motor(int pin1, int pin2) {
		this->pin1 = pin1;
		this->pin2 = pin2;
	}

	int getPin1() {
		return this->pin1;
	}
	int getPin2() {
		return this->pin2;
	}

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

class ponteH {
private:
// Status: A testar.
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

// No loop:
	// delay(500);
	// mqttClient.publish("/srs/usrs/LUISAO-MandaPix-10pila/IoT", "1");

Encoder* Encoder::instance = nullptr;

VL53L0X *sensor = new VL53L0X();
MPU6050 *sensorMPU = new MPU6050();

Motor *motorD = new Motor(12, 13);
Motor *motorE = new Motor(33, 25);
Encoder *encoderD = new Encoder(35, 34);
Encoder *encoderE = new Encoder(32, 39);
ponteH *ponte = new ponteH(motorD, motorE, encoderD, encoderE);

// MQTT mqtt = MQTT("mqtt.flespi.io");
LED led_carro = LED(2); // Led da Lavoura, controlado pelo MQTT

void MQTT_Callback(char* topic, byte* payload, unsigned int length) {
	// if (1 == 1) {
	// 	Serial.println("ReceivedMessage! But I was not implemented to do anything with it ;-;, sorry...");
	// 	return;
	// }
	char message[100];
	Serial.println("ReceivedMessage!");	
	Serial.print("Message arrived [");
	for (int i = 0; i < length; i++) {
		message[i] = (char)payload[i];
		Serial.print((char)payload[i]);
	}
	Serial.println("]");

	Serial.println(topic);

	if (strcmp(topic, "/Henrique/IoT/TF/LED_CARRO") == 0) {
		Serial.println("Topic found!");
		Serial.print("Message: ");
		Serial.println(message);
		if (strcmp(message, "1") == 0) {
			led_carro.on();
		} else if (message[0] == '2') { 
			led_carro.toggle();
			mqttClient.publish("/Henrique/IoT/TF/LED_CARRO/Status", led_carro.isOn() ? "1" : "0");
		} else {
			led_carro.off();
		}
	} else {
		Serial.println("Topic not found!");
	}
}

void handleData() {
	String json = "{";
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

	server.send(200, "application/json", json);
}

void bluepadbt_setup() {
	// This function will be removed at the end of the project, it's for the beginning of the project.
	Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
	const uint8_t *addr = BP32.localBdAddress();
	Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);


	// Setup the Bluepad32 callbacks
	BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

	// "forgetBluetoothKeys()" should be called when the user performs  a "device factory reset", or similar.
	// Calling "forgetBluetoothKeys" in setup() just as an example.
	// Forgetting Bluetooth keys prevents "paired" gamepads to reconnect.
	// But might also fix some connection / re-connection issues.
	BP32.forgetBluetoothKeys();
	Serial.println("Bluepad32 setup complete!");
}

bool isTurning = false;
bool isMovingBackward = false;

void setup() {
	Serial.begin(115200);

	if (!SPIFFS.begin(true)) {
		Serial.println("SPIFFS Mount Failed");
		return;
	}

	ConnectToWiFi();

	bluepadbt_setup();

	led_carro.setup();
	Serial.println("Iniciando conexão com o MQTT...");

	// mqttClient.setServer("mqtt.flespi.io", 1883);
	// mqttClient.setCallback(MQTT_Callback);
	// mqttConnect();
	// mqttClient.subscribe("/Henrique/IoT/TF/LED_CARRO");

	Serial.println("Connected to MQTT broker!");

	// carro.setup();

	ponte->setup();
	sensor->setup();
	sensorMPU->setup();

	Serial.println("Setup completo!");

	// Define rota raiz
	server.serveStatic("/", SPIFFS, "/dashboard.html");
	server.on("/data", HTTP_GET, handleData);

	// Inicia servidor
	server.begin();
	Serial.println("Servidor HTTP iniciado");
}

unsigned long lastRead = 0;

void loop() {
	server.handleClient();

	if (millis() - lastRead < 1000) {
		return;
	}
	lastRead = millis();

	int front_distance = sensor->loop();
	Serial.print("Distancia: ");
	if (front_distance < 100) { // 10 cm
		Serial.print("Obstáculo a ");
		Serial.print(front_distance);
		Serial.println(" mm");
	} else {
		Serial.println("Sem obstáculo");
	}
	Serial.print("Acelerômetro: ");
	Serial.print(sensorMPU->getAccelerometerX());
	Serial.print(" ");
	Serial.print(sensorMPU->getAccelerometerY());
	Serial.print(" ");
	Serial.print(sensorMPU->getAccelerometerZ());
	Serial.println();
	Serial.print("Giroscópio: ");
	Serial.print(sensorMPU->getGyroscopeX());
	Serial.print(" ");
	Serial.print(sensorMPU->getGyroscopeY());
	Serial.print(" ");
	Serial.print(sensorMPU->getGyroscopeZ());
	Serial.println();

	// Encoders:
	ponte->mostrarLeiturasEncoders();
	// Serial.println("Encoders lidos!");

	if (true) {
		return;
	}

	if (!mqttClient.connected()){
		mqttConnect();
	}
	mqttClient.loop();

	// This call fetches all the gamepad info from the NINA (ESP32) module.
	// Just call this function in your main loop.
	// The gamepads pointer (the ones received in the callbacks) gets updated automatically.
	BP32.update();

	// It is safe to always do this before using the gamepad API.
	// This guarantees that the gamepad is valid and connected.
	for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
		GamepadPtr myGamepad = myGamepads[i];

		if (myGamepad && myGamepad->isConnected()) {
			// There are different ways to query whether a button is pressed.
			// By query each button individually: a(), b(), x(), y(), l1(), etc...
			if (myGamepad->a()) {
				static int colorIdx = 0;
				// Some gamepads like DS4 and DualSense support changing the color LED.
				// It is possible to change it by calling:
				switch (colorIdx % 3) {
					case 0:
						// Red
						myGamepad->setColorLED(255, 0, 0);
						break;
					case 1:
						// Green
						myGamepad->setColorLED(0, 255, 0);
						break;
					case 2:
						// Blue
						myGamepad->setColorLED(0, 0, 255);
						break;
				}
				colorIdx++;
			}

			if (myGamepad->b()) {
				// Turn on the 4 LED. Each bit represents one LED.
				static int led = 0;
				led++;
				// Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch support changing the "Player LEDs": those 4 LEDs that usually indicate the "gamepad seat". It is possible to change them by calling:
				myGamepad->setPlayerLEDs(led & 0x0f);
			}

			if (myGamepad->x()) {
				// Duration: 255 is ~2 seconds
				// force: intensity
				// Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support rumble.
				// It is possible to set it by calling:
				myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
			}
			dumpGamepad(myGamepad);

			int throttle = myGamepad->throttle();
			int brake = myGamepad->brake();

			if (throttle > 0) {
				if (front_distance < 100) { // 10 cm
					ponte->stop();
					led_carro.on();
					mqttClient.publish("/Henrique/IoT/TF/LED_CARRO/Status", "1");
					continue;
				} else {
					led_carro.off();
					mqttClient.publish("/Henrique/IoT/TF/LED_CARRO/Status", "0");
				}
				// ponte->forward_percent(throttle);
				ponte->forward();
			} else if (brake > 0) {
				// ponte->backward_percent(brake);
				ponte->backward();
			} else {
				int axisX = myGamepad->axisX();
				if (axisX < -100) { // Esquerda!
					ponte->turnRight(); // Os motores estão invertidos...
				} else if (axisX > 100) { // Direita!
					ponte->turnLeft(); // Motores invertidos...
				} else {
					ponte->stop();
				}
			}
		}
	}

	// carro.loop();
	// mqtt.loop();
}
