/*
	Trabalho de conclusão de Curso - UNIFESP - Campus São José dos Campos
	Alunos: Henrique Campanha Garcia
	Professor Orientador: André Marcorin
	Universidade: UNIFESP - Campus São José dos Campos
	Ideia do projeto: Utilizar de uma carcaça de um carrinho de controle remoto para controlar o carrinho para de modo remoto, fazer leituras de sensores para monitoramento de temperatura, umidade e luminosidade, para então processar os dados para saber se a central deveria ligar ou desligar luzes, ventuinhas e etc em uma lavoura. Os dados são enviados para um servidor MQTT para monitoramento a distância.
	Itens utilizados:
	- ESP32 --> Controlador do carrinho, que controla tudo.
	- Conversor de nível lógico --> Utilizado para converter o sinal de 5V do HC-SR04 e do DHT11 para 3.3V do ESP32.
	- Sensor Ultrassônico (HC-SR04) --> Sensor de distância. Utilizado para evitar colisões, bloqueando o movimento (para frente) do carrinho caso tenha algo muito perto.
		* Pino 34 --> Conversor de nível lógico --> HC-SR04 [TRIGGER].
		* Pino 39 --> Conversor de nível lógico --> HC-SR04 [ECHO].
	- Ponte H --> Utilizado para controlar os motores do carrinho.
	- 4 motores CC --> Utilizado para movimentar o carrinho.
		* Motor 1: Motor direito.
			| Pino 35 --> Ponte H.
			| Pino 32 --> Ponte H.
		* Motor 2: Motor esquerdo.
			| Pino 33 --> Ponte H.
			| Pino 25 --> Ponte H.
	- 1 controle de videogame --> Utilizado para controlar o carrinho.
		* Bluepad32, sem pino físico
	- 1 bateria de 9V --> Utilizado para alimentar a ponte H que alimenta os motores e o ESP32 (via 5V).
		* Externo ao ESP32, ligado na ponte H.
*/



#include <PubSubClient.h>
#include <WiFi.h>
#include <Bluepad32.h>


#define ULTRA_SONIC_READ_INTERVAL 100
#define READ_INTERVAL 500
#define DHTPIN 22
#define DHTTYPE DHT11
const int ldr_pin = A0;

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

class Sensor { // Classe para o sensor ultrassonico
private:
// Status: Checado e funcionando.
	int echoPin, trigPin;
	int lastValue;
	unsigned long lastRead;
public:
	Sensor(int trigPin, int echoPin) {
		this->echoPin = echoPin;
		this->trigPin = trigPin;
		this->lastRead = 0;
		this->lastValue = 0;
	}

	void setup() {
		pinMode(this->echoPin, INPUT);
		digitalWrite(this->echoPin, LOW);
		pinMode(this->trigPin, OUTPUT);
		digitalWrite(this->trigPin, LOW);
	}

	bool isReading() {
		return (millis() - this->lastRead) < ULTRA_SONIC_READ_INTERVAL;
	}

	int loop() {
		if (this->isReading()) {
			return this->lastValue;
		}
		int triggerStats = digitalRead(this->trigPin);
		if (triggerStats == LOW) {
			digitalWrite(this->trigPin, HIGH);
			this->lastRead = millis();
			return this->lastValue;
		}
		digitalWrite(this->trigPin, LOW);
		int duration = pulseIn(this->echoPin, HIGH);
		int distance = duration / 58.0;
		distance = abs(distance);
		this->lastValue = distance;
		Serial.print("Distancia no sensor ");
		Serial.print(this->echoPin);
		Serial.print(": ");
		Serial.println(distance);
		return distance;
	}
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

class ponteH {
private:
// Status: Funcionando
	Motor *motorD, *motorE;
public:
	ponteH(Motor *motor_d, Motor *motor_e) {
		this->motorD = motor_d;
		this->motorE = motor_e;
	}

	void setup() {
		this->motorD->setup();
		this->motorE->setup();
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

Motor *motorD = new Motor(12, 13);
Motor *motorE = new Motor(33, 25);
ponteH *ponte = new ponteH(motorD, motorE);
// TRIGGER > ECHO
Sensor *SensorFrontal = new Sensor(26, 27);

// MQTT mqtt = MQTT("mqtt.flespi.io");
LED led_carro = LED(2); // Led da Lavoura, controlado pelo MQTT
LED led_estufa = LED(4); // Led da Estufa, controlado pelo MQTT e pelo LDR

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
	} else if (strcmp(topic, "/Henrique/IoT/TF/LED_ESTUFA") == 0) {
		if (strcmp(message, "1") == 0) {
			// led_estufa.on();
		} else if (message[0] == '2') { 
			led_estufa.toggle();
			mqttClient.publish("/Henrique/IoT/TF/LED_ESTUFA/Status", led_estufa.isOn() ? "1" : "0");
		} else {
			led_estufa.off();
		}
	} else {
		Serial.println("Topic not found!");
	}
}

void tinybt_setup() {
	// This function will be removed at the end of the project, it's for the beginning of the project.
	Serial.printf("Firmware: %s\n", BP32.firmwareVersion());
	const uint8_t *addr = BP32.localBdAddress();
	Serial.printf("BD Addr: %2X:%2X:%2X:%2X:%2X:%2X\n", addr[0], addr[1], addr[2], addr[3], addr[4], addr[5]);


	// Setup the Bluepad32 callbacks
	BP32.setup(&onConnectedGamepad, &onDisconnectedGamepad);

	// "forgetBluetoothKeys()" should be called when the user performs
	// a "device factory reset", or similar.
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

	ConnectToWiFi();

	tinybt_setup();

	led_carro.setup();
	led_estufa.setup();
	Serial.println("Iniciando conexão com o MQTT...");

	mqttClient.setServer("mqtt.flespi.io", 1883);
	mqttClient.setCallback(MQTT_Callback);
	mqttConnect();
	mqttClient.subscribe("/Henrique/IoT/TF/LED_CARRO");
	mqttClient.subscribe("/Henrique/IoT/TF/LED_ESTUFA");

	Serial.println("Connected to MQTT broker!");

	// carro.setup();

	ponte->setup();
	SensorFrontal->setup();

	Serial.println("Setup completo!");
}

unsigned long lastRead = 0;

int cur_sensor = 1;
int ldr_read_count = 0;
int ldr_medium[10];
int ldr_medium_idx = 0;

void loop() {
	if (!mqttClient.connected()){
		mqttConnect();
	}
	mqttClient.loop();

	// This call fetches all the gamepad info from the NINA (ESP32) module.
	// Just call this function in your main loop.
	// The gamepads pointer (the ones received in the callbacks) gets updated
	// automatically.
	BP32.update();

	int front_distance = SensorFrontal->loop();

	// It is safe to always do this before using the gamepad API.
	// This guarantees that the gamepad is valid and connected.
	for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
		GamepadPtr myGamepad = myGamepads[i];

		if (myGamepad && myGamepad->isConnected()) {
			// There are different ways to query whether a button is pressed.
			// By query each button individually:
			//	a(), b(), x(), y(), l1(), etc...
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
				// Some gamepads like the DS3, DualSense, Nintendo Wii, Nintendo Switch
				// support changing the "Player LEDs": those 4 LEDs that usually
				// indicate the "gamepad seat". It is possible to change them by
				// calling:
				myGamepad->setPlayerLEDs(led & 0x0f);
			}

			if (myGamepad->x()) {
				// Duration: 255 is ~2 seconds
				// force: intensity
				// Some gamepads like DS3, DS4, DualSense, Switch, Xbox One S support
				// rumble.
				// It is possible to set it by calling:
				myGamepad->setRumble(0xc0 /* force */, 0xc0 /* duration */);
			}
			dumpGamepad(myGamepad);

			int throttle = myGamepad->throttle();
			int brake = myGamepad->brake();

			if (throttle > 0) {
				if (front_distance < 30) {
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

	if (millis() - lastRead > READ_INTERVAL) {
		lastRead = millis();

		if (cur_sensor == 1) {
			int ldr = analogRead(ldr_pin);
			ldr_read_count++;
			Serial.print(ldr_read_count);
			Serial.print(") LDR: ");
			Serial.println(ldr);
			cur_sensor = 2;
			ldr_medium[ldr_medium_idx] = ldr;
			ldr_medium_idx++;
			ldr_medium_idx = ldr_medium_idx % 10;
		} else {
			float temp = readDHTTemp();
			Serial.print("Temperatura: ");
			Serial.println(temp);
			float humi = readDHTHumidity();
			Serial.print("Umidade: ");
			Serial.println(humi);
			cur_sensor = 1;

			mqttClient.publish("/Henrique/IoT/TF/DHT/Temperature", String(temp).c_str());
			mqttClient.publish("/Henrique/IoT/TF/DHT/Humidity", String(humi).c_str());
			if (ldr_read_count >= 10) {
				int ldr_read = 0;
				for (int i = 0; i < 10; i++) {
					ldr_read += ldr_medium[i];
				}
				ldr_read /= 10;
				mqttClient.publish("/Henrique/IoT/TF/LDR", String(ldr_read).c_str());
				if (ldr_read < 75) {
					led_estufa.on();
					mqttClient.publish("/Henrique/IoT/TF/LED_ESTUFA/Status", "1");
				}
			}
		}
	}
}
