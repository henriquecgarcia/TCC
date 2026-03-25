#include <Constants.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#define DEBUG_PRINTS

#pragma region Funções Auxiliares

static double clamp(double value, double min, double max) {
	return (value < min) ? min : (value > max) ? max : value;
}

#include "Encoder.h"
#include "PID.h"

#pragma region Motor e Controle de Velocidade

enum MotorDirection {
	FORWARD,
	BACKWARD,
	STOPPED
};

class Motor {
private:
	unsigned long lastReading = 0; // para evitar leituras excessivas
	unsigned long lastDebug = 0;
	unsigned long lastThink = 0;
	const uint8_t in1Pin, in2Pin, pwmPin;
	Encoder* encoder;

	double targetRadS = rpmToRadS(100.0); // alvo em rad/s para controle
	double currentRPM = 0.0;
	double pidOutput = 0.0;

	MotorDirection direction = STOPPED;

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
		direction = FORWARD;

		targetRadS = rpmToRadS(100.0);
	}
	void backward() {
		encoder->reset();
		stop();

		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, HIGH);
		direction = BACKWARD;

		targetRadS = rpmToRadS(100.0);
	}
	void stop() {
		digitalWrite(in1Pin, LOW);
		digitalWrite(in2Pin, LOW);
		encoder->reset();

		direction = STOPPED;
		targetRadS = 0.0;
		pidRPM.reset();
		currentRPM = 0.0;
		pidOutput = 0.0;
		analogWrite(pwmPin, 0);
		lastReading = millis();
	}

	void update() {
		unsigned long now = millis();
		if (now - lastThink < thinkInterval) return;
		lastThink = now;

		currentRPM = encoder->getRPM(10, thinkInterval/1000.0);

		// controle de RPM com compensação de giro (se houver)
		float currentRadS = rpmToRadS(currentRPM);
		float rpmControl  = pidRPM.compute(targetRadS, currentRadS);
		rpmControl = pidRPM.scaleToPWM(rpmControl); // converte para valor de PWM

		pidOutput = rpmControl;
		pidOutput = clamp(pidOutput, 0.0, 255.0);

		analogWrite(pwmPin, int(pidOutput));
		encoder->reset();
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
		return targetRadS * (30.0 / M_PI); // converte de volta para RPM
	}
	double getTargetRadS() const {
		return targetRadS;
	}
	double getPIDOutput() const {
		return pidOutput;
	}
	MotorDirection getDirection() const {
		return direction;
	}
};

#pragma region Ponte H e controle

class PonteH {
private:
	Motor* motorRight;
	Motor* motorLeft;

	unsigned long lastUpdate = 0;

	static const unsigned long controlInterval = 100; // ms entre controles
public:
	PonteH(Motor* right, Motor* left) : motorRight(right), motorLeft(left) {}

	void setup() {
		if (!motorRight || !motorLeft) {
			return;
		}
		motorRight->begin();
		motorLeft->begin();
	}

	void forward() {
		if (!motorRight || !motorLeft) return;
		stop();
		motorRight->forward();
		motorLeft->forward();
	}

	void backward() {
		if (!motorRight || !motorLeft) return;
		stop();
		motorRight->backward();
		motorLeft->backward();
	}

	void turnLeft() {
		if (!motorRight || !motorLeft) return;
		stop();
		motorRight->backward();
		motorLeft->forward();

		motorRight->setTargetRPM(75.0);
		motorLeft->setTargetRPM(75.0);
	}

	void turnRight() {
		if (!motorRight || !motorLeft) return;
		stop();
		motorRight->forward();
		motorLeft->backward();

		motorRight->setTargetRPM(75.0);
		motorLeft->setTargetRPM(75.0);
	}

	void stop() {
		if (!motorRight || !motorLeft) return;
		motorRight->stop();
		motorLeft->stop();
	}

	void processReceivedCommand(const MotorCommand& cmd) {
		if (!motorRight || !motorLeft) return;
		int16_t targetRpmDir = cmd.targetRpmDir;
		int16_t targetRpmEsq = cmd.targetRpmEsq;

		bool shouldMove = (targetRpmDir != 0) || (targetRpmEsq != 0);
		if (!shouldMove) {
			stop();
			return;
		}

		// Armazenando para evitar altarnar entre forward/backward desnecessariamente
		MotorDirection dirDir = motorRight->getDirection();
		MotorDirection dirEsq = motorLeft->getDirection();

		if (targetRpmDir > 0) {
			motorRight->setTargetRPM(targetRpmDir);
			if (dirDir != FORWARD) {
				motorRight->forward();
			}
		} else if (targetRpmDir < 0) {
			motorRight->setTargetRPM(-targetRpmDir);
			if (dirDir != BACKWARD) {
				motorRight->backward();
			}
		} else {
			motorRight->stop();
		}

		if (targetRpmEsq > 0) {
			motorLeft->setTargetRPM(targetRpmEsq);
			if (dirEsq != FORWARD) {
				motorLeft->forward();
			}
		} else if (targetRpmEsq < 0) {
			motorLeft->setTargetRPM(-targetRpmEsq);
			if (dirEsq != BACKWARD) {
				motorLeft->backward();
			}
		} else {
			motorLeft->stop();
		}
	}

	void loop() {
		if (!motorRight || !motorLeft) {
			return;
		}

		unsigned long now = millis();
		if (now - lastUpdate < controlInterval) {
			return;
		}
		double deltaTime = (now - lastUpdate) / 1000.0;  // em segundos
		lastUpdate = now;

		motorRight->update();
		motorLeft->update();
	}
};

void onReceive(int numBytes) {
	String received = "";
	while (Wire.available()) {
		char c = Wire.read();
		received += c;
		Serial.print(c);
	}
	Serial.print("Received: ");
	Serial.println(received);
}
void onRequest() {
	Wire.write("Hello from ESP32!");
}

void setup() {
	Serial.begin(115200);
	Wire.begin((uint8_t) MOTOR_CONTROLER_ESP32_ADDR);
	Wire.onReceive(onReceive);
	Wire.onRequest(onRequest);
}

void loop() {
	// PonteH.loop() deve ser chamado aqui para atualizar o controle dos motores
}