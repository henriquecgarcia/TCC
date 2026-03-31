#include <Constants.h>
#include <Arduino.h>
#include <Wire.h>
#include <math.h>
#define DEBUG_PRINTS

const double intentKp = 1.0;
const double intentKi = 0.5;
const double intentKd = 0.0;

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
	unsigned long lastReading = 0;
	unsigned long lastDebug = 0;
	unsigned long lastThink = 0;

	const uint8_t in1Pin, in2Pin, pwmPin;
	const uint8_t ledcChannel;

	static const int ledcFreq = 20000;	 // 20 kHz
	static const int ledcResolution = 8;   // 8 bits
	static const int PWM_MAX = (1 << ledcResolution) - 1;

	Encoder* encoder;

	volatile double targetRadS = rpmToRadS(100.0);
	double currentRPM = 0.0;
	double pidOutput = 0.0;

	MotorDirection direction = STOPPED;
	PID pidRPM;

	static constexpr unsigned long thinkInterval = 500;

	double rpmToRadS(double rpm) {
		return rpm * (M_PI / 30.0f);
	}

	void applyDirection() {
		switch (direction) {
			case FORWARD:
				digitalWrite(in1Pin, HIGH);
				digitalWrite(in2Pin, LOW);
				break;
			case BACKWARD:
				digitalWrite(in1Pin, LOW);
				digitalWrite(in2Pin, HIGH);
				break;
			case STOPPED:
			default:
				digitalWrite(in1Pin, LOW);
				digitalWrite(in2Pin, LOW);
				break;
		}
	}

	void setPWM(int value) {
		value = clamp(value, 0, PWM_MAX);
		ledcWrite(ledcChannel, value);
	}

public:
	Motor(int in1, int in2, int pwm, uint8_t channel, Encoder* enc, float kp = 1.0, float ki = 5.0, float kd = 0.0)
		: in1Pin(in1), in2Pin(in2), pwmPin(pwm), ledcChannel(channel), encoder(enc), pidRPM(kp, ki, kd, thinkInterval / 1000.0f) {}

	void begin() {
		pinMode(in1Pin, OUTPUT);
		pinMode(in2Pin, OUTPUT);

		ledcSetup(ledcChannel, ledcFreq, ledcResolution);
		ledcAttachPin(pwmPin, ledcChannel);

		encoder->begin();
		encoder->reset();

		stop();

		pidRPM.setMaxMin(rpmToRadS(200.0), 0.0);
	}

	void setDirection(MotorDirection dir) {
		direction = dir;
		applyDirection();
	}

	void setTargetRadS(double radS) {
		targetRadS = clamp(radS, 0.0, rpmToRadS(200.0));
	}

	void setTargetRPM(double rpm) {
		return setTargetRadS(rpmToRadS(rpm));
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
		direction = STOPPED;
		applyDirection();

		targetRadS = 0.0;
		currentRPM = 0.0;
		pidOutput = 0.0;

		pidRPM.reset();
		setPWM(0);

		encoder->reset();
		lastReading = millis();
	}

	void update() {
		unsigned long now = millis();
		if (now - lastThink < thinkInterval) return;
		lastThink = now;

		currentRPM = encoder->getRPM(10, thinkInterval / 1000.0);

		float currentRadS = rpmToRadS(currentRPM);

		float control = pidRPM.compute(targetRadS, currentRadS);
		control = pidRPM.scaleToPWM(control);

		pidOutput = clamp(control, 0, PWM_MAX);

		setPWM((int)pidOutput);

		encoder->reset();
	}

	bool isMoving() {
		return (pidOutput > 1.0) && (targetRadS > 0.0);
	}

	void manualAddPID(int value) {
		pidOutput += value;
		pidOutput = clamp(pidOutput, 0, PWM_MAX);

		setPWM((int)pidOutput);

		#ifdef DEBUG_PRINTS
			Serial.print("[Motor "); Serial.print(pwmPin);
			Serial.print("] PID manual: "); Serial.println(value);
			Serial.print("Novo PWM: "); Serial.println(pidOutput);
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
		int16_t targetRadSDir = cmd.targetRadSRight;
		int16_t targetRadSEsq = cmd.targetRadSLeft;

		bool shouldMove = (targetRadSDir != 0) || (targetRadSEsq != 0);
		if (!shouldMove) {
			stop();
			return;
		}

		// Armazenando para evitar altarnar entre forward/backward desnecessariamente
		MotorDirection dirDir = motorRight->getDirection();
		MotorDirection dirEsq = motorLeft->getDirection();

		if (targetRadSDir > 0) {
			motorRight->setTargetRadS(targetRadSDir);
			if (dirDir != FORWARD) {
				motorRight->forward();
			}
		} else if (targetRadSDir < 0) {
			motorRight->setTargetRadS(-targetRadSDir);
			if (dirDir != BACKWARD) {
				motorRight->backward();
			}
		} else {
			motorRight->stop();
		}

		if (targetRadSEsq > 0) {
			motorLeft->setTargetRadS(targetRadSEsq);
			if (dirEsq != FORWARD) {
				motorLeft->forward();
			}
		} else if (targetRadSEsq < 0) {
			motorLeft->setTargetRadS(-targetRadSEsq);
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

// ——————— Encoders ———————
// Motor Direito (Motor A)
Encoder *encoderD = new Encoder(15, 4);  // CH A=15, CH B=4
// Motor Esquerdo (Motor B) — moved to avoid pin conflicts with IN pins
Encoder *encoderE = new Encoder(34, 35);  // CH A=34, CH B=35

// ——————— Motores ———————
// Mapeamento solicitado:
// 🔵 Motor A (Direito)
// IN1 -> GPIO18, IN2 -> GPIO19, PWM -> GPIO27 (LEDC channel 0)
// 🔴 Motor B (Esquerdo)
// IN3 -> GPIO16, IN4 -> GPIO17, PWM -> GPIO14 (LEDC channel 1)

Motor *motorDireito  = new Motor(18, 19, 27, 0, encoderD, intentKp, intentKi, intentKd );
Motor *motorEsquerdo = new Motor(16, 17, 14, 1, encoderE, intentKp, intentKi, intentKd );

PonteH ponteH(motorDireito, motorEsquerdo);

void onReceive(int numBytes) {
	// Verifica se recebeu exatamente o tamanho esperado
	if (numBytes != sizeof(MotorCommand)) {
		// Descarta bytes inválidos
		while (Wire.available()) Wire.read();
		return;
	}
	MotorCommand currentCommand;

	uint8_t buffer[sizeof(MotorCommand)];

	// Lê todos os bytes
	for (int i = 0; i < sizeof(MotorCommand); i++) {
		if (Wire.available()) {
			buffer[i] = Wire.read();
		}
	}

	memcpy((void*)&currentCommand, buffer, sizeof(MotorCommand));

	ponteH.processReceivedCommand(currentCommand);
}

volatile MotorStatus currentStatus;

void onRequest() {
	// Copia local para evitar inconsistência
	MotorStatus snapshot;

	noInterrupts();
	snapshot.radSLeft = (int16_t)motorEsquerdo->getTargetRadS();
	snapshot.radSRight = (int16_t)motorDireito->getTargetRadS();
	snapshot.pwmLeft = (int16_t)motorEsquerdo->getPIDOutput();
	snapshot.pwmRight = (int16_t)motorDireito->getPIDOutput();
	interrupts();

	// Envia como array de bytes
	Wire.write((uint8_t*)&snapshot, sizeof(MotorStatus));
}

void setup() {
	Serial.begin(115200);
	Wire.begin((uint8_t) MOTOR_CONTROLER_ESP32_ADDR);
	Wire.onReceive(onReceive);
	Wire.onRequest(onRequest);
}

void loop() {
	// PonteH.loop() deve ser chamado aqui para atualizar o controle dos motores
	ponteH.loop();
}