#pragma once
#include <Arduino.h>

class MotorDriver {
public:
	/**
	 * Cria um driver de motor com pinos de direcao e PWM.
	 * @param in1 Pino IN1.
	 * @param in2 Pino IN2.
	 * @param pwmPin Pino de saida PWM.
	 * @param pwmChannel Canal PWM utilizado.
	 * @return Nao retorna valor.
	 */
	MotorDriver(uint8_t in1, uint8_t in2, uint8_t pwmPin, uint8_t pwmChannel);
	/**
	 * Configura GPIO e PWM do driver.
	 * @param pwmFreq Frequencia do PWM em Hz.
	 * @param pwmResolutionBits Resolucao do PWM em bits.
	 * @return Nao retorna valor.
	 */
	void begin(uint16_t pwmFreq, uint8_t pwmResolutionBits);
	/**
	 * Aplica um comando PWM ao motor.
	 * @param pwm Valor de comando, com sinal indicando direcao.
	 * @return Nao retorna valor.
	 */
	void setPWM(int16_t pwm);
	/**
	 * Freia ativamente o motor.
	 * @return Nao retorna valor.
	 */
	void brake();
	/**
	 * Coloca o motor em estado de roda livre.
	 * @return Nao retorna valor.
	 */
	void coast();

private:
	uint8_t _in1;
	uint8_t _in2;
	uint8_t _pwmPin;
	uint8_t _pwmChannel;
	uint8_t _maxPwm;
};
