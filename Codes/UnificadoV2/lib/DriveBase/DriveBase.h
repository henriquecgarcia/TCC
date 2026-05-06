#pragma once
#include <Arduino.h>
#include "../MotorDriver/MotorDriver.h"
#include "../Encoder/Encoder.h"
#include "../PID/PID.h"
#include "../../constants/RobotConfig.h"
#include "../../src/SharedTypes.h"

class DriveBase {
public:
	/**
	 * Cria a base de locomocao a partir dos motores e encoders.
	 * @param leftMotor Motor do lado esquerdo.
	 * @param rightMotor Motor do lado direito.
	 * @param leftEncoder Encoder do lado esquerdo.
	 * @param rightEncoder Encoder do lado direito.
	 * @return Nao retorna valor.
	 */
	DriveBase(MotorDriver& leftMotor, MotorDriver& rightMotor, Encoder& leftEncoder, Encoder& rightEncoder);
	/**
	 * Inicializa os controladores internos da base de locomocao.
	 * @return Nao retorna valor.
	 */
	void begin();
	/**
	 * Atualiza o controle de velocidade e orientacao da base.
	 * @param targetLeftMps Velocidade alvo da roda esquerda em m/s.
	 * @param targetRightMps Velocidade alvo da roda direita em m/s.
	 * @param dt Intervalo de tempo da atualizacao em segundos.
	 * @param imuYawRad Yaw atual do IMU em radianos.
	 * @return Amostra com os valores medidos e comandos aplicados.
	 */
	WheelSample update(float targetLeftMps, float targetRightMps, float dt, float imuYawRad);
	/**
	 * Para completamente a base de locomocao.
	 * @return Nao retorna valor.
	 */
	void stop();

private:
	MotorDriver& _leftMotor;
	MotorDriver& _rightMotor;
	Encoder& _leftEncoder;
	Encoder& _rightEncoder;
	PID _leftPid;
	PID _rightPid;
	PID _headingPid;
	float _headingReferenceRad;
	bool _headingReferenceValid;
	/**
	 * Converte pulsos de encoder em velocidade linear.
	 * @param ticks Quantidade de pulsos lidos.
	 * @param dt Intervalo de tempo em segundos.
	 * @return Velocidade estimada em metros por segundo.
	 */
	float speedFromTicks(long ticks, float dt) const;
};
