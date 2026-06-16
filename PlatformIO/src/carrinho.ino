#define DEBUG_PRINTS
struct SensorSnapshot;

#include <Wire.h>
#include <Arduino.h>
#include <math.h>
#include <ArduinoJson.h>
#include "RobotConfig.h"

#include <WiFi.h>
#include <SPIFFS.h>

#include "LED.h"
#include "MediaMovel.h"
#include "Map.h"
#include "PoseEKF.h"
#include "BrownoutLogger.h"
#include "ExperimentLogger.h"


const char* ssid = "Canguru";
const char* password = "VamoPula";
const double intentKp = 1.0;
const double intentKi = 0.5;
const double intentKd = 0.0;

static const unsigned int GRID_WIDTH = 7;   // Mapa binário 7x7 para os experimentos do TCC.
static const unsigned int GRID_HEIGHT = 7;  // Bordas ocupadas (1) e área interna livre (0).
static const int ENCODER_TEETH = 10;

static constexpr float GRID_CELL_SIZE_M = 0.30f; // Cada célula do TCC representa 30 cm x 30 cm no mundo real.
static constexpr float GRID_CELL_SIZE_CM = GRID_CELL_SIZE_M * 100.0f;

static constexpr float CELL_TARGET_TOLERANCE_M = 0.015f; // Tolerância de 1,5 cm para encerrar o avanço da célula.
static constexpr float CELL_HEADING_TOLERANCE_RAD = 2.0f * (M_PI / 180.0f); // Erro angular aceitável após cada célula.

static constexpr double TURN_ENCODER_SCALE = 0.50;

static constexpr double TURN_MAX_RAW_DELTA_DEG_PER_SEC = 600.0;
static constexpr double TURN_MAX_RAW_DELTA_DEG_FLOOR = 35.0;
static constexpr double TURN_MAX_REVERSE_DELTA_DEG = 8.0;

static constexpr double QMC_HEADING_MAX_INNOVATION_DEG = 25.0; // QMC só corrige frente/ré se não der salto absurdo.

static constexpr double EKF_MAX_ENCODER_DELTA_TICKS_PER_LOOP = 18.0; // trava pulsos impossíveis antes do EKF.

static constexpr double GYRO_Z_LEFT_TURN_SIGN = -1.0; // QMC5883L montado com eixo Z invertido para curvas à esquerda.
static constexpr double GYRO_Z_RIGHT_TURN_SIGN = 1.0; // Sinal usado para transformar giro à direita em progresso angular positivo.

static constexpr float ODOM_WHEEL_RADIUS_M = 0.0325f; // Mesmo raio usado no PoseEKF, necessário para medir cada célula pelos encoders.
static constexpr float ODOM_TICKS_PER_REV = 20.0f; // Mesmo valor usado no PoseEKF para converter ticks em distância.
static constexpr float ODOM_WHEEL_BASE_M = 0.1400f; // Distância entre rodas usada para estimar giro por encoder.

static constexpr float CELL_OVERRUN_ABORT_FACTOR = 1.80f; // Uma célula acima de 180% do alvo é considerada falha de odometria/controle.
static constexpr int FRONT_OBSTACLE_REPLAN_MM = 220; // Distância frontal usada para marcar obstáculo e replanejar.
static constexpr unsigned long CELL_MAX_TIME_MS = 6500; // Timeout de segurança por célula de 30 cm.

static const size_t PATH_BUFFER_SIZE = 64;
static const unsigned int MAP_VIEW_WIDTH = GRID_WIDTH;
static const unsigned int MAP_VIEW_HEIGHT = GRID_HEIGHT;

#include "WebInterface.h"

#pragma region Funções Auxiliares

static double clamp(double value, double min, double max) {
	return (value < min) ? min : (value > max) ? max : value;
}

static float normalizeSignedAngleGlobal(float angleRad) {
	while (angleRad > M_PI) angleRad -= 2.0f * M_PI;
	while (angleRad < -M_PI) angleRad += 2.0f * M_PI;
	return angleRad;
}

static void webLog(const String& msg) {
	#ifdef DEBUG_PRINTS
		Serial.print(msg);
	#endif
	WebInterface::getInstance().log(msg);
}

static void webLog(const char* msg) {
	#ifdef DEBUG_PRINTS
		Serial.print(msg);
	#endif
	WebInterface::getInstance().log(msg);
}

void sendTelemetryFrame();
String getMapSnapshotJson();
int getCachedFrontDistanceMm();
float getCachedGyroZ();
float getCachedMagAngleZ();
float getCachedMagRateZ();
unsigned long getCachedQmcAgeMs();
unsigned long getCachedMagAgeMs();
unsigned long getCachedTofAgeMs();

/**
 * Retorna true quando existe algum comando físico ativo nos motores.
 *
 * Esta função é usada para impedir que a odometria/EKF continue integrando
 * ruído quando o robô está parado. Sem essa trava, pequenos ruídos dos
 * encoders ou do QMC5883L podem fazer a localização discreta "andar sozinha"
 * no mapa enviado ao WebServer.
 */
bool isRobotPhysicallyCommanded();
long getSignedLeftTicks();
long getSignedRightTicks();
long getRawLeftEncoderTicks();
long getRawRightEncoderTicks();
void updateSignedEncoderTicks();
void syncEncoderReferenceForEKF();

#include "VL53L0X.h"
#include "QMC5883L_Custom.h"

#include "Encoder.h"
#include "PID.h"
#include "Motor.h"

#pragma region "Ponte H e controle"

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
	unsigned long lastUpdate   = 0;
	unsigned long lastSocketUpdate = 0;
	static const unsigned long controlInterval = 100; // ms entre controles
	unsigned long turnStartedAt = 0;
	unsigned long turnTimeoutMs = 4000;

	static constexpr double turnToleranceRad = 2.0 * (M_PI / 180.0);      // alvo: erro < 2 graus
	static constexpr double turnFineThresholdRad = 15.0 * (M_PI / 180.0); // troca coarse -> fine

	// flag para alternar quais motores atualizar
	double targetHeading = 0.0;
	double turnTargetRad = 0.0;
	int turnDirectionSign = 0; // +1 = esquerda, -1 = direita
	long turnStartLeftRawTicks = 0;
	long turnStartRightRawTicks = 0;
	double turnFilteredProgressRad = 0.0;
	double turnLastRawProgressRad = 0.0;
	unsigned int turnRejectedSpikeCount = 0;

	PID pidGyro = PID(0.1, 0.0, 0.0, controlInterval / 1000.0); // Kp, Ki, Kd para magnetômetro
	PID pidTurn = PID(0.0, 0.0, 0.0, controlInterval / 1000.0); // mantido por compatibilidade; giro agora fecha por encoder

	PID pidCellHeading = PID(2.4, 0.0, 0.18, controlInterval / 1000.0); // correção angular contínua por célula

	static double normalizeAngle(double angle) {
		angle = fmod(angle, 2 * M_PI);
		if (angle < 0.0) {
			angle += 2 * M_PI;
		}
		return angle;
	}

	static double ticksToDistanceAbs(long ticks) {
		return fabs((double)ticks) / (double)ODOM_TICKS_PER_REV * (2.0 * M_PI * (double)ODOM_WHEEL_RADIUS_M);
	}

		double getRawEncoderTurnProgressRad() const {
			if (!motorRight || !motorLeft || turnDirectionSign == 0) return 0.0;

			const long leftDelta = getRawLeftEncoderTicks() - turnStartLeftRawTicks;
			const long rightDelta = getRawRightEncoderTicks() - turnStartRightRawTicks;

			const double leftDist = ticksToDistanceAbs(leftDelta);
			const double rightDist = ticksToDistanceAbs(rightDelta);
			const double avgWheelArc = (leftDist + rightDist) * 0.5;

			return turnDirectionSign * ((2.0 * avgWheelArc) / (double)ODOM_WHEEL_BASE_M);
		}

		double getEncoderTurnProgressRad() const {
			return getRawEncoderTurnProgressRad() * TURN_ENCODER_SCALE;
		}

		double updateFilteredEncoderTurnProgressRad(double rawProgressRad, double deltaTime) {
			if (turnDirectionSign == 0) return 0.0;

			double deltaRaw = rawProgressRad - turnLastRawProgressRad;
			turnLastRawProgressRad = rawProgressRad;

			const double dt = deltaTime > 0.001 ? deltaTime : (controlInterval / 1000.0);
			const double maxStepDeg = max(TURN_MAX_RAW_DELTA_DEG_FLOOR, TURN_MAX_RAW_DELTA_DEG_PER_SEC * dt);
			const double deltaRawDeg = deltaRaw * (180.0 / M_PI);

			// Se o delta veio contra o sentido comandado, quase sempre é ruído/rollback de encoder.
			if ((deltaRaw * turnDirectionSign) < -(TURN_MAX_REVERSE_DELTA_DEG * M_PI / 180.0)) {
				turnRejectedSpikeCount++;
				webLog("[PonteH] Pulso reverso de encoder ignorado durante giro: delta_raw=" + String(deltaRawDeg, 2) + " deg\n");
				return turnFilteredProgressRad;
			}

			// Rejeita pulos fisicamente impossíveis. Sem isso, um salto bruto de centenas de graus
			// encerra uma curva de 90° antes de o robô realmente virar.
			if (fabs(deltaRawDeg) > maxStepDeg) {
				turnRejectedSpikeCount++;
				webLog("[PonteH] Spike de encoder ignorado durante giro: delta_raw=" + String(deltaRawDeg, 2) +
					" deg | limite=" + String(maxStepDeg, 2) + " deg | raw_total=" + String(rawProgressRad * 180.0 / M_PI, 2) + " deg\n");
				return turnFilteredProgressRad;
			}

			turnFilteredProgressRad += deltaRaw * TURN_ENCODER_SCALE;
			return turnFilteredProgressRad;
		}

public:
	PonteH(Motor* right, Motor* left) : motorRight(right), motorLeft(left) {
		pidGyro.setTunning(0.1, 0.0, 0.0); // Kp, Ki, Kd
		pidGyro.setMaxMin(0.5, -0.5); // limites de correção
		pidTurn.setMaxMin(1.5, -1.5); // limites de correcao na fase fina
		pidCellHeading.setMaxMin(1.2, -1.2); // limites de correção contínua durante o avanço de célula
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
		lastUpdate = millis();
		pidGyro.reset();
		pidTurn.reset();
		pidCellHeading.reset();
		webLog("[PonteH] Configuração completa!\n");
	}

	void forward() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_FORWARD) {
			stop();
			motorRight->forward();
			motorLeft->forward();
			syncEncoderReferenceForEKF();
			currentMove = MOVEMENT_FORWARD;
			isMoving = true;
			lastUpdate = millis();
			if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll("{\"status\": \"moving\", \"direction\": \"forward\"}");
		}
	}

	void backward() {
		if (!motorRight || !motorLeft) return;
		if (!isMoving || currentMove != MOVEMENT_BACKWARDS) {
			stop();
			currentMove = MOVEMENT_BACKWARDS;
			motorRight->backward();
			motorLeft->backward();
			syncEncoderReferenceForEKF();
			isMoving = true;
			lastUpdate = millis();
			if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll("{\"status\": \"moving\", \"direction\": \"backward\"}");
		}
	}

	void turnLeft(double degs = 90.0) {
		if (!motorRight || !motorLeft) return;
		degs = fabs(degs);
		if (degs > 180.0)
			return turnRight(360.0 - degs); // "vire 270* para esquerda" = "vire 90* para direita"

		if (!isMoving || currentMove != MOVEMENT_TURN_LEFT) {
			stop();

			// Giro fechado por encoder: o QMC5883L é magnetômetro e fica muito ruidoso
			// perto do L298N/motores. Por isso ele não é usado para decidir quando parar a curva.
			motorRight->backward();
			motorLeft->forward();
			motorRight->setTargetRPM(115.0);
			motorLeft->setTargetRPM(115.0);
			syncEncoderReferenceForEKF();

			currentMove = MOVEMENT_TURN_LEFT;
			isMoving = true;
			lastUpdate = millis();

			if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll("{\"status\": \"moving\", \"direction\": \"left\"}");

			turnTargetRad = degs * (M_PI / 180.0);
			turnDirectionSign = +1;
			turnStartLeftRawTicks = getRawLeftEncoderTicks();
			turnStartRightRawTicks = getRawRightEncoderTicks();
			turnFilteredProgressRad = 0.0;
			turnLastRawProgressRad = 0.0;
			turnRejectedSpikeCount = 0;

			// Mantém targetHeading somente para telemetria/referência, não para controle.
			float currentAngleZ = getCachedMagAngleZ();
			targetHeading = normalizeAngle(currentAngleZ + turnTargetRad);

			pidTurn.reset();
			turnStartedAt = millis();
			turnTimeoutMs = static_cast<unsigned long>(10000.0 + (degs * 90.0));
			webLog("[PonteH] Encoder turn calibrated: direction=left target=" + String(degs) + " deg (" + String(turnTargetRad) + " radians) timeout=" + String(turnTimeoutMs) + " ms\n");
		}
	}

	void turnRight(double degs = 90.0) {
		if (!motorRight || !motorLeft) return;
		degs = fabs(degs);
		if (degs > 180.0)
			return turnLeft(360.0 - degs); // "vire 270* para direita" = "vire 90* para esquerda"

		if (!isMoving || currentMove != MOVEMENT_TURN_RIGHT) {
			stop();

			// Giro fechado por encoder: o QMC5883L é magnetômetro e fica muito ruidoso
			// perto do L298N/motores. Por isso ele não é usado para decidir quando parar a curva.
			motorRight->forward();
			motorLeft->backward();
			motorRight->setTargetRPM(115.0);
			motorLeft->setTargetRPM(115.0);
			syncEncoderReferenceForEKF();

			currentMove = MOVEMENT_TURN_RIGHT;
			isMoving = true;
			lastUpdate = millis();

			if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll("{\"status\": \"moving\", \"direction\": \"right\"}");

			turnTargetRad = degs * (M_PI / 180.0);
			turnDirectionSign = -1;
			turnStartLeftRawTicks = getRawLeftEncoderTicks();
			turnStartRightRawTicks = getRawRightEncoderTicks();
			turnFilteredProgressRad = 0.0;
			turnLastRawProgressRad = 0.0;
			turnRejectedSpikeCount = 0;

			// Mantém targetHeading somente para telemetria/referência, não para controle.
			float currentAngleZ = getCachedMagAngleZ();
			targetHeading = normalizeAngle(currentAngleZ - turnTargetRad);

			pidTurn.reset();
			turnStartedAt = millis();
			turnTimeoutMs = static_cast<unsigned long>(10000.0 + (degs * 90.0));
			webLog("[PonteH] Encoder turn calibrated: direction=right target=" + String(degs) + " deg (" + String(turnTargetRad) + " radians) timeout=" + String(turnTimeoutMs) + " ms\n");
		}
	}

	void stop() {
		if (!motorRight || !motorLeft) return;
		motorRight->stop();
		motorLeft->stop();

		pidGyro.reset();
		pidCellHeading.reset();
		// pidTurn.reset();
		targetHeading = 0.0;
		turnTargetRad = 0.0;
		turnDirectionSign = 0;
		turnStartLeftRawTicks = getRawLeftEncoderTicks();
		turnStartRightRawTicks = getRawRightEncoderTicks();
		turnFilteredProgressRad = 0.0;
		turnLastRawProgressRad = 0.0;
		turnRejectedSpikeCount = 0;

		isMoving = false;
		currentMove = MOVEMENT_STOPPED;
		syncEncoderReferenceForEKF();
	}

	bool isStopped() const {
		return !isMoving || (currentMove == MOVEMENT_STOPPED); // Vai que eu esqueci de setar como parado em uma das duas, ai... Agora ta seguro :)
	}

	// Deve ser chamado dentro de loop()
	void loop(int frontDistanceMm, float _gyroZRadS) {
		if (!isMoving || !motorRight || !motorLeft) {
			return;
		}

		unsigned long now = millis();
		if (now - lastUpdate < controlInterval) {
			return;
		}
		double deltaTime = (now - lastUpdate) / 1000.0;  // em segundos
		lastUpdate = now;

		float gyroRead = getCachedMagAngleZ();
		switch (currentMove) {
			case MOVEMENT_FORWARD: {
				int d = frontDistanceMm;
				if (d > 0 && d < 100) {
					stop();
					#ifdef DEBUG_PRINTS
						Serial.println("Parando por obstáculo!");
					#endif
					webLog("[PonteH] Parando por obstáculo!\n");
					if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll("{\"movement\": \"stopped\", \"reason\": \"obstacle\"}");
					break;
				}
				// Como é a mesma lógica para controle de "Frente" e "Trás", não colocamos BREAK, caindo direto para o proximo
			}
			case MOVEMENT_BACKWARDS: {
				double correction = pidGyro.compute(targetHeading, gyroRead);
				motorRight->update(correction);
				motorLeft->update(-correction);
				break;
			}
			case MOVEMENT_TURN_LEFT:
			case MOVEMENT_TURN_RIGHT: {
				// Controle de curva por encoder diferencial.
				// O QMC5883L sofre saltos fortes perto dos motores/L298N; usar o heading
				// magnético como condição de parada fazia o robô girar sem convergir.
				const double rawEncoderTurnRad = getRawEncoderTurnProgressRad();
				const double rawEncoderTurnDeg = rawEncoderTurnRad * (180.0 / M_PI);
				const double encoderTurnRad = updateFilteredEncoderTurnProgressRad(rawEncoderTurnRad, deltaTime);
				const double encoderTurnDeg = encoderTurnRad * (180.0 / M_PI);
				const double magHeadingDeg = gyroRead * (180.0 / M_PI);

				const double absProgress = fabs(encoderTurnRad);
				double remainingRad = turnTargetRad - absProgress;
				if (remainingRad < 0.0) remainingRad = 0.0;
				const double remainingDeg = remainingRad * (180.0 / M_PI);
				const bool isFinePhase = (remainingRad <= turnFineThresholdRad);

				// Reduz a velocidade perto do alvo para diminuir overshoot mecânico.
				double targetRpm = isFinePhase ? 55.0 : 105.0;

				motorRight->setTargetRPM(targetRpm);
				motorLeft->setTargetRPM(targetRpm);

				#ifdef DEBUG_PRINTS
					Serial.print("[PonteH] Encoder Turn Z scaled: ");
					Serial.print(encoderTurnRad);
					Serial.print(" rad (");
					Serial.print(encoderTurnDeg);
					Serial.printf(" deg) on deltaTime: %f\n", deltaTime);
					Serial.print("Raw encoder turn before scale (deg): ");
					Serial.println(rawEncoderTurnDeg);
					Serial.print("Turn encoder scale: ");
					Serial.println(TURN_ENCODER_SCALE);
					Serial.print("Remaining angle (deg): ");
					Serial.println(remainingDeg);
					Serial.print("QMC heading ref (deg): ");
					Serial.println(magHeadingDeg);
					Serial.print("Turn phase: ");
					Serial.println(isFinePhase ? "fine" : "coarse");
					Serial.print("Target RPM: ");
					Serial.println(targetRpm);
				#endif

				webLog("[PonteH] Encoder Turn Z scaled: " + String(encoderTurnRad) +
					" rad (" + String(encoderTurnDeg) + " deg)" +
					" | raw: " + String(rawEncoderTurnDeg) + " deg" +
					" | scale: " + String(TURN_ENCODER_SCALE) +
					" | rejected_spikes: " + String(turnRejectedSpikeCount) +
					" | deltaTime: " + String(deltaTime) +
					"\n[PonteH] Remaining angle: " + String(remainingDeg) + " deg" +
					" | qmc_heading: " + String(magHeadingDeg) + " deg" +
					" | phase: " + String(isFinePhase ? "fine" : "coarse") +
					" | target_rpm: " + String(targetRpm) + "\n");

				if (auto cs = WebInterface::getInstance().getCarSocket()) {
					cs->textAll("{\"turning_angleZ\": " + String(encoderTurnRad) +
						", \"turningAngleZ_deg\": " + String(encoderTurnDeg) +
						", \"turn_raw_deg\": " + String(rawEncoderTurnDeg) +
						", \"turn_encoder_scale\": " + String(TURN_ENCODER_SCALE) +
						", \"turn_remaining_deg\": " + String(remainingDeg) +
						", \"qmc_heading_deg\": " + String(magHeadingDeg) +
						", \"delta_time\": " + String(deltaTime) + "}");
				}

				if (remainingRad <= turnToleranceRad) {
					stop();
					webLog("[PonteH] Curva finalizada por encoder. Alvo angular atingido.\n");
					if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll("{\"movement\": \"stopped\", \"reason\": \"encoder target angle reached\"}");
					return;
				}

				if (now - turnStartedAt > turnTimeoutMs) {
					stop();
					webLog("[PonteH] Curva interrompida por timeout de seguranca. Verifique encoder/roda patinando.\n");
					if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll("{\"movement\": \"stopped\", \"reason\": \"turn timeout\"}");
					return;
				}

				// Direção física já foi definida em turnLeft()/turnRight().
				// Aqui apenas fechamos velocidade de cada roda, sem usar QMC como erro.
				motorRight->update(0.0);
				motorLeft->update(0.0);
				break;
			}
			default:
				break;
		}

		if (auto cs = WebInterface::getInstance().getCarSocket()) {
			if (cs->count() > 0) {
			if (isMoving && now - lastSocketUpdate >= 100) {
				lastSocketUpdate = now;
				sendTelemetryFrame();
			}
			}
		} else {
			#ifdef DEBUG_PRINTS
				Serial.println("[PonteH] Nenhum cliente conectado, não enviando dados.");
			#endif
		}
	}

	Movement getCurrentMove() const {
		return currentMove;
	}
};


/**
 * Controla o deslocamento discreto do robô em células de 30 cm x 30 cm.
 *
 * A classe também executa caminhos planejados pelo A*, registra dados
 * experimentais em CSV e trata obstáculos dinâmicos detectados pelo VL53L0X.
 * O loop é totalmente não bloqueante para preservar WebSocket, sensores e EKF.
 */
class GridCellNavigator {
private:
	/** Estados internos da navegação célula-a-célula e da execução de caminho. */
	enum State {
		STATE_IDLE,
		STATE_ALIGN_TO_PATH_STEP,
		STATE_WAIT_PATH_TURN,
		STATE_START_CELL,
		STATE_MOVING_CELL,
		STATE_SETTLE_AFTER_CELL,
		STATE_CORRECT_HEADING,
		STATE_WAIT_TURN,
		STATE_FINISHED,
		STATE_ABORTED
	};

	PonteH* ponte = nullptr;
	PoseEKF* pose = nullptr;
	Map* map = nullptr;
	VL53L0X* frontSensor = nullptr;
	ExperimentLogger* logger = nullptr;
	Encoder* leftEncoder = nullptr;
	Encoder* rightEncoder = nullptr;

	State state = STATE_IDLE;
	int requestedCells = 0;
	int completedCells = 0;
	float cellStartX = 0.0f;
	float cellStartY = 0.0f;
	long cellStartLeftTicks = 0;
	long cellStartRightTicks = 0;
	float targetHeadingRad = 0.0f;
	unsigned long stateStartedAt = 0;
	String abortReason = "";
	bool lastObstacleDetected = false;
	int lastFrontDistanceMm = -1;
	bool lastReplanTriggered = false;

	bool pathMode = false;
	Map::Position path[PATH_BUFFER_SIZE];
	size_t pathLen = 0;
	size_t pathIndex = 0;
	Map::Position finalTarget = {0, 0};
	Map::Position cellStartGrid = {MAP_ORIGIN_X, MAP_ORIGIN_Y};
	Map::Position expectedNextGrid = {MAP_ORIGIN_X, MAP_ORIGIN_Y};
	bool currentCellReverse = false; // Quando true, a próxima célula será executada dando ré, sem girar 180 graus.

	/** Normaliza um ângulo para o intervalo [-PI, PI]. */
	static float normalizeSignedAngle(float angleRad) {
		while (angleRad > M_PI) angleRad -= 2.0f * M_PI;
		while (angleRad < -M_PI) angleRad += 2.0f * M_PI;
		return angleRad;
	}

	/** Retorna o nome textual do estado atual para telemetria e debug. */
	const char* getStateName() const {
		switch (state) {
			case STATE_IDLE: return "idle";
			case STATE_ALIGN_TO_PATH_STEP: return "align_to_path_step";
			case STATE_WAIT_PATH_TURN: return "wait_path_turn";
			case STATE_START_CELL: return "start_cell";
			case STATE_MOVING_CELL: return "moving_cell";
			case STATE_SETTLE_AFTER_CELL: return "settle_after_cell";
			case STATE_CORRECT_HEADING: return "correct_heading";
			case STATE_WAIT_TURN: return "wait_turn";
			case STATE_FINISHED: return "finished";
			case STATE_ABORTED: return "aborted";
			default: return "unknown";
		}
	}

	/** Converte delta absoluto de ticks em distância linear usando a calibração do EKF. */
	float ticksToCellDistance(long tickDelta) const {
		return ((float)labs(tickDelta) / ODOM_TICKS_PER_REV) * (2.0f * M_PI * ODOM_WHEEL_RADIUS_M);
	}

	/**
	 * Calcula a distância percorrida desde o início da célula atual.
	 *
	 * A versão anterior dependia apenas da pose contínua do EKF. Quando os
	 * encoders eram resetados ao iniciar um motor, a referência podia carregar
	 * ticks antigos e gerar uma primeira célula absurda, como 1,226 m para alvo
	 * de 0,30 m. Agora a distância da célula vem diretamente do delta assinado
	 * dos encoders desde o início desta célula.
	 */
	float getDistanceFromCellStart() const {
		const float leftDist = ticksToCellDistance(getSignedLeftTicks() - cellStartLeftTicks);
		const float rightDist = ticksToCellDistance(getSignedRightTicks() - cellStartRightTicks);
		return (leftDist + rightDist) * 0.5f;
	}

	/** Atualiza a célula discreta do mapa usando a pose contínua do EKF. */
	void updateMapCellFromPose() {
		if (!map || !pose) return;
		int mapX = MAP_ORIGIN_X + (int)roundf(pose->getX() / GRID_CELL_SIZE_M);
		int mapY = MAP_ORIGIN_Y + (int)roundf(pose->getY() / GRID_CELL_SIZE_M);
		mapX = (int)clamp(mapX, 0, map->getWidth() - 1);
		mapY = (int)clamp(mapY, 0, map->getHeight() - 1);
		map->setPosition((unsigned int)mapX, (unsigned int)mapY);
	}

	/** Converte um deslocamento de célula em heading contínuo do robô. */
	float headingForStep(const Map::Position& from, const Map::Position& to) const {
		const int dx = (int)to.x - (int)from.x;
		const int dy = (int)to.y - (int)from.y;
		if (dx > 0) return 0.0f;
		if (dx < 0) return M_PI;
		if (dy > 0) return M_PI / 2.0f;
		return -M_PI / 2.0f;
	}

	/** Retorna a diferença absoluta entre dois ângulos, sempre no intervalo [0, PI]. */
	float absoluteHeadingDelta(float a, float b) const {
		return fabsf(normalizeSignedAngle(a - b));
	}

	/**
	 * Decide se o próximo passo do A* deve ser feito dando ré.
	 *
	 * Quando a célula desejada está atrás do robô, girar 180 graus costuma ser
	 * instável em robôs pequenos com encoder/QMC5883L. Nessa situação, o robô
	 * mantém o heading atual e desloca uma célula para trás. Para passos laterais
	 * ou frontais, o comportamento antigo é preservado: gira para alinhar e anda.
	 */
	bool shouldReverseForStep(const Map::Position& from, const Map::Position& to) const {
		const float stepHeading = headingForStep(from, to);
		const float headingError = absoluteHeadingDelta(pose->getTheta(), stepHeading);
		return headingError > (3.0f * M_PI / 4.0f);
	}

	/** Retorna a célula diretamente à frente com base no heading atual do EKF. */
	bool getAheadCell(Map::Position& out) const {
		if (!map || !pose) return false;
		const Map::Position current = map->getPosition();
		const float theta = normalizeSignedAngle(pose->getTheta());
		int dx = 0;
		int dy = 0;

		if (theta >= -M_PI / 4.0f && theta < M_PI / 4.0f) dx = 1;
		else if (theta >= M_PI / 4.0f && theta < 3.0f * M_PI / 4.0f) dy = 1;
		else if (theta < -M_PI / 4.0f && theta >= -3.0f * M_PI / 4.0f) dy = -1;
		else dx = -1;

		const int ax = (int)current.x + dx;
		const int ay = (int)current.y + dy;
		if (ax < 0 || ay < 0 || ax >= (int)map->getWidth() || ay >= (int)map->getHeight()) return false;
		out.x = (unsigned int)ax;
		out.y = (unsigned int)ay;
		return true;
	}

	/** Retorna a célula diretamente atrás do robô com base no heading atual do EKF. */
	bool getBehindCell(Map::Position& out) const {
		if (!map || !pose) return false;
		const Map::Position current = map->getPosition();
		const float theta = normalizeSignedAngle(pose->getTheta());
		int dx = 0;
		int dy = 0;

		if (theta >= -M_PI / 4.0f && theta < M_PI / 4.0f) dx = -1;
		else if (theta >= M_PI / 4.0f && theta < 3.0f * M_PI / 4.0f) dy = -1;
		else if (theta < -M_PI / 4.0f && theta >= -3.0f * M_PI / 4.0f) dy = 1;
		else dx = 1;

		const int bx = (int)current.x + dx;
		const int by = (int)current.y + dy;
		if (bx < 0 || by < 0 || bx >= (int)map->getWidth() || by >= (int)map->getHeight()) return false;
		out.x = (unsigned int)bx;
		out.y = (unsigned int)by;
		return true;
	}


	/** Converte uma célula discreta do mapa para a coordenada contínua X em metros. */
	float gridToWorldX(const Map::Position& grid) const {
		return ((int)grid.x - MAP_ORIGIN_X) * GRID_CELL_SIZE_M;
	}

	/** Converte uma célula discreta do mapa para a coordenada contínua Y em metros. */
	float gridToWorldY(const Map::Position& grid) const {
		return ((int)grid.y - MAP_ORIGIN_Y) * GRID_CELL_SIZE_M;
	}

	/**
	 * Calcula a próxima célula esperada a partir de um heading discreto.
	 *
	 * O cálculo arredonda o heading para um dos quatro eixos do grid. Isso evita
	 * que pequenos erros do QMC5883L criem deslocamentos diagonais ou façam o robô
	 * "pular" célula na localização enviada ao WebServer.
	 */
	bool getNextGridFromHeading(const Map::Position& current, float headingRad, Map::Position& out) const {
		const float theta = normalizeSignedAngle(headingRad);
		int dx = 0;
		int dy = 0;

		if (theta >= -M_PI / 4.0f && theta < M_PI / 4.0f) dx = 1;
		else if (theta >= M_PI / 4.0f && theta < 3.0f * M_PI / 4.0f) dy = 1;
		else if (theta < -M_PI / 4.0f && theta >= -3.0f * M_PI / 4.0f) dy = -1;
		else dx = -1;

		const int nx = (int)current.x + dx;
		const int ny = (int)current.y + dy;
		if (nx < 0 || ny < 0 || nx >= (int)map->getWidth() || ny >= (int)map->getHeight()) return false;
		out.x = (unsigned int)nx;
		out.y = (unsigned int)ny;
		return true;
	}

	/** Calcula a próxima célula no sentido oposto ao heading atual, usada para dar ré. */
	bool getPreviousGridFromHeading(const Map::Position& current, float headingRad, Map::Position& out) const {
		const float theta = normalizeSignedAngle(headingRad);
		int dx = 0;
		int dy = 0;

		if (theta >= -M_PI / 4.0f && theta < M_PI / 4.0f) dx = -1;
		else if (theta >= M_PI / 4.0f && theta < 3.0f * M_PI / 4.0f) dy = -1;
		else if (theta < -M_PI / 4.0f && theta >= -3.0f * M_PI / 4.0f) dy = 1;
		else dx = 1;

		const int nx = (int)current.x + dx;
		const int ny = (int)current.y + dy;
		if (nx < 0 || ny < 0 || nx >= (int)map->getWidth() || ny >= (int)map->getHeight()) return false;
		out.x = (unsigned int)nx;
		out.y = (unsigned int)ny;
		return true;
	}

	/**
	 * Fixa a pose contínua exatamente na célula discreta esperada.
	 *
	 * Esta função é usada ao fim de cada célula. Na prática, ela remove o erro
	 * residual pequeno de odometria/gyro e impede que a pausa entre células gere
	 * drift acumulado. A pose contínua e a célula do mapa voltam a ficar
	 * sincronizadas, mantendo o A* e a Web UI consistentes.
	 */
	void snapPoseToExpectedCell() {
		if (!pose || !map) return;
		pose->reset(
			gridToWorldX(expectedNextGrid),
			gridToWorldY(expectedNextGrid),
			targetHeadingRad,
			getSignedLeftTicks(),
			getSignedRightTicks()
		);
		map->setPosition(expectedNextGrid.x, expectedNextGrid.y);
	}

	/**
	 * Registra a célula onde um obstáculo foi observado, sem alterar o mapa-base.
	 *
	 * O mapa oficial do TCC deve permanecer o binário 7x7 carregado na inicialização:
	 * bordas ocupadas (1) e miolo livre (0). Esta função apenas gera log para debug.
	 */
	bool markObstacleAhead() {
		Map::Position obstacle;
		const bool hasCell = currentCellReverse ? getBehindCell(obstacle) : getAheadCell(obstacle);
		if (!hasCell) return false;
		webLog("[GridCellNavigator] Obstáculo detectado em X=" + String(obstacle.x) + " Y=" + String(obstacle.y) + " | mapa 7x7 preservado, sem setCell().\n");
		return true;
	}

	/** Calcula novamente o caminho até o alvo final, preservando obstáculos já descobertos. */
	bool replanPathFromCurrent() {
		if (!pathMode || !map) return false;
		lastReplanTriggered = true;
		updateMapCellFromPose();
		map->setTarget(finalTarget.x, finalTarget.y);
		size_t newLen = 0;
		const bool ok = map->findPathAStar(finalTarget.x, finalTarget.y, path, PATH_BUFFER_SIZE, newLen);
		if (!ok || newLen < 2) {
			pathLen = 0;
			pathIndex = 0;
			return false;
		}
		pathLen = newLen;
		pathIndex = 1;
		requestedCells = (int)pathLen - 1;
		completedCells = 0;
		webLog("[GridCellNavigator] Caminho replanejado automaticamente. Novo tamanho: " + String(pathLen) + " células.\n");
		notifyStatus("path_replanned");
		return true;
	}

	/** Registra uma linha de experimento, quando o logger estiver habilitado. */
	void logExperiment(const String& eventName, const String& note = "") {
		if (!logger || !pose || !map) return;
		const Map::Position p = map->getPosition();
		const float headingError = normalizeSignedAngle(pose->getTheta() - targetHeadingRad);
		logger->log(
			eventName,
			getStateName(),
			requestedCells,
			completedCells,
			(int)p.x,
			(int)p.y,
			pose->getX(),
			pose->getY(),
			pose->getTheta(),
			targetHeadingRad,
			headingError,
			getDistanceFromCellStart(),
			lastFrontDistanceMm,
			lastObstacleDetected,
			pathMode,
			pathIndex,
			pathLen,
			lastReplanTriggered,
			note
		);
		lastReplanTriggered = false;
	}

	/** Publica o estado da navegação no WebSocket do carro, quando houver cliente conectado. */
	void notifyStatus(const char* eventName) {
		StaticJsonDocument<512> doc;
		doc["type"] = "cell_nav";
		doc["event"] = eventName;
		doc["state"] = getStateName();
		doc["requested"] = requestedCells;
		doc["completed"] = completedCells;
		doc["cell_size_cm"] = GRID_CELL_SIZE_CM;
		doc["front_distance_mm"] = lastFrontDistanceMm;
		doc["obstacle"] = lastObstacleDetected;
		doc["path_mode"] = pathMode;
		doc["path_index"] = pathIndex;
		doc["path_len"] = pathLen;
		doc["heading_error_deg"] = normalizeSignedAngle(pose->getTheta() - targetHeadingRad) * (180.0f / M_PI);
		doc["reverse"] = currentCellReverse;
		if (abortReason.length() > 0) doc["reason"] = abortReason;

		String payload;
		payload.reserve(512);
		serializeJson(doc, payload);
		if (auto cs = WebInterface::getInstance().getCarSocket()) {
			if (cs->count() > 0) cs->textAll(payload);
		}
		logExperiment(eventName);
	}

	/** Aborta a navegação, para os motores e registra o motivo. */
	void abort(const String& reason) {
		abortReason = reason;
		state = STATE_ABORTED;
		stateStartedAt = millis();
		if (ponte) {
			ponte->stop();
		}
		webLog("[GridCellNavigator] Navegação abortada: " + abortReason + "\n");
		notifyStatus("aborted");
	}

	/** Prepara a próxima célula do caminho A*, fazendo giro se necessário. */
	void prepareNextPathStep() {
		if (!pathMode) {
			state = STATE_START_CELL;
			return;
		}
		if (pathIndex >= pathLen) {
			state = STATE_FINISHED;
			stateStartedAt = millis();
			notifyStatus("path_finished");
			return;
		}
		const Map::Position current = map->getPosition();
		const Map::Position next = path[pathIndex];
		const float stepHeading = headingForStep(current, next);
		currentCellReverse = shouldReverseForStep(current, next);
		// Se a célula está atrás, mantém a orientação atual e executa a célula de ré.
		// Caso contrário, alinha normalmente para o heading do próximo passo.
		targetHeadingRad = currentCellReverse ? pose->getTheta() : stepHeading;
		state = STATE_ALIGN_TO_PATH_STEP;
		stateStartedAt = millis();
		notifyStatus(currentCellReverse ? "path_step_selected_reverse" : "path_step_selected");
	}

public:
	GridCellNavigator(PonteH* ponteRef, PoseEKF* poseRef, Map* mapRef, VL53L0X* sensorRef, ExperimentLogger* loggerRef, Encoder* leftEncoderRef, Encoder* rightEncoderRef) :
		ponte(ponteRef), pose(poseRef), map(mapRef), frontSensor(sensorRef), logger(loggerRef), leftEncoder(leftEncoderRef), rightEncoder(rightEncoderRef) {}

	/** Retorna true quando a máquina de estados está executando um comando. */
	bool isActive() const {
		return state != STATE_IDLE && state != STATE_FINISHED && state != STATE_ABORTED;
	}

	/**
	 * Limpa estados finais antigos quando o robô já está parado.
	 *
	 * Isso evita que a telemetria mantenha o navegador como "finished" ou
	 * "aborted" por tempo indefinido e deixa claro para a Web UI que não há
	 * comando pendente. Não inicia movimento e não altera o mapa.
	 */
	void resetIdleIfTerminal() {
		if (state == STATE_FINISHED || state == STATE_ABORTED) {
			state = STATE_IDLE;
		}
	}

	int getCompletedCells() const { return completedCells; }
	int getRequestedCells() const { return requestedCells; }
	const char* getStatusName() const { return getStateName(); }
	String getAbortReason() const { return abortReason; }
	bool isPathMode() const { return pathMode; }
	size_t getPathIndex() const { return pathIndex; }
	size_t getPathLen() const { return pathLen; }
	int getLastFrontDistanceMm() const { return lastFrontDistanceMm; }
	bool getLastObstacleDetected() const { return lastObstacleDetected; }
	bool isCurrentCellReverse() const { return currentCellReverse; }

	/** Inicia o comando "vá X células para frente". */
	bool startForwardCells(int cells) {
		if (!ponte || !pose || !map) {
			webLog("[GridCellNavigator] Erro: dependências não configuradas.\n");
			return false;
		}
		if (cells <= 0) {
			webLog("[GridCellNavigator] Comando recusado: número de células inválido.\n");
			return false;
		}
		if (isActive()) {
			webLog("[GridCellNavigator] Comando recusado: navegação em andamento.\n");
			return false;
		}

		pathMode = false;
		currentCellReverse = false;
		pathLen = 0;
		pathIndex = 0;
		requestedCells = cells;
		completedCells = 0;
		targetHeadingRad = pose->getTheta();
		abortReason = "";
		lastObstacleDetected = false;
		lastReplanTriggered = false;
		state = STATE_START_CELL;
		stateStartedAt = millis();
		webLog("[GridCellNavigator] Iniciando avanço de " + String(cells) + " célula(s) de " + String(GRID_CELL_SIZE_CM) + " cm.\n");
		notifyStatus("started");
		return true;
	}

	/** Planeja e executa automaticamente um caminho A* até uma célula do mapa. */
	bool startPathTo(unsigned int targetX, unsigned int targetY) {
		if (!ponte || !pose || !map) return false;
		if (isActive()) return false;
		updateMapCellFromPose();
		if (!map->setTarget(targetX, targetY)) return false;

		size_t outLen = 0;
		const bool ok = map->findPathAStar(targetX, targetY, path, PATH_BUFFER_SIZE, outLen);
		if (!ok || outLen < 2) {
			webLog("[GridCellNavigator] Não foi possível planejar caminho até o alvo.\n");
			return false;
		}

		pathMode = true;
		pathLen = outLen;
		pathIndex = 1;
		finalTarget.x = targetX;
		finalTarget.y = targetY;
		requestedCells = (int)pathLen - 1;
		completedCells = 0;
		abortReason = "";
		lastObstacleDetected = false;
		lastReplanTriggered = false;
		currentCellReverse = false;
		stateStartedAt = millis();
		webLog("[GridCellNavigator] Executando caminho A* com " + String(pathLen) + " célula(s).\n");
		prepareNextPathStep();
		notifyStatus("path_started");
		return true;
	}

	/** Cancela manualmente o comando atual de navegação por células/caminho. */
	void cancel() {
		if (ponte) {
			ponte->stop();
		}
		abortReason = "cancelled";
		state = STATE_ABORTED;
		stateStartedAt = millis();
		notifyStatus("cancelled");
	}

	/** Copia o caminho interno para outro buffer para respostas WebSocket. */
	size_t copyPath(Map::Position* out, size_t maxLen) const {
		if (!out || maxLen == 0) return 0;
		const size_t n = pathLen < maxLen ? pathLen : maxLen;
		for (size_t i = 0; i < n; ++i) out[i] = path[i];
		return n;
	}

	/** Executa um passo não bloqueante da máquina de estados. */
	void loop() {
		if (!isActive()) return;

		const unsigned long now = millis();
		// Durante a navegação ativa, a célula oficial do mapa fica travada.
		// Ela só é atualizada no início/fim de célula ou em replanejamento.
		// Isso evita que ruído intermediário do EKF faça o A* "pular" células.
		lastObstacleDetected = false;

		if (logger && logger->shouldLogPeriodic()) {
			logExperiment("periodic");
		}

		switch (state) {
			case STATE_ALIGN_TO_PATH_STEP: {
				const float headingError = normalizeSignedAngle(pose->getTheta() - targetHeadingRad);
				const float absError = fabsf(headingError);
				if (absError <= CELL_HEADING_TOLERANCE_RAD) {
					state = STATE_START_CELL;
					stateStartedAt = now;
					notifyStatus("path_step_aligned");
					break;
				}
				const float correctionDeg = absError * (180.0f / M_PI);
				if (headingError > 0.0f) ponte->turnRight(correctionDeg);
				else ponte->turnLeft(correctionDeg);
				syncEncoderReferenceForEKF();
				state = STATE_WAIT_PATH_TURN;
				stateStartedAt = now;
				notifyStatus("path_turn_started");
				break;
			}

			case STATE_WAIT_PATH_TURN: {
				if (ponte->isStopped()) {
					// A curva foi comandada para alinhar com targetHeadingRad. Não usa QMC aqui:
					// ele sofre interferência durante/ logo após os motores. Fixamos o heading lógico
					// no alvo planejado para evitar correções residuais absurdas de 170°/180°.
					pose->reset(pose->getX(), pose->getY(), targetHeadingRad, getSignedLeftTicks(), getSignedRightTicks());
					state = STATE_ALIGN_TO_PATH_STEP;
					stateStartedAt = now;
					notifyStatus("path_turn_finished");
				}
				break;
			}

			case STATE_START_CELL: {
				updateMapCellFromPose();
				cellStartGrid = map->getPosition();

				if (pathMode) {
					if (pathIndex >= pathLen) {
						state = STATE_FINISHED;
						stateStartedAt = now;
						notifyStatus("path_finished");
						break;
					}
					expectedNextGrid = path[pathIndex];
				} else {
					currentCellReverse = false;
					if (!getNextGridFromHeading(cellStartGrid, targetHeadingRad, expectedNextGrid)) {
						abort("next_cell_out_of_bounds");
						break;
					}
				}

				if (map->getCell(expectedNextGrid.x, expectedNextGrid.y) == 1U) {
					abort("next_cell_occupied");
					break;
				}

				cellStartX = pose->getX();
				cellStartY = pose->getY();
				pose->holdCurrentPose(getSignedLeftTicks(), getSignedRightTicks());
				if (currentCellReverse) ponte->backward();
				else ponte->forward();
				// PonteH/Motor reseta os encoders físicos ao iniciar movimento.
				// Sincronizamos a referência imediatamente para a primeira célula
				// não herdar deltas antigos e "andar" metros no log.
				syncEncoderReferenceForEKF();
				cellStartLeftTicks = getSignedLeftTicks();
				cellStartRightTicks = getSignedRightTicks();
				state = STATE_MOVING_CELL;
				stateStartedAt = now;
				webLog("[GridCellNavigator] " + String(currentCellReverse ? "Dando ré" : "Avançando") + " célula " + String(completedCells + 1) + "/" + String(requestedCells) + " para X=" + String(expectedNextGrid.x) + " Y=" + String(expectedNextGrid.y) + ".\n");
				notifyStatus("cell_started");
				break;
			}

			case STATE_MOVING_CELL: {
				const float travelled = getDistanceFromCellStart();

				if (frontSensor && !currentCellReverse) {
					lastFrontDistanceMm = getCachedFrontDistanceMm();
					if (lastFrontDistanceMm > 0 && lastFrontDistanceMm <= FRONT_OBSTACLE_REPLAN_MM) {
						lastObstacleDetected = true;
						ponte->stop();
						markObstacleAhead();
						// O mapa-base 7x7 não deve ser alterado por detecções dinâmicas.
						// Por isso, nesta versão o robô para e aborta a execução em vez de escrever 1 no bitmap.
						abort("obstacle_detected_map_preserved");
						break;
					}
				}

				if (now - stateStartedAt > CELL_MAX_TIME_MS) {
					abort("cell_timeout");
					break;
				}

				if (ponte->isStopped() && travelled < (GRID_CELL_SIZE_M - CELL_TARGET_TOLERANCE_M)) {
					abort("stopped_before_cell_target");
					break;
				}

				if (travelled > (GRID_CELL_SIZE_M * CELL_OVERRUN_ABORT_FACTOR)) {
					ponte->stop();
					abort("cell_distance_overrun");
					break;
				}

				if (travelled >= (GRID_CELL_SIZE_M - CELL_TARGET_TOLERANCE_M)) {
					ponte->stop();
					completedCells++;
					if (pathMode && pathIndex < pathLen) pathIndex++;
					snapPoseToExpectedCell();
					state = STATE_SETTLE_AFTER_CELL;
					stateStartedAt = now;
					webLog("[GridCellNavigator] Célula concluída. Pose fixada em X=" + String(expectedNextGrid.x) + " Y=" + String(expectedNextGrid.y) + ". Distância: " + String(travelled, 3) + " m.\n");
					notifyStatus("cell_completed");
				}
				break;
			}

			case STATE_SETTLE_AFTER_CELL: {
				// Durante a pausa entre células, a pose deve ficar congelada.
				// Isso impede que ruído de encoder/gyro altere a célula atual antes
				// do próximo comando de avanço.
				pose->holdCurrentPose(getSignedLeftTicks(), getSignedRightTicks());
				map->setPosition(expectedNextGrid.x, expectedNextGrid.y);
				if (now - stateStartedAt >= 300) {
					state = STATE_CORRECT_HEADING;
					stateStartedAt = now;
				}
				break;
			}

			case STATE_CORRECT_HEADING: {
				const float headingError = normalizeSignedAngle(pose->getTheta() - targetHeadingRad);
				const float absError = fabsf(headingError);

				if (absError <= CELL_HEADING_TOLERANCE_RAD) {
					if (pathMode) {
						prepareNextPathStep();
					} else if (completedCells >= requestedCells) {
						state = STATE_FINISHED;
						stateStartedAt = now;
						webLog("[GridCellNavigator] Navegação por células finalizada.\n");
						notifyStatus("finished");
					} else {
						state = STATE_START_CELL;
						stateStartedAt = now;
						notifyStatus("next_cell");
					}
					break;
				}

				const float correctionDeg = absError * (180.0f / M_PI);
				if (headingError > 0.0f) ponte->turnRight(correctionDeg);
				else ponte->turnLeft(correctionDeg);
				syncEncoderReferenceForEKF();

				state = STATE_WAIT_TURN;
				stateStartedAt = now;
				webLog("[GridCellNavigator] Corrigindo erro angular residual de " + String(correctionDeg, 2) + " graus.\n");
				notifyStatus("heading_correction_started");
				break;
			}

			case STATE_WAIT_TURN: {
				if (ponte->isStopped()) {
					pose->reset(pose->getX(), pose->getY(), targetHeadingRad, getSignedLeftTicks(), getSignedRightTicks());
					state = STATE_SETTLE_AFTER_CELL;
					stateStartedAt = now;
					notifyStatus("heading_correction_finished");
				}
				break;
			}

			default:
				break;
		}
	}
};

bool ConnectToWiFi(unsigned long timeoutMs = 15000) {
	if (WiFi.status() == WL_CONNECTED) {
		return true;
	}

	WiFi.begin(ssid, password);
	unsigned long startedAt = millis();
	#ifdef DEBUG_PRINTS
		Serial.print("Conectando ao WiFi -> ");
		Serial.print(ssid);
	#endif
	while (WiFi.status() != WL_CONNECTED && (millis() - startedAt) < timeoutMs) {
		#ifdef DEBUG_PRINTS
			Serial.print(".");
		#endif
		delay(100);
	}

	const bool connected = (WiFi.status() == WL_CONNECTED);
	#ifdef DEBUG_PRINTS
		if (connected) {
			Serial.println(" WiFi conectado!");
			Serial.print("Endereco IP: ");
			Serial.println(WiFi.localIP());
		} else {
			Serial.println(" Falha ao conectar dentro do timeout.");
		}
	#endif
	return connected;
}

#pragma region "Global Objects"

// ——————— Sensores ———————
VL53L0X *sensor = new VL53L0X();	// VL53L0X no I²C (SDA=21, SCL=22)
QMC5883LCompass *sensorQMC = new QMC5883LCompass();	// QMC5883L no I²C (SDA=21, SCL=22)


// ——————— Cache assíncrono dos sensores ———————
struct SensorSnapshot {
	// Heading/yaw relativo do QMC5883L usado pelo robô.
	float magAngleZ = 0.0f;
	// Derivada aproximada do heading Z. Mantém compatibilidade com chamadas antigas getCachedGyroZ().
	float magRateZ = 0.0f;
	int16_t magRawX = 0;
	int16_t magRawY = 0;
	int16_t magRawZ = 0;
	float magGaussX = 0.0f;
	float magGaussY = 0.0f;
	float magGaussZ = 0.0f;
	uint8_t magStatus = 0;
	uint8_t magAddress = 0;
	uint8_t magChipId = 0;
	int frontDistanceMm = 99999;
	unsigned long magUpdatedAt = 0;
	unsigned long tofUpdatedAt = 0;
};

SensorSnapshot sensorSnapshot;
SemaphoreHandle_t sensorSnapshotMutex = nullptr;
SemaphoreHandle_t i2cBusMutex = nullptr;
TaskHandle_t qmcTaskHandle = nullptr;
TaskHandle_t tofTaskHandle = nullptr;

// Contadores assinados usados pelo EKF.
// Os encoders físicos do projeto contam pulsos absolutos; para o EKF saber se o
// robô foi para frente, para trás ou girou, transformamos os deltas brutos em
// ticks com sinal conforme o comando atual da PonteH.
long ekfSignedLeftTicks = 0;
long ekfSignedRightTicks = 0;
long lastRawLeftTicksForEKF = 0;
long lastRawRightTicksForEKF = 0;

/** Copia atômica simples do cache de sensores para evitar leituras parcialmente atualizadas. */
SensorSnapshot getSensorSnapshot() {
	SensorSnapshot copy;
	if (sensorSnapshotMutex && xSemaphoreTake(sensorSnapshotMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
		copy = sensorSnapshot;
		xSemaphoreGive(sensorSnapshotMutex);
	} else {
		copy = sensorSnapshot;
	}
	return copy;
}

int getCachedFrontDistanceMm() { return getSensorSnapshot().frontDistanceMm; }
float getCachedMagAngleZ() { return getSensorSnapshot().magAngleZ; }
float getCachedMagRateZ() { return getSensorSnapshot().magRateZ; }
// Wrapper mantido para não quebrar partes antigas do controle que ainda usam getCachedGyroZ().
// Agora ele representa a taxa angular Z estimada pela variação do heading do QMC5883L.
float getCachedGyroZ() { return getCachedMagRateZ(); }
unsigned long getCachedQmcAgeMs() {
	const SensorSnapshot s = getSensorSnapshot();
	return s.magUpdatedAt == 0 ? 999999UL : millis() - s.magUpdatedAt;
}
unsigned long getCachedMagAgeMs() { return getCachedQmcAgeMs(); }
unsigned long getCachedTofAgeMs() {
	const SensorSnapshot s = getSensorSnapshot();
	return s.tofUpdatedAt == 0 ? 999999UL : millis() - s.tofUpdatedAt;
}

/** Thread dedicada ao QMC5883L. Mantém heading e taxa angular estimada fora do loop principal. */
void qmcSensorTask(void* parameter) {
	(void)parameter;
	for (;;) {
		if (i2cBusMutex && xSemaphoreTake(i2cBusMutex, pdMS_TO_TICKS(20)) == pdTRUE) {
			sensorQMC->loop();
			xSemaphoreGive(i2cBusMutex);
		}
		if (sensorSnapshotMutex && xSemaphoreTake(sensorSnapshotMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
			sensorSnapshot.magAngleZ = sensorQMC->getAngleZRad();
			sensorSnapshot.magRateZ = sensorQMC->getAngularVelocityZRadS();
			sensorSnapshot.magRawX = sensorQMC->getRawX();
			sensorSnapshot.magRawY = sensorQMC->getRawY();
			sensorSnapshot.magRawZ = sensorQMC->getRawZ();
			sensorSnapshot.magGaussX = sensorQMC->getXGauss();
			sensorSnapshot.magGaussY = sensorQMC->getYGauss();
			sensorSnapshot.magGaussZ = sensorQMC->getZGauss();
			sensorSnapshot.magStatus = sensorQMC->getStatus();
			sensorSnapshot.magAddress = sensorQMC->getAddress();
			sensorSnapshot.magChipId = sensorQMC->getChipId();
			sensorSnapshot.magUpdatedAt = millis();
			xSemaphoreGive(sensorSnapshotMutex);
		}
		vTaskDelay(pdMS_TO_TICKS(20));
	}
}

/** Thread dedicada ao VL53L0X. Evita que rangingTest trave o controle dos motores. */
void tofSensorTask(void* parameter) {
	(void)parameter;
	for (;;) {
		int d = 99999;
		if (i2cBusMutex && xSemaphoreTake(i2cBusMutex, pdMS_TO_TICKS(40)) == pdTRUE) {
			d = sensor->loop();
			xSemaphoreGive(i2cBusMutex);
		}
		if (sensorSnapshotMutex && xSemaphoreTake(sensorSnapshotMutex, pdMS_TO_TICKS(2)) == pdTRUE) {
			sensorSnapshot.frontDistanceMm = d;
			sensorSnapshot.tofUpdatedAt = millis();
			xSemaphoreGive(sensorSnapshotMutex);
		}
		vTaskDelay(pdMS_TO_TICKS(50));
	}
}

/** Cria as tarefas FreeRTOS de leitura dos sensores. */
void startSensorTasks() {
	if (!sensorSnapshotMutex) sensorSnapshotMutex = xSemaphoreCreateMutex();
	if (!i2cBusMutex) i2cBusMutex = xSemaphoreCreateMutex();

	if (!qmcTaskHandle) {
		xTaskCreatePinnedToCore(qmcSensorTask, "qmc5883l_task", 4096, nullptr, 2, &qmcTaskHandle, 0);
	}
	if (!tofTaskHandle) {
		xTaskCreatePinnedToCore(tofSensorTask, "vl53l0x_task", 4096, nullptr, 1, &tofTaskHandle, 0);
	}
	webLog("[Sensors] Threads FreeRTOS iniciadas: QMC5883L e VL53L0X.\n");
}

// ——————— Encoders ———————
// Motor Direito
Encoder *encoderD = new Encoder(39, 36);  // CH A=36, CH B=39
// Motor Esquerdo
Encoder *encoderE = new Encoder(34, 35);  // CH A=35, CH B=34

Map *robotMap = new Map(GRID_WIDTH, GRID_HEIGHT);
PoseEKF *poseEKF = new PoseEKF(0.0325f, 0.1400f, GRID_CELL_SIZE_M, 20.0f);  // Pose Extended Kalman Filter; célula lógica de 30 cm
Map::Position plannedPath[PATH_BUFFER_SIZE];
size_t plannedPathLen = 0;
bool hasPlannedPath = false;

// ——————— Motores com PID ———————
// Motor Direito  → IN1=14, IN2=12, PWM=13
Motor *motorDireito  = new Motor( 14, 12, 13, 0, encoderD, intentKp, intentKi, intentKd );
// Motor Esquerdo → IN1=26, IN2=27, PWM=25
Motor *motorEsquerdo = new Motor( 26, 27, 25, 1, encoderE, intentKp, intentKi, intentKd );

// ——————— Ponte H (drive de 2 motores) ———————
PonteH *ponte = new PonteH(motorDireito, motorEsquerdo);

// ——————— Logger experimental do TCC ———————
ExperimentLogger experimentLogger("/tcc_experiment_log.csv");

// ——————— Navegação discreta em células de 30 cm x 30 cm ———————
GridCellNavigator *cellNavigator = new GridCellNavigator(ponte, poseEKF, robotMap, sensor, &experimentLogger, encoderE, encoderD);

long getSignedLeftTicks() { return ekfSignedLeftTicks; }
long getSignedRightTicks() { return ekfSignedRightTicks; }
long getRawLeftEncoderTicks() { return encoderE ? encoderE->getTotalTicks() : 0; }
long getRawRightEncoderTicks() { return encoderD ? encoderD->getTotalTicks() : 0; }

/**
 * Sincroniza a referência bruta dos encoders físicos com os contadores
 * assinados usados pelo EKF.
 *
 * Os motores resetam seus encoders ao mudar de direção/parar. Sem esta
 * sincronização, o próximo delta bruto pode incluir lixo ou ficar negativo,
 * gerando saltos como a primeira célula registrada com 1,226 m.
 */
void syncEncoderReferenceForEKF() {
	if (!encoderE || !encoderD) return;
	lastRawLeftTicksForEKF = encoderE->getTotalTicks();
	lastRawRightTicksForEKF = encoderD->getTotalTicks();
	if (poseEKF) {
		poseEKF->holdCurrentPose(ekfSignedLeftTicks, ekfSignedRightTicks);
	}
}

/**
 * Atualiza os ticks assinados usados pelo EKF.
 *
 * Os encoders E2-Q2 deste projeto retornam contagem acumulada absoluta. Isso
 * funciona para medir distância percorrida, mas não indica direção. Sem esta
 * camada, ao dar ré o EKF interpretaria os pulsos como avanço para frente.
 * A função abaixo aplica sinal aos deltas de encoder com base no comando atual:
 * frente (+,+), ré (-,-), giro esquerda (-,+), giro direita (+,-).
 */
void updateSignedEncoderTicks() {
	const long rawLeft = encoderE->getTotalTicks();
	const long rawRight = encoderD->getTotalTicks();
	long deltaLeft = rawLeft - lastRawLeftTicksForEKF;
	long deltaRight = rawRight - lastRawRightTicksForEKF;

	lastRawLeftTicksForEKF = rawLeft;
	lastRawRightTicksForEKF = rawRight;

	// Trava pulsos impossíveis antes de entrarem no EKF. Isso não altera o contador físico,
	// só evita que um ruído isolado de encoder vire salto de theta/x/y.
	if (abs(deltaLeft) > EKF_MAX_ENCODER_DELTA_TICKS_PER_LOOP) {
		webLog("[EKF] Spike ignorado no encoder esquerdo: delta=" + String(deltaLeft) + " ticks\n");
		deltaLeft = 0;
	}
	if (abs(deltaRight) > EKF_MAX_ENCODER_DELTA_TICKS_PER_LOOP) {
		webLog("[EKF] Spike ignorado no encoder direito: delta=" + String(deltaRight) + " ticks\n");
		deltaRight = 0;
	}

	if (!ponte || ponte->isStopped()) {
		return;
	}

	switch (ponte->getCurrentMove()) {
		case MOVEMENT_FORWARD:
			ekfSignedLeftTicks += deltaLeft;
			ekfSignedRightTicks += deltaRight;
			break;
		case MOVEMENT_BACKWARDS:
			ekfSignedLeftTicks -= deltaLeft;
			ekfSignedRightTicks -= deltaRight;
			break;
		case MOVEMENT_TURN_LEFT:
			ekfSignedLeftTicks -= deltaLeft;
			ekfSignedRightTicks += deltaRight;
			break;
		case MOVEMENT_TURN_RIGHT:
			ekfSignedLeftTicks += deltaLeft;
			ekfSignedRightTicks -= deltaRight;
			break;
		default:
			break;
	}
}

/**
 * Converte a posição contínua estimada pelo EKF para a célula discreta do mapa.
 *
 * Convenção usada no TCC:
 * - célula (0,0): canto superior esquerdo do mapa;
 * - célula (1,1): posição inicial do robô, logo dentro da borda ocupada;
 * - eixo X positivo: deslocamento para a direita;
 * - eixo Y positivo: deslocamento para baixo na matriz do mapa;
 * - cada célula tem 0,30 m x 0,30 m.
 */
Map::Position getRobotGridPositionFromPose() {
	int mapX = MAP_ORIGIN_X + (int)roundf(poseEKF->getX() / GRID_CELL_SIZE_M);
	int mapY = MAP_ORIGIN_Y + (int)roundf(poseEKF->getY() / GRID_CELL_SIZE_M);
	mapX = (int)clamp(mapX, 0, robotMap->getWidth() - 1);
	mapY = (int)clamp(mapY, 0, robotMap->getHeight() - 1);
	return Map::Position{(unsigned int)mapX, (unsigned int)mapY};
}

/** Atualiza a posição discreta oficial do mapa com base na pose contínua do EKF. */
void updateRobotMapPositionFromPose() {
	const Map::Position p = getRobotGridPositionFromPose();
	robotMap->setPosition(p.x, p.y);
}

/**
 * Cria o mapa binário 7x7 usado nos testes do TCC.
 *
 * A borda do mapa é ocupada (1) para representar paredes/limites físicos.
 * O miolo 5x5 é livre (0), permitindo planejar caminhos internos com A*.
 * O robô inicia em (1,1), isto é, no primeiro quadrado livre do canto superior esquerdo.
 */
void initializeTccBinaryMap7x7() {
	robotMap->resize(GRID_WIDTH, GRID_HEIGHT);
	for (unsigned int y = 0; y < robotMap->getHeight(); ++y) {
		for (unsigned int x = 0; x < robotMap->getWidth(); ++x) {
			const bool border = (x == 0 || y == 0 || x == robotMap->getWidth() - 1 || y == robotMap->getHeight() - 1);
			robotMap->setCell(x, y, border ? 1 : 0);
		}
	}
	robotMap->setPosition(MAP_ORIGIN_X, MAP_ORIGIN_Y);
	robotMap->clearTarget();
	hasPlannedPath = false;
	plannedPathLen = 0;
}

// ——————— Outros ———————
// LED da carroceria (MQTT)
LED led_carro(2);

void sendTelemetryFrame() {
	if (auto cs = WebInterface::getInstance().getCarSocket()) {
		if (cs->count() == 0) return;
	} else {
		return;
	}

	StaticJsonDocument<1536> doc;
	doc["led"] = led_carro.isOn();
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

	JsonObject cellNav = doc["cell_nav"].to<JsonObject>();
	cellNav["state"] = cellNavigator->getStatusName();
	cellNav["active"] = cellNavigator->isActive();
	cellNav["requested"] = cellNavigator->getRequestedCells();
	cellNav["completed"] = cellNavigator->getCompletedCells();
	cellNav["cell_size_cm"] = GRID_CELL_SIZE_CM;
	cellNav["path_mode"] = cellNavigator->isPathMode();
	cellNav["path_index"] = cellNavigator->getPathIndex();
	cellNav["path_len"] = cellNavigator->getPathLen();
	cellNav["front_distance_mm"] = cellNavigator->getLastFrontDistanceMm();
	cellNav["obstacle"] = cellNavigator->getLastObstacleDetected();
	cellNav["logger_enabled"] = experimentLogger.isEnabled();
	cellNav["reverse"] = cellNavigator->isCurrentCellReverse();
	if (cellNavigator->getAbortReason().length() > 0) {
		cellNav["abort_reason"] = cellNavigator->getAbortReason();
	}

	const Map::Position gridPos = getRobotGridPositionFromPose();
	JsonObject location = doc["location"].to<JsonObject>();
	location["grid_x"] = gridPos.x;
	location["grid_y"] = gridPos.y;
	location["world_x_m"] = poseEKF->getX();
	location["world_y_m"] = poseEKF->getY();
	location["theta_rad"] = poseEKF->getTheta();
	location["theta_deg"] = poseEKF->getTheta() * (180.0f / M_PI);
	location["cell_size_m"] = GRID_CELL_SIZE_M;
	location["origin_grid_x"] = MAP_ORIGIN_X;
	location["origin_grid_y"] = MAP_ORIGIN_Y;

	const SensorSnapshot sensorState = getSensorSnapshot();
	JsonObject mag = doc["magnetometer"].to<JsonObject>();
	mag["angle_z_rad"] = sensorState.magAngleZ;
	mag["heading_rad"] = sensorState.magAngleZ;
	mag["heading_deg"] = sensorState.magAngleZ * (180.0f / M_PI);
	mag["rate_z_rad_s"] = sensorState.magRateZ;
	mag["raw_x"] = sensorState.magRawX;
	mag["raw_y"] = sensorState.magRawY;
	mag["raw_z"] = sensorState.magRawZ;
	mag["gauss_x"] = sensorState.magGaussX;
	mag["gauss_y"] = sensorState.magGaussY;
	mag["gauss_z"] = sensorState.magGaussZ;
	mag["status"] = sensorState.magStatus;
	mag["address"] = sensorState.magAddress;
	mag["chip_id"] = sensorState.magChipId;
	mag["data_ready"] = (sensorState.magStatus & 0x01) != 0;
	mag["overflow"] = (sensorState.magStatus & 0x02) != 0;
	mag["data_skip"] = (sensorState.magStatus & 0x04) != 0;
	mag["declination_deg"] = sensorQMC->declinationDeg;
	mag["zero_z_rad"] = sensorQMC->angleZeroOffsetZRad;

	// Mantém apenas gyro.z para compatibilidade com trechos antigos da UI/controle.
	// Não há gyro físico: este valor vem da derivada do heading do magnetômetro.
	JsonObject gyro = doc["gyro"].to<JsonObject>();
	gyro["z"] = sensorState.magRateZ;
	gyro["source"] = "QMC5883L_heading_rate";
	doc["distance"] = getCachedFrontDistanceMm();
	doc["sensor_age"]["mag_ms"] = getCachedMagAgeMs();
	doc["sensor_age"]["tof_ms"] = getCachedTofAgeMs();

	JsonObject motorD = doc["motorD"].to<JsonObject>();
	motorD["rpm"] = motorDireito->getRPM();
	motorD["pidOutput"] = motorDireito->getPIDOutput();
	motorD["targetRPM"] = motorDireito->getTargetRPM();
	motorD["isClockwise"] = motorDireito->isClockwise();
	motorD["ticks"] = encoderD->getTotalTicks();
	motorD["turns"] = encoderD->getTotalRevolutions(ENCODER_TEETH);
	motorD["fullTurn"] = encoderD->hasCompletedFullTurn(ENCODER_TEETH);

	JsonObject motorE = doc["motorE"].to<JsonObject>();
	motorE["rpm"] = motorEsquerdo->getRPM();
	motorE["pidOutput"] = motorEsquerdo->getPIDOutput();
	motorE["targetRPM"] = motorEsquerdo->getTargetRPM();
	motorE["isClockwise"] = motorEsquerdo->isClockwise();
	motorE["ticks"] = encoderE->getTotalTicks();
	motorE["turns"] = encoderE->getTotalRevolutions(ENCODER_TEETH);
	motorE["fullTurn"] = encoderE->hasCompletedFullTurn(ENCODER_TEETH);

	// PoseEKF (Pose Extended Kalman Filter) - PRINCIPAL
	JsonObject pose = doc["pose"].to<JsonObject>();
	pose["x"] = poseEKF->getX();
	pose["y"] = poseEKF->getY();
	pose["theta"] = poseEKF->getTheta();
	pose["grid_x"] = gridPos.x;
	pose["grid_y"] = gridPos.y;
	pose["origin_grid_x"] = MAP_ORIGIN_X;
	pose["origin_grid_y"] = MAP_ORIGIN_Y;
	pose["cell_size_m"] = GRID_CELL_SIZE_M;
	pose["cov_x"] = poseEKF->getCovariance_X();
	pose["cov_y"] = poseEKF->getCovariance_Y();
	pose["cov_theta"] = poseEKF->getCovariance_Theta();

	// Odometria (referência de debug)
	// JsonObject odom = doc["odometry_debug"].to<JsonObject>();
	// odom["x"] = robotOdom->getX();
	// odom["y"] = robotOdom->getY();
	// odom["theta"] = robotOdom->getTheta();

	String output;
	output.reserve(1536);
	serializeJson(doc, output);
	if (auto cs = WebInterface::getInstance().getCarSocket()) cs->textAll(output);
}

String getMapSnapshotJson() {
	const Map::Position robotPos = robotMap->getPosition();
	const unsigned int mapWidth = robotMap->getWidth();
	const unsigned int mapHeight = robotMap->getHeight();

	if (mapWidth == 0 || mapHeight == 0) {
		return "{\"type\":\"map\",\"error\":\"empty_map\"}";
	}

	const unsigned int viewWidth = (MAP_VIEW_WIDTH < mapWidth) ? MAP_VIEW_WIDTH : mapWidth;
	const unsigned int viewHeight = (MAP_VIEW_HEIGHT < mapHeight) ? MAP_VIEW_HEIGHT : mapHeight;

	unsigned int startX = 0;
	unsigned int startY = 0;

	if (robotPos.x > (viewWidth / 2U)) {
		startX = robotPos.x - (viewWidth / 2U);
	}
	if (robotPos.y > (viewHeight / 2U)) {
		startY = robotPos.y - (viewHeight / 2U);
	}
	if (startX + viewWidth > mapWidth) {
		startX = mapWidth - viewWidth;
	}
	if (startY + viewHeight > mapHeight) {
		startY = mapHeight - viewHeight;
	}

	DynamicJsonDocument doc(16384);
	doc["type"] = "map";
	JsonObject mapObj = doc["map"].to<JsonObject>();
	mapObj["width"] = mapWidth;
	mapObj["height"] = mapHeight;
	mapObj["viewWidth"] = viewWidth;
	mapObj["viewHeight"] = viewHeight;
	mapObj["viewStartX"] = startX;
	mapObj["viewStartY"] = startY;

	JsonObject robot = mapObj["robot"].to<JsonObject>();
	robot["x"] = robotPos.x;
	robot["y"] = robotPos.y;
	robot["world_x_m"] = poseEKF->getX();
	robot["world_y_m"] = poseEKF->getY();
	robot["theta_rad"] = poseEKF->getTheta();
	robot["theta_deg"] = poseEKF->getTheta() * (180.0f / M_PI);
	robot["origin_grid_x"] = MAP_ORIGIN_X;
	robot["origin_grid_y"] = MAP_ORIGIN_Y;
	mapObj["cell_size_m"] = GRID_CELL_SIZE_M;
	mapObj["binary"] = true;
	mapObj["start_x"] = MAP_ORIGIN_X;
	mapObj["start_y"] = MAP_ORIGIN_Y;

	if (robotMap->hasTarget()) {
		const Map::Position target = robotMap->getTarget();
		JsonObject targetObj = mapObj["target"].to<JsonObject>();
		targetObj["x"] = target.x;
		targetObj["y"] = target.y;
	}

	JsonArray rows = mapObj["rows"].to<JsonArray>();
	for (unsigned int y = 0; y < viewHeight; ++y) {
		String row;
		row.reserve(viewWidth);
		const unsigned int ay = startY + y;
		for (unsigned int x = 0; x < viewWidth; ++x) {
			const unsigned int ax = startX + x;
			row += (robotMap->getCell(ax, ay) == 1U) ? '1' : '0';
		}
		rows.add(row);
	}

	JsonArray path = mapObj["path"].to<JsonArray>();
	if (hasPlannedPath) {
		for (size_t i = 0; i < plannedPathLen; ++i) {
			JsonObject step = path.add<JsonObject>();
			step["x"] = plannedPath[i].x;
			step["y"] = plannedPath[i].y;
		}
	}

	String output;
	output.reserve(8192);
	serializeJson(doc, output);
	return output;
}

// ——————— WebSocket ———————
// ——————— Car Command Handler ———————
String getCarStatus() {
	StaticJsonDocument<768> doc;
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

	doc["cell_nav"]["state"] = cellNavigator->getStatusName();
	doc["cell_nav"]["active"] = cellNavigator->isActive();
	doc["cell_nav"]["requested"] = cellNavigator->getRequestedCells();
	doc["cell_nav"]["completed"] = cellNavigator->getCompletedCells();
	doc["cell_nav"]["cell_size_cm"] = GRID_CELL_SIZE_CM;
	doc["cell_nav"]["path_mode"] = cellNavigator->isPathMode();
	doc["cell_nav"]["path_index"] = cellNavigator->getPathIndex();
	doc["cell_nav"]["path_len"] = cellNavigator->getPathLen();
	doc["cell_nav"]["front_distance_mm"] = cellNavigator->getLastFrontDistanceMm();
	doc["cell_nav"]["logger_enabled"] = experimentLogger.isEnabled();
	doc["cell_nav"]["reverse"] = cellNavigator->isCurrentCellReverse();

	const Map::Position gridPos = getRobotGridPositionFromPose();
	doc["location"]["grid_x"] = gridPos.x;
	doc["location"]["grid_y"] = gridPos.y;
	doc["location"]["world_x_m"] = poseEKF->getX();
	doc["location"]["world_y_m"] = poseEKF->getY();
	doc["location"]["theta_rad"] = poseEKF->getTheta();
	doc["location"]["cell_size_m"] = GRID_CELL_SIZE_M;

	// Pose principal: PoseEKF (substitui odometria)
	doc["pose"]["x"] = poseEKF->getX();
	doc["pose"]["y"] = poseEKF->getY();
	doc["pose"]["theta"] = poseEKF->getTheta();
	doc["pose"]["grid_x"] = gridPos.x;
	doc["pose"]["grid_y"] = gridPos.y;
	doc["pose"]["confidence_theta"] = poseEKF->getCovariance_Theta();

	String output;
	output.reserve(768);
	serializeJson(doc, output);
	return output;
}

/**
 * Extrai o primeiro número inteiro positivo encontrado em um comando textual.
 * Exemplo: "vá 3 células para frente" retorna 3.
 */
int extractFirstPositiveInt(const String& text) {
	String digits = "";
	for (size_t i = 0; i < text.length(); ++i) {
		const char c = text[i];
		if (c >= '0' && c <= '9') {
			digits += c;
		} else if (digits.length() > 0) {
			break;
		}
	}
	return digits.length() > 0 ? digits.toInt() : -1;
}

/**
 * Verifica se um comando textual parece pedir deslocamento por células.
 * Aceita variações simples com ou sem acento: celula, célula, celulas, células.
 */
bool isForwardCellsTextCommand(const String& command) {
	const bool hasCellWord = command.indexOf("celula") >= 0 || command.indexOf("célula") >= 0 ||
	                         command.indexOf("celulas") >= 0 || command.indexOf("células") >= 0;
	const bool hasForwardWord = command.indexOf("frente") >= 0 || command.indexOf("forward") >= 0;
	return hasCellWord && hasForwardWord;
}

void handleCar(const String& cmd, AsyncWebSocketClient* client = nullptr) {
	if (cmd.length() > 0 && cmd[0] == '{') {
		StaticJsonDocument<256> request;
		DeserializationError error = deserializeJson(request, cmd);
		if (!error) {
			const char* action = request["action"] | "";
			if (strcmp(action, "forward_cells") == 0 || strcmp(action, "go_cells_forward") == 0) {
				int cells = request["cells"] | -1;
				if (cells <= 0) cells = request["x"] | -1;
				const bool ok = cellNavigator->startForwardCells(cells);

				StaticJsonDocument<192> response;
				response["type"] = "cell_nav";
				response["ok"] = ok;
				response["requested"] = cells;
				response["cell_size_cm"] = GRID_CELL_SIZE_CM;
				if (!ok) response["reason"] = "invalid_or_busy";

				String payload;
				payload.reserve(192);
				serializeJson(response, payload);
				if (client) client->text(payload);
				return;
			} else if (strcmp(action, "path_to") == 0) {
				const long txRaw = request["x"] | -1;
				const long tyRaw = request["y"] | -1;

				if (txRaw < 0 || tyRaw < 0) {
					if (client) client->text("{\"type\":\"path\",\"ok\":false,\"reason\":\"invalid_target\"}");
					return;
				}

				const unsigned int tx = static_cast<unsigned int>(txRaw);
				const unsigned int ty = static_cast<unsigned int>(tyRaw);
				const bool execute = request["execute"] | true;

				bool ok = false;
				size_t outLen = 0;
				if (execute) {
					ok = cellNavigator->startPathTo(tx, ty);
					outLen = cellNavigator->copyPath(plannedPath, PATH_BUFFER_SIZE);
					hasPlannedPath = ok && outLen > 0;
					plannedPathLen = outLen;
				} else {
					if (!robotMap->setTarget(tx, ty)) {
						if (client) client->text("{\"type\":\"path\",\"ok\":false,\"reason\":\"target_out_of_bounds\"}");
						return;
					}
					hasPlannedPath = robotMap->findPathAStar(tx, ty, plannedPath, PATH_BUFFER_SIZE, outLen);
					plannedPathLen = hasPlannedPath ? outLen : 0;
					ok = hasPlannedPath;
				}

				DynamicJsonDocument response(16384);
				response["type"] = "path";
				response["ok"] = ok;
				response["execute"] = execute;
				if (!ok) response["reason"] = "path_not_found_or_busy";

				JsonObject targetObj = response["target"].to<JsonObject>();
				targetObj["x"] = tx;
				targetObj["y"] = ty;

				JsonArray pathJson = response["cells"].to<JsonArray>();
				if (ok) {
					for (size_t i = 0; i < plannedPathLen; ++i) {
						JsonObject step = pathJson.add<JsonObject>();
						step["x"] = plannedPath[i].x;
						step["y"] = plannedPath[i].y;
					}
				}

				String payload;
				payload.reserve(12288);
				serializeJson(response, payload);
				if (client) {
					client->text(payload);
					client->text(getMapSnapshotJson());
				} else if (auto cs = WebInterface::getInstance().getCarSocket()) {
					if (cs->count() > 0) {
						cs->textAll(payload);
						cs->textAll(getMapSnapshotJson());
					}
				}
				return;
			} else if (strcmp(action, "experiment_log") == 0) {
				const char* mode = request["mode"] | "status";
				if (strcmp(mode, "clear") == 0) experimentLogger.clear();
				else if (strcmp(mode, "enable") == 0) experimentLogger.setEnabled(true);
				else if (strcmp(mode, "disable") == 0) experimentLogger.setEnabled(false);
				if (client) {
					StaticJsonDocument<160> response;
					response["type"] = "experiment_log";
					response["enabled"] = experimentLogger.isEnabled();
					response["path"] = experimentLogger.path();
					String payload;
					serializeJson(response, payload);
					client->text(payload);
				}
				return;
			} else if (strcmp(action, "qmc_zero") == 0 || strcmp(action, "mag_zero") == 0) {
				const char* mode = request["mode"] | "current";
				if (strcmp(mode, "clear") == 0) {
					sensorQMC->clearAngleZeroOffset();
					webLog("[QMC5883L] Offset angular zerado manualmente.\n");
				} else if (request.containsKey("x") || request.containsKey("y") || request.containsKey("z")) {
					const float x = request["x"] | 0.0f;
					const float y = request["y"] | 0.0f;
					const float z = request["z"] | 0.0f;
					sensorQMC->setAngleZeroOffset(x, y, z);
					webLog("[QMC5883L] Offset de heading Z customizado aplicado.\n");
				} else {
					sensorQMC->setCurrentAnglesAsZero();
					poseEKF->reset(poseEKF->getX(), poseEKF->getY(), 0.0f, getSignedLeftTicks(), getSignedRightTicks());
					webLog("[QMC5883L] Heading atual calibrado como Z=0.\n");
				}
				if (client) client->text("{\"type\":\"qmc_zero\",\"ok\":true}");
				return;

			} else if (strcmp(action, "qmc_config") == 0 || strcmp(action, "mag_config") == 0) {
				// Configuração simples e de alto nível do QMC5883L.
				// Exemplo: {"action":"qmc_config","declination_deg":-21.5}
				// Exemplo: {"action":"qmc_config","offset_x":120,"offset_y":-80,"offset_z":15}
				if (request.containsKey("declination_deg")) {
					sensorQMC->declinationDeg = request["declination_deg"] | sensorQMC->declinationDeg;
				}

				if (request.containsKey("offset_x") || request.containsKey("offset_y") || request.containsKey("offset_z")) {
					const int16_t ox = request["offset_x"] | sensorQMC->offsetX;
					const int16_t oy = request["offset_y"] | sensorQMC->offsetY;
					const int16_t oz = request["offset_z"] | sensorQMC->offsetZ;
					sensorQMC->setRawOffset(ox, oy, oz);
				}

				StaticJsonDocument<256> response;
				response["type"] = "qmc_config";
				response["ok"] = true;
				response["address"] = sensorQMC->getAddress();
				response["declination_deg"] = sensorQMC->declinationDeg;
				response["offset_x"] = sensorQMC->offsetX;
				response["offset_y"] = sensorQMC->offsetY;
				response["offset_z"] = sensorQMC->offsetZ;
				String payload;
				serializeJson(response, payload);
				if (client) client->text(payload);
				webLog("[QMC5883L] Configuração atualizada.\n");
				return;
			} else if (strcmp(action, "ekf_config") == 0) {
				// Comando para configurar ruído do EKF
				const char* configType = request["type"] | "";

				if (strlen(configType) > 0) {
					// Usar preset
					if (strcmp(configType, "smooth") == 0) {
						poseEKF->setProcessNoise(0.003f, 0.003f, 0.005f);
						poseEKF->setMeasurementNoise(0.01f, 0.02f, 0.005f);
						webLog("[EKF] Configuração SMOOTH aplicada\n");
					} else if (strcmp(configType, "rough") == 0) {
						poseEKF->setProcessNoise(0.015f, 0.015f, 0.03f);
						poseEKF->setMeasurementNoise(0.05f, 0.1f, 0.02f);
						webLog("[EKF] Configuração ROUGH aplicada\n");
					} else if (strcmp(configType, "narrow") == 0) {
						poseEKF->setProcessNoise(0.005f, 0.005f, 0.01f);
						poseEKF->setMeasurementNoise(0.015f, 0.03f, 0.008f);
						webLog("[EKF] Configuração NARROW aplicada\n");
					}
				} else {
					// Usar valores customizados
					float qx = request["q_x"] | 0.005f;
					float qy = request["q_y"] | 0.005f;
					float qtheta = request["q_theta"] | 0.01f;
					float rtheta = request["r_theta"] | 0.02f;
					float rdist = request["r_distance"] | 0.05f;
					float rdrift = request["r_drift"] | 0.01f;

					poseEKF->setProcessNoise(qx, qy, qtheta);
					poseEKF->setMeasurementNoise(rtheta, rdist, rdrift);
					webLog("[EKF] Configuração customizada aplicada\n");
				}

				if (client) {
					client->text("{\"type\":\"ekf\",\"action\":\"config\",\"status\":\"ok\"}");
				}
				return;
			}
		}
	}

	String normalized = cmd;
	normalized.toLowerCase();
	const String& command = normalized;
	if (command == "stop") {
		cellNavigator->cancel();
		ponte->stop();
		webLog("Carro parado.\n");
	} else if (isForwardCellsTextCommand(command)) {
		const int cells = extractFirstPositiveInt(command);
		const bool ok = cellNavigator->startForwardCells(cells);
		if (client) {
			client->text(ok ? "{\"type\":\"cell_nav\",\"ok\":true}" : "{\"type\":\"cell_nav\",\"ok\":false,\"reason\":\"invalid_or_busy\"}");
		}
		webLog(ok ? "Comando de células iniciado.\n" : "Falha ao iniciar comando de células.\n");
	} else if (command == "right") {
		ponte->turnRight();
		syncEncoderReferenceForEKF();
		webLog("Carro virando para a direita.\n");
	} else if (command == "left") {
		ponte->turnLeft();
		syncEncoderReferenceForEKF();
		webLog("Carro virando para a esquerda.\n");
	} else if (command == "forward") {
		ponte->forward();
		syncEncoderReferenceForEKF();
		webLog("Carro indo para frente.\n");
	} else if (command == "backward") {
		ponte->backward();
		syncEncoderReferenceForEKF();
		webLog("Carro indo para trás.\n");
	} else if (command == "status") {
		String status = getCarStatus();
		if (auto cs = WebInterface::getInstance().getCarSocket()) {
			if (cs->count() > 0) {
				if (client) {
					client->text(status);
				} else {
					cs->textAll(status);
				}
			}
		}
	} else if (command == "log_clear") {
		experimentLogger.clear();
		webLog("Log experimental CSV limpo.\n");
	} else if (command == "log_enable") {
		experimentLogger.setEnabled(true);
		webLog("Log experimental CSV ativado.\n");
	} else if (command == "log_disable") {
		experimentLogger.setEnabled(false);
		webLog("Log experimental CSV desativado.\n");
	} else if (command == "qmc_zero" || command == "mag_zero") {
		sensorQMC->setCurrentAnglesAsZero();
		poseEKF->reset(poseEKF->getX(), poseEKF->getY(), 0.0f, getSignedLeftTicks(), getSignedRightTicks());
		webLog("[QMC5883L] Heading atual calibrado como Z=0.\n");
	} else if (command == "qmc_scan" || command == "i2c_scan") {
		if (i2cBusMutex && xSemaphoreTake(i2cBusMutex, pdMS_TO_TICKS(300)) == pdTRUE) {
			sensorQMC->scanI2C(Serial);
			xSemaphoreGive(i2cBusMutex);
			webLog("[QMC5883L] Scan I2C enviado ao Serial.\n");
		} else {
			webLog("[QMC5883L] Não foi possível obter o barramento I2C para scan.\n");
		}
	} else if (command == "map") {
		String mapPayload = getMapSnapshotJson();
		if (client) {
			client->text(mapPayload);
		} else if (auto cs = WebInterface::getInstance().getCarSocket()) {
			if (cs->count() > 0) cs->textAll(mapPayload);
		}
	} else if (command == "ekf_status") {
		// Retorna status do EKF
		StaticJsonDocument<512> doc;
		doc["type"] = "ekf_status";
		doc["x"] = poseEKF->getX();
		doc["y"] = poseEKF->getY();
		doc["theta"] = poseEKF->getTheta();
		const Map::Position gridPos = getRobotGridPositionFromPose();
		doc["grid_x"] = gridPos.x;
		doc["grid_y"] = gridPos.y;
		doc["cell_size_m"] = GRID_CELL_SIZE_M;
		doc["cov_x"] = poseEKF->getCovariance_X();
		doc["cov_y"] = poseEKF->getCovariance_Y();
		doc["cov_theta"] = poseEKF->getCovariance_Theta();

		String payload;
		payload.reserve(512);
		serializeJson(doc, payload);
		if (client) {
			client->text(payload);
		} else if (auto cs = WebInterface::getInstance().getCarSocket()) {
			if (cs->count() > 0) cs->textAll(payload);
		}
	} else if (command == "ekf_reset") {
		// Reseta EKF para origem e sincroniza os contadores assinados.
		lastRawLeftTicksForEKF = encoderE->getTotalTicks();
		lastRawRightTicksForEKF = encoderD->getTotalTicks();
		ekfSignedLeftTicks = 0;
		ekfSignedRightTicks = 0;
		poseEKF->reset(0.0f, 0.0f, 0.0f,
		               getSignedLeftTicks(),
		               getSignedRightTicks());
		updateRobotMapPositionFromPose();
		webLog("EKF resetado para origem contínua (0,0,0), que corresponde à célula (1,1) no mapa 7x7.\n");
		if (auto cs = WebInterface::getInstance().getCarSocket()) {
			if (cs->count() > 0) cs->textAll("{\"type\":\"ekf\",\"action\":\"reset\",\"status\":\"ok\"}");
		}
	} else {
		webLog("Comando desconhecido: " + command + "\n");
	}
}
void handleCommand(const String& cmd) {
	String normalized = cmd;
	normalized.toLowerCase();
	const String& command = normalized;
	if (command == "led") {
		led_carro.toggle();
		webLog("LED Alterado, agora está: " + String(led_carro.isOn() ? "ON" : "OFF") + "\n");
	} else if (command == "reboot" || command == "restart") {
		webLog("Reiniciando o ESP...\n");
		delay(100);
		Serial.println("Reiniciando o ESP...\n");
		ESP.restart();
	} else if (command == "ip") {
		String ip = WiFi.localIP().toString();
		webLog("IP atual: " + ip + "\n");
	} else if (command == "status") {
		String status = getCarStatus();
		webLog("Status do carro: " + status + "\n");
	} else if (command == "brownout" || command == "brownout_status") {
		webLog("Brownout status: " + BrownoutLogger::getInstance().toJson() + "\n");
	} else if (command == "brownout_clear") {
		BrownoutLogger::getInstance().clear();
		webLog("Histórico de brownout limpo.\n");
	} else {
		webLog("Comando desconhecido: " + command + "\n");
	}
}
void onWebSocketEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
	const char* sockName = "Unknown Socket";
	AsyncWebSocket* carSock = WebInterface::getInstance().getCarSocket();
	AsyncWebSocket* webSock = WebInterface::getInstance().getWebLogSocket();
	if (server == carSock) {
		sockName = "Car Socket";
	} else if (server == webSock) {
		sockName = "WebSocket";
	}
	switch (type) {
		case WS_EVT_CONNECT:
			Serial.printf("[%s] WebSocket client #%u connected from %s\n", sockName, client->id(), client->remoteIP().toString().c_str());
			webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Conectado de " + client->remoteIP().toString() + "\n");
			if (server == carSock) {
				Serial.printf("WebSocket [server #%s] client #%u is ready\n", sockName, client->id());
				client->text("{\"ready\":\"true\"}");
				client->text(getMapSnapshotJson());
			}
			break;
		case WS_EVT_DISCONNECT:
			if (server == carSock) {
				if (server->count() == 0) {
					ponte->stop();
				}
			}
			webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Desconectado\n");
			Serial.printf("WebSocket [server #%s] client #%u disconnected\n", sockName, client->id());
			break;
		case WS_EVT_DATA:
			AwsFrameInfo *info;
			info = (AwsFrameInfo*)arg;
			if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
				String command((const char*)data, len);
				Serial.printf("Comando recebido do cliente #%u: %s\n", client->id(), command.c_str());
				if (server == carSock) {
					handleCar(command, client);
					// webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Comando recebido: " + command + "\n");
				} else if (server == webSock) {
					webLog("["+ String(sockName) + "] Cliente ["+ String(client->id()) +"]: Comando recebido: " + command + "\n");
					handleCommand(command);
				} else {
					webLog("["+ String(sockName) + "] Comando desconhecido: " + command + "\n");
				}
			}
			break;
		case WS_EVT_PONG:
		case WS_EVT_ERROR:
			break;
	}
}

void prepare_http_server() {
	auto &wi = WebInterface::getInstance();
	AsyncWebServer* server = wi.getServer();
	AsyncWebSocket* ws = wi.getWebLogSocket();
	AsyncWebSocket* carSocket = wi.getCarSocket();

	if (!server) {
		webLog("prepare_http_server: servidor HTTP não iniciado. Chame WebInterface::getInstance().begin() primeiro.");
		return;
	}

	server->on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (!SPIFFS.exists("/dashboard.html")) {
			request->send(404, "text/plain", "dashboard.html não encontrado");
			webLog("dashboard.html não encontrado\n");
			return;
		}
		request->send(SPIFFS, "/dashboard.html", String(), false);
	});
	server->on("/monitor", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (!SPIFFS.exists("/monitor.html")) {
			request->send(404, "text/plain", "monitor.html não encontrado");
			webLog("monitor.html não encontrado\n");
			return;
		}
		request->send(SPIFFS, "/monitor.html", String(), false);
	});
	server->on("/brownout", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (!SPIFFS.exists("/brownout.html")) {
			request->send(404, "text/plain", "brownout.html não encontrado");
			webLog("brownout.html não encontrado\n");
			return;
		}
		request->send(SPIFFS, "/brownout.html", String(), false);
	});
	server->on("/api/brownout-logs", HTTP_GET, [](AsyncWebServerRequest *request) {
		request->send(200, "application/json", BrownoutLogger::getInstance().toJson());
	});
	server->on("/api/brownout-logs/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
		BrownoutLogger::getInstance().clear();
		webLog("[BrownoutLogger] Histórico de brownout limpo pela Web UI.\n");
		request->send(200, "application/json", BrownoutLogger::getInstance().toJson());
	});
	server->on("/api/tcc-log.csv", HTTP_GET, [](AsyncWebServerRequest *request) {
		if (!SPIFFS.exists(experimentLogger.path())) {
			request->send(404, "text/plain", "tcc_experiment_log.csv não encontrado");
			return;
		}
		request->send(SPIFFS, experimentLogger.path(), "text/csv", true);
	});
	server->on("/api/tcc-log/clear", HTTP_POST, [](AsyncWebServerRequest *request) {
		experimentLogger.clear();
		request->send(200, "application/json", "{\"ok\":true}");
	});
	server->onNotFound([](AsyncWebServerRequest *request) {
		webLog("Requisição não encontrada: " + request->url());
		request->send(404, "text/plain", "Not Found");
	});

	// WebSocket event handlers are configured inside WebInterface::begin()
	if (ws) server->addHandler(ws);
	if (carSocket) server->addHandler(carSocket);

	#ifdef DEBUG_PRINTS
		Serial.println("Servidor HTTP preparado");
	#endif
}

void setup() {
	Serial.begin(115200);

	#ifdef DEBUG_PRINTS
		Serial.println("Iniciando...");
	#endif

	BrownoutLogger::getInstance().begin();
	led_carro.setup();

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
	experimentLogger.begin(true, 250);

	ConnectToWiFi();

	// Inicializa webinterface (cria server e websockets)
	WebInterface::getInstance().begin();

	// Registra handlers existentes para os websockets
	WebInterface::getInstance().setCarCommandHandler(handleCar);
	WebInterface::getInstance().setWebCommandHandler([](const String& cmd, AsyncWebSocketClient* client){ handleCommand(cmd); });

	prepare_http_server();

	ponte->setup();
	// Mapa oficial do TCC: 7x7, bordas ocupadas e robô iniciando na célula livre (1,1).
	initializeTccBinaryMap7x7();
	// A pose contínua (0,0,0) corresponde à célula discreta (1,1), definida por MAP_ORIGIN_X/Y.
	lastRawLeftTicksForEKF = encoderE->getTotalTicks();
	lastRawRightTicksForEKF = encoderD->getTotalTicks();
	ekfSignedLeftTicks = 0;
	ekfSignedRightTicks = 0;
	poseEKF->initialize(0.0f, 0.0f, 0.0f, getSignedLeftTicks(), getSignedRightTicks());
	updateRobotMapPositionFromPose();

	Wire.begin(21, 22);
	Wire.setClock(100000);
	sensorSnapshotMutex = xSemaphoreCreateMutex();
	i2cBusMutex = xSemaphoreCreateMutex();
	sensor->setup();
	if (sensorQMC->setup()) {
		Serial.print("[QMC5883L] Sensor inicializado no endereco 0x");
		if (sensorQMC->getAddress() < 16) Serial.print("0");
		Serial.print(sensorQMC->getAddress(), HEX);
		Serial.print(" | Chip ID: 0x");
		if (sensorQMC->getChipId() < 16) Serial.print("0");
		Serial.println(sensorQMC->getChipId(), HEX);
	} else {
		Serial.println("[QMC5883L] ERRO: sensor nao encontrado. Use qmc_scan para verificar o I2C.");
	}
	sensorQMC->setCurrentAnglesAsZero(); // orientação inicial do robô passa a ser (0,0,0)
	startSensorTasks();

	#ifdef DEBUG_PRINTS
		Serial.println("Setup completo!");
	#endif

}

/**
 * Indica se existe movimento físico comandado no momento.
 *
 * O critério principal é a PonteH: se ela está parada, os motores não devem
 * gerar odometria. O navegador pode continuar ativo durante pequenos estados
 * de espera/correção, mas a pose só deve integrar sensores quando a PonteH
 * realmente estiver mandando os motores andarem ou girarem.
 */
bool isRobotPhysicallyCommanded() {
	return ponte && !ponte->isStopped();
}


unsigned long lastLoopTime = 0;
const unsigned long loopInterval = 20;  // ms

void loop() {
	if (WebInterface::getInstance().getWebLogSocket())
		WebInterface::getInstance().getWebLogSocket()->cleanupClients();
	if (WebInterface::getInstance().getCarSocket())
		WebInterface::getInstance().getCarSocket()->cleanupClients();

	if (WiFi.status() != WL_CONNECTED) {
		#ifdef DEBUG_PRINTS
			Serial.println("WiFi desconectado, tentando reconectar...");
		#endif
		ponte->stop();
		ConnectToWiFi(5000);
		return;
	}

	unsigned long now = millis();
	if (now - lastLoopTime >= loopInterval) {
		lastLoopTime = now;
		float deltaTime_s = loopInterval / 1000.0f;

		updateSignedEncoderTicks();
		const long leftTicks = getSignedLeftTicks();
		const long rightTicks = getSignedRightTicks();
		const bool commandedMotion = isRobotPhysicallyCommanded();

		if (commandedMotion) {
			// Atualiza PoseEKF somente quando há movimento comandado.
			// Durante giro, a orientação vem da odometria diferencial dos encoders.
			// O QMC5883L é magnetômetro e sofre interferência dos motores/L298N,
			// então NÃO deve corrigir theta enquanto o robô está virando.
			poseEKF->predict(leftTicks, rightTicks, deltaTime_s);
			if (ponte->getCurrentMove() == MOVEMENT_FORWARD || ponte->getCurrentMove() == MOVEMENT_BACKWARDS) {
				const float magHeading = getCachedMagAngleZ();
				const float innovationDeg = fabsf(normalizeSignedAngleGlobal(magHeading - poseEKF->getTheta())) * (180.0f / M_PI);
				if (innovationDeg <= QMC_HEADING_MAX_INNOVATION_DEG) {
					poseEKF->updateWithHeading(magHeading);
				} else {
					static unsigned long lastQmcRejectLog = 0;
					if (millis() - lastQmcRejectLog > 500) {
						lastQmcRejectLog = millis();
						webLog("[QMC5883L] Heading ignorado no EKF por salto: innovation=" + String(innovationDeg, 2) + " deg\n");
					}
				}
			}
		} else {
			// Robô parado: sincroniza a referência dos sensores sem alterar x/y/theta.
			// Assim, qualquer pulso espúrio enquanto parado não vira deslocamento acumulado
			// quando o próximo comando começar.
			poseEKF->holdCurrentPose(leftTicks, rightTicks);
			cellNavigator->resetIdleIfTerminal();
		}

		// Atualiza posição discreta enviada para o mapa/WebServer apenas quando
		// não há navegação em célula ativa. Durante uma célula, a posição oficial
		// do mapa permanece travada e só é fixada no fim da célula esperada.
		if (!cellNavigator || !cellNavigator->isActive()) {
			updateRobotMapPositionFromPose();
		}

		ponte->loop(getCachedFrontDistanceMm(), getCachedGyroZ());
		cellNavigator->loop();
	}
}
