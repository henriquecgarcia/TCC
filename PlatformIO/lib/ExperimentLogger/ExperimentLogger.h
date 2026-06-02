#pragma once

#include <Arduino.h>
#include <SPIFFS.h>

/**
 * Registra dados experimentais do TCC em CSV dentro da SPIFFS.
 *
 * O objetivo desta classe é transformar cada teste do robô em evidência
 * mensurável para a monografia: distância, pose estimada, erro angular,
 * célula atual, evento da navegação, leituras de sensor e estado do mapa.
 */
class ExperimentLogger {
private:
    const char* _path;
    bool _enabled;
    unsigned long _lastPeriodicLogMs;
    unsigned long _periodMs;

    /** Cria o cabeçalho CSV quando o arquivo ainda não existe. */
    void ensureHeader();

    /** Escapa separadores simples para manter uma linha CSV válida. */
    String sanitize(const String& value) const;

public:
    explicit ExperimentLogger(const char* path = "/tcc_experiment_log.csv");

    /** Inicializa o logger após a SPIFFS estar montada. */
    bool begin(bool enabled = true, unsigned long periodMs = 250);

    /** Liga ou desliga a gravação para evitar desgaste de flash durante testes informais. */
    void setEnabled(bool enabled);

    /** Retorna se o logger está gravando no momento. */
    bool isEnabled() const;

    /** Limpa o arquivo CSV e recria o cabeçalho. */
    bool clear();

    /** Retorna o caminho do arquivo CSV dentro da SPIFFS. */
    const char* path() const;

    /** Indica se já passou o intervalo configurado para log periódico. */
    bool shouldLogPeriodic();

    /**
     * Adiciona uma linha ao CSV.
     *
     * Os campos foram mantidos genéricos para permitir logging de navegação por
     * células, execução de caminho A* e replanejamento dinâmico.
     */
    void log(
        const String& eventName,
        const String& stateName,
        int requestedCells,
        int completedCells,
        int gridX,
        int gridY,
        float poseX,
        float poseY,
        float thetaRad,
        float targetHeadingRad,
        float headingErrorRad,
        float travelledInCellM,
        int distanceMm,
        bool obstacleDetected,
        bool pathMode,
        size_t pathIndex,
        size_t pathLen,
        bool replanTriggered,
        const String& note
    );
};
