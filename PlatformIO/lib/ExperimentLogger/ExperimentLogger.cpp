#include "ExperimentLogger.h"

ExperimentLogger::ExperimentLogger(const char* path)
    : _path(path), _enabled(false), _lastPeriodicLogMs(0), _periodMs(250) {}

bool ExperimentLogger::begin(bool enabled, unsigned long periodMs) {
    _enabled = enabled;
    _periodMs = periodMs > 0 ? periodMs : 250;
    _lastPeriodicLogMs = millis();
    if (!_enabled) {
        return true;
    }
    ensureHeader();
    return SPIFFS.exists(_path);
}

void ExperimentLogger::setEnabled(bool enabled) {
    _enabled = enabled;
    if (_enabled) {
        ensureHeader();
    }
}

bool ExperimentLogger::isEnabled() const {
    return _enabled;
}

bool ExperimentLogger::clear() {
    if (SPIFFS.exists(_path)) {
        SPIFFS.remove(_path);
    }
    ensureHeader();
    return SPIFFS.exists(_path);
}

const char* ExperimentLogger::path() const {
    return _path;
}

bool ExperimentLogger::shouldLogPeriodic() {
    if (!_enabled) {
        return false;
    }

    const unsigned long now = millis();
    if (now - _lastPeriodicLogMs < _periodMs) {
        return false;
    }

    _lastPeriodicLogMs = now;
    return true;
}

String ExperimentLogger::sanitize(const String& value) const {
    String output = value;
    output.replace(";", ",");
    output.replace("\r", " ");
    output.replace("\n", " ");
    return output;
}

void ExperimentLogger::ensureHeader() {
    if (SPIFFS.exists(_path)) {
        return;
    }

    File file = SPIFFS.open(_path, FILE_WRITE);
    if (!file) {
        return;
    }

    file.println("millis;event;state;requested_cells;completed_cells;grid_x;grid_y;pose_x_m;pose_y_m;theta_rad;target_heading_rad;heading_error_rad;travelled_cell_m;front_distance_mm;obstacle_detected;path_mode;path_index;path_len;replan_triggered;note");
    file.close();
}

void ExperimentLogger::log(
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
) {
    if (!_enabled) {
        return;
    }

    ensureHeader();

    File file = SPIFFS.open(_path, FILE_APPEND);
    if (!file) {
        return;
    }

    file.print(millis()); file.print(';');
    file.print(sanitize(eventName)); file.print(';');
    file.print(sanitize(stateName)); file.print(';');
    file.print(requestedCells); file.print(';');
    file.print(completedCells); file.print(';');
    file.print(gridX); file.print(';');
    file.print(gridY); file.print(';');
    file.print(poseX, 4); file.print(';');
    file.print(poseY, 4); file.print(';');
    file.print(thetaRad, 5); file.print(';');
    file.print(targetHeadingRad, 5); file.print(';');
    file.print(headingErrorRad, 5); file.print(';');
    file.print(travelledInCellM, 4); file.print(';');
    file.print(distanceMm); file.print(';');
    file.print(obstacleDetected ? 1 : 0); file.print(';');
    file.print(pathMode ? 1 : 0); file.print(';');
    file.print(static_cast<unsigned long>(pathIndex)); file.print(';');
    file.print(static_cast<unsigned long>(pathLen)); file.print(';');
    file.print(replanTriggered ? 1 : 0); file.print(';');
    file.println(sanitize(note));
    file.close();
}
