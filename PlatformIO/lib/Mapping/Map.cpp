#include "Map.h"

#include <math.h>
#include <new>

Map::Map(unsigned int m, unsigned int n)
    : _m(0), _n(0), _posX(0), _posY(0), _targetX(0), _targetY(0), _hasTarget(false), _data(nullptr), _spiffsReady(false) {
    allocateGrid(m, n);
}

Map::Map()
    : _m(0), _n(0), _posX(0), _posY(0), _targetX(0), _targetY(0), _hasTarget(false), _data(nullptr), _spiffsReady(false) {}

Map::~Map() {
    delete[] _data;
    _data = nullptr;
}

bool Map::begin(bool formatOnFail) {
    if (_spiffsReady) {
        return true;
    }

    _spiffsReady = SPIFFS.begin(formatOnFail);
    return _spiffsReady;
}

bool Map::resize(unsigned int m, unsigned int n) {
    return allocateGrid(m, n);
}

bool Map::resizeFromRealDimensions(float realWidth, float realHeight, float cellWidth, float cellHeight) {
    unsigned int calculatedM = 0;
    unsigned int calculatedN = 0;

    if (!calculateGridSizeFromRealDimensions(
            realWidth,
            realHeight,
            cellWidth,
            cellHeight,
            calculatedM,
            calculatedN)) {
        return false;
    }

    return allocateGrid(calculatedM, calculatedN);
}

bool Map::calculateGridSizeFromRealDimensions(
    float realWidth,
    float realHeight,
    float cellWidth,
    float cellHeight,
    unsigned int& outM,
    unsigned int& outN
) {
    if (realWidth <= 0.0f || realHeight <= 0.0f || cellWidth <= 0.0f || cellHeight <= 0.0f) {
        return false;
    }

    const float mFloat = ceilf(realWidth / cellWidth);
    const float nFloat = ceilf(realHeight / cellHeight);

    if (mFloat <= 0.0f || nFloat <= 0.0f) {
        return false;
    }

    if (mFloat > 4294967295.0f || nFloat > 4294967295.0f) {
        return false;
    }

    outM = static_cast<unsigned int>(mFloat);
    outN = static_cast<unsigned int>(nFloat);
    return true;
}

bool Map::setCell(unsigned int x, unsigned int y, uint8_t value) {
    if (!_data || !isInBounds(x, y)) {
        return false;
    }

    const size_t idx = byteIndex(x, y);
    const uint8_t mask = bitMask(x, y);

    if (value) {
        _data[idx] |= mask;
    } else {
        _data[idx] &= static_cast<uint8_t>(~mask);
    }

    return true;
}

uint8_t Map::getCell(unsigned int x, unsigned int y) const {
    if (!_data || !isInBounds(x, y)) {
        return 1;
    }

    const size_t idx = byteIndex(x, y);
    const uint8_t mask = bitMask(x, y);

    return (_data[idx] & mask) ? 1 : 0;
}

bool Map::setPosition(unsigned int x, unsigned int y) {
    if (!isInBounds(x, y)) {
        return false;
    }

    _posX = x;
    _posY = y;
    return true;
}

Map::Position Map::getPosition() const {
    Position p;
    p.x = _posX;
    p.y = _posY;
    return p;
}

bool Map::setTarget(unsigned int x, unsigned int y) {
    if (!isInBounds(x, y)) {
        return false;
    }

    _targetX = x;
    _targetY = y;
    _hasTarget = true;
    return true;
}

bool Map::clearTarget() {
    _hasTarget = false;
    _targetX = 0;
    _targetY = 0;
    return true;
}

bool Map::hasTarget() const {
    return _hasTarget;
}

Map::Position Map::getTarget() const {
    Position p;
    p.x = _targetX;
    p.y = _targetY;
    return p;
}

bool Map::isOccupied(unsigned int x, unsigned int y) const {
    return getCell(x, y) == 1;
}

bool Map::generateStraightLineTest(unsigned int freeRowY) {
    if (!_data || _m == 0 || _n == 0 || freeRowY >= _n) {
        return false;
    }

    for (unsigned int y = 0; y < _n; ++y) {
        for (unsigned int x = 0; x < _m; ++x) {
            const uint8_t occupied = (y == freeRowY) ? 0 : 1;
            setCell(x, y, occupied);
        }
    }

    setPosition(0, freeRowY);
    setTarget(_m - 1U, freeRowY);
    return true;
}

bool Map::findPathAStar(
    unsigned int targetX,
    unsigned int targetY,
    Position* outPath,
    size_t maxPathLen,
    size_t& outPathLen
) const {
    outPathLen = 0;

    if (!_data || !outPath || maxPathLen == 0) {
        return false;
    }

    if (!isInBounds(_posX, _posY) || !isInBounds(targetX, targetY)) {
        return false;
    }

    if (isOccupied(_posX, _posY) || isOccupied(targetX, targetY)) {
        return false;
    }

    const size_t totalCells = static_cast<size_t>(_m) * static_cast<size_t>(_n);
    if (totalCells == 0) {
        return false;
    }

    const size_t startIndex = linearIndex(_posX, _posY);
    const size_t goalIndex = linearIndex(targetX, targetY);

    if (startIndex == goalIndex) {
        outPath[0] = getPosition();
        outPathLen = 1;
        return true;
    }

    // Track arrival direction as part of the state so turns are optimized first.
    const size_t stateCount = totalCells * 4U;
    const uint16_t STATE_NONE = 0xFFFFU;
    const uint16_t TURN_INF = 0xFFFFU;
    const int8_t dirX[4] = {1, -1, 0, 0};
    const int8_t dirY[4] = {0, 0, 1, -1};

    if (stateCount == 0 || stateCount > STATE_NONE) {
        return false;
    }

    const auto makeState = [&](size_t cellIndex, uint8_t dir) -> uint16_t {
        return static_cast<uint16_t>((cellIndex * 4U) + static_cast<size_t>(dir));
    };

    const auto stateCell = [&](uint16_t state) -> size_t {
        return static_cast<size_t>(state) / 4U;
    };

    const auto stateDir = [&](uint16_t state) -> uint8_t {
        return static_cast<uint8_t>(state % 4U);
    };

    const size_t nextLayerBytes = (stateCount + 7U) / 8U;
    uint16_t* turnScore = new (std::nothrow) uint16_t[stateCount];
    uint16_t* queue = new (std::nothrow) uint16_t[stateCount];
    uint8_t* nextLayer = new (std::nothrow) uint8_t[nextLayerBytes];

    if (!turnScore || !queue || !nextLayer) {
        delete[] turnScore;
        delete[] queue;
        delete[] nextLayer;
        return false;
    }

    for (size_t i = 0; i < stateCount; ++i) {
        turnScore[i] = TURN_INF;
    }
    memset(nextLayer, 0, nextLayerBytes);

    size_t queueHead = 0;
    size_t queueTail = 0;

    const auto queuePush = [&](uint16_t state) -> bool {
        if (queueTail >= stateCount) {
            return false;
        }
        queue[queueTail++] = state;
        return true;
    };

    const auto nextLayerSet = [&](uint16_t state) {
        nextLayer[state / 8U] |= static_cast<uint8_t>(1U << (state % 8U));
    };

    const auto nextLayerGet = [&](size_t state) -> bool {
        return (nextLayer[state / 8U] & (1U << (state % 8U))) != 0;
    };

    const auto tryVisit = [&](const Position& c, uint8_t nextDir, uint16_t nextTurns, uint16_t activeTurns) -> bool {
        const int nx = static_cast<int>(c.x) + dirX[nextDir];
        const int ny = static_cast<int>(c.y) + dirY[nextDir];

        if (nx < 0 || ny < 0) {
            return true;
        }

        const unsigned int ux = static_cast<unsigned int>(nx);
        const unsigned int uy = static_cast<unsigned int>(ny);
        if (!isInBounds(ux, uy) || isOccupied(ux, uy)) {
            return true;
        }

        const uint16_t nextState = makeState(linearIndex(ux, uy), nextDir);
        if (nextTurns >= turnScore[nextState]) {
            return true;
        }

        turnScore[nextState] = nextTurns;
        if (nextTurns == activeTurns) {
            return queuePush(nextState);
        }

        nextLayerSet(nextState);
        return true;
    };

    bool hasSeed = false;
    const Position startPos = getPosition();
    for (uint8_t dir = 0; dir < 4U; ++dir) {
        const int nx = static_cast<int>(startPos.x) + dirX[dir];
        const int ny = static_cast<int>(startPos.y) + dirY[dir];

        if (nx < 0 || ny < 0) {
            continue;
        }

        const unsigned int ux = static_cast<unsigned int>(nx);
        const unsigned int uy = static_cast<unsigned int>(ny);
        if (!isInBounds(ux, uy) || isOccupied(ux, uy)) {
            continue;
        }

        const uint16_t state = makeState(linearIndex(ux, uy), dir);
        turnScore[state] = 0;
        if (!queuePush(state)) {
            delete[] turnScore;
            delete[] queue;
            delete[] nextLayer;
            return false;
        }
        hasSeed = true;
    }

    if (!hasSeed) {
        delete[] turnScore;
        delete[] queue;
        delete[] nextLayer;
        return false;
    }

    uint16_t currentTurns = 0;
    uint16_t finalState = STATE_NONE;

    while (true) {
        while (queueHead < queueTail) {
            const uint16_t current = queue[queueHead++];
            if (turnScore[current] != currentTurns) {
                continue;
            }

            const size_t currentCell = stateCell(current);
            if (currentCell == goalIndex) {
                finalState = current;
                break;
            }

            const Position c = positionFromIndex(currentCell);
            const uint8_t currentDir = stateDir(current);

            if (!tryVisit(c, currentDir, currentTurns, currentTurns)) {
                delete[] turnScore;
                delete[] queue;
                delete[] nextLayer;
                return false;
            }

            if (currentTurns < static_cast<uint16_t>(TURN_INF - 1U)) {
                const uint16_t nextTurns = static_cast<uint16_t>(currentTurns + 1U);
                for (uint8_t nextDir = 0; nextDir < 4U; ++nextDir) {
                    if (nextDir == currentDir) {
                        continue;
                    }

                    if (!tryVisit(c, nextDir, nextTurns, currentTurns)) {
                        delete[] turnScore;
                        delete[] queue;
                        delete[] nextLayer;
                        return false;
                    }
                }
            }
        }

        if (finalState != STATE_NONE) {
            break;
        }

        if (currentTurns >= static_cast<uint16_t>(TURN_INF - 1U)) {
            delete[] turnScore;
            delete[] queue;
            delete[] nextLayer;
            return false;
        }

        currentTurns = static_cast<uint16_t>(currentTurns + 1U);
        queueHead = 0;
        queueTail = 0;

        for (size_t state = 0; state < stateCount; ++state) {
            if (nextLayerGet(state) && turnScore[state] == currentTurns) {
                if (!queuePush(static_cast<uint16_t>(state))) {
                    delete[] turnScore;
                    delete[] queue;
                    delete[] nextLayer;
                    return false;
                }
            }
        }

        memset(nextLayer, 0, nextLayerBytes);

        if (queueTail == 0) {
            delete[] turnScore;
            delete[] queue;
            delete[] nextLayer;
            return false;
        }
    }

    const auto findPreviousState = [&](uint16_t state, uint16_t& previousState, bool& reachedStart) -> bool {
        reachedStart = false;
        previousState = STATE_NONE;

        const Position p = positionFromIndex(stateCell(state));
        const uint8_t dir = stateDir(state);
        const int px = static_cast<int>(p.x) - dirX[dir];
        const int py = static_cast<int>(p.y) - dirY[dir];

        if (px < 0 || py < 0) {
            return false;
        }

        const unsigned int ux = static_cast<unsigned int>(px);
        const unsigned int uy = static_cast<unsigned int>(py);
        if (!isInBounds(ux, uy)) {
            return false;
        }

        const size_t previousCell = linearIndex(ux, uy);
        const uint16_t turnsHere = turnScore[state];

        if (previousCell == startIndex && turnsHere == 0) {
            reachedStart = true;
            return true;
        }

        for (uint8_t pass = 0; pass < 5U; ++pass) {
            const uint8_t previousDir = (pass == 0U) ? dir : static_cast<uint8_t>(pass - 1U);
            if (pass > 0U && previousDir == dir) {
                continue;
            }

            const uint16_t candidate = makeState(previousCell, previousDir);
            const uint16_t turnsBefore = turnScore[candidate];
            if (turnsBefore == TURN_INF) {
                continue;
            }

            const uint16_t penalty = (previousDir == dir) ? 0U : 1U;
            if (turnsBefore <= static_cast<uint16_t>(TURN_INF - penalty) &&
                static_cast<uint16_t>(turnsBefore + penalty) == turnsHere) {
                previousState = candidate;
                return true;
            }
        }

        return false;
    };

    size_t pathLen = 1;
    uint16_t walkState = finalState;
    while (true) {
        ++pathLen;
        if (pathLen > maxPathLen || pathLen > totalCells + 1U) {
            delete[] turnScore;
            delete[] queue;
            delete[] nextLayer;
            return false;
        }

        bool reachedStart = false;
        uint16_t previousState = STATE_NONE;
        if (!findPreviousState(walkState, previousState, reachedStart)) {
            delete[] turnScore;
            delete[] queue;
            delete[] nextLayer;
            return false;
        }

        if (reachedStart) {
            break;
        }

        walkState = previousState;
    }

    outPath[0] = startPos;
    walkState = finalState;
    for (size_t writePos = pathLen; writePos > 1U; --writePos) {
        outPath[writePos - 1U] = positionFromIndex(stateCell(walkState));

        bool reachedStart = false;
        uint16_t previousState = STATE_NONE;
        if (!findPreviousState(walkState, previousState, reachedStart)) {
            delete[] turnScore;
            delete[] queue;
            delete[] nextLayer;
            return false;
        }

        if (reachedStart) {
            break;
        }

        walkState = previousState;
    }

    outPathLen = pathLen;

    delete[] turnScore;
    delete[] queue;
    delete[] nextLayer;
    return true;
}

bool Map::loadFromFile(const char* path) {
    if (!_spiffsReady || !_data || !path) {
        return false;
    }

    File file = SPIFFS.open(path, FILE_READ);
    if (!file) {
        return false;
    }

    FileHeader header;
    const size_t headerSize = sizeof(FileHeader);

    if (file.read(reinterpret_cast<uint8_t*>(&header), headerSize) != headerSize) {
        file.close();
        return false;
    }

    const size_t expectedSize = getDataSizeBytes();

    if (header.magic != FILE_MAGIC ||
        header.version != FILE_VERSION ||
        header.width != _m ||
        header.height != _n ||
        header.dataSize != expectedSize) {
        file.close();
        return false;
    }

    if (file.read(_data, expectedSize) != expectedSize) {
        file.close();
        return false;
    }

    file.close();

    if (!isInBounds(header.posX, header.posY)) {
        _posX = 0;
        _posY = 0;
    } else {
        _posX = header.posX;
        _posY = header.posY;
    }

    return true;
}

bool Map::saveToFile(const char* path) const {
    if (!_spiffsReady || !_data || !path) {
        return false;
    }

    File file = SPIFFS.open(path, FILE_WRITE);
    if (!file) {
        return false;
    }

    FileHeader header;
    header.magic = FILE_MAGIC;
    header.version = FILE_VERSION;
    header.reserved = 0;
    header.width = _m;
    header.height = _n;
    header.posX = _posX;
    header.posY = _posY;
    header.dataSize = getDataSizeBytes();

    const size_t headerSize = sizeof(FileHeader);
    const size_t dataSize = getDataSizeBytes();

    if (file.write(reinterpret_cast<const uint8_t*>(&header), headerSize) != headerSize) {
        file.close();
        return false;
    }

    if (file.write(_data, dataSize) != dataSize) {
        file.close();
        return false;
    }

    file.close();
    return true;
}

unsigned int Map::getWidth() const {
    return _m;
}

unsigned int Map::getHeight() const {
    return _n;
}

size_t Map::getDataSizeBytes() const {
    const size_t totalBits = static_cast<size_t>(_m) * static_cast<size_t>(_n);
    return (totalBits + 7U) / 8U;
}

bool Map::allocateGrid(unsigned int m, unsigned int n) {
    if (m == 0 || n == 0) {
        return false;
    }

    const size_t totalCells = static_cast<size_t>(m) * static_cast<size_t>(n);
    if (totalCells == 0) {
        return false;
    }

    const size_t bytes = (totalCells + 7U) / 8U;
    uint8_t* newData = new (std::nothrow) uint8_t[bytes];
    if (!newData) {
        return false;
    }

    memset(newData, 0, bytes);

    delete[] _data;
    _data = newData;
    _m = m;
    _n = n;
    _posX = 0;
    _posY = 0;
    _targetX = 0;
    _targetY = 0;
    _hasTarget = false;
    return true;
}

size_t Map::bitIndex(unsigned int x, unsigned int y) const {
    return static_cast<size_t>(y) * static_cast<size_t>(_m) + static_cast<size_t>(x);
}

size_t Map::linearIndex(unsigned int x, unsigned int y) const {
    return bitIndex(x, y);
}

Map::Position Map::positionFromIndex(size_t index) const {
    Position p;
    p.x = static_cast<unsigned int>(index % _m);
    p.y = static_cast<unsigned int>(index / _m);
    return p;
}

size_t Map::byteIndex(unsigned int x, unsigned int y) const {
    return bitIndex(x, y) / 8U;
}

uint8_t Map::bitMask(unsigned int x, unsigned int y) const {
    return static_cast<uint8_t>(1U << (bitIndex(x, y) % 8U));
}

bool Map::isInBounds(unsigned int x, unsigned int y) const {
    return (x < _m) && (y < _n);
}
