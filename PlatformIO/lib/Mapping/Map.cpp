#include "Map.h"

#include <math.h>
#include <new>

Map::Map(unsigned int m, unsigned int n)
    : _m(0), _n(0), _posX(0), _posY(0), _data(nullptr), _spiffsReady(false) {
    allocateGrid(m, n);
}

Map::Map()
    : _m(0), _n(0), _posX(0), _posY(0), _data(nullptr), _spiffsReady(false) {}

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

bool Map::isOccupied(unsigned int x, unsigned int y) const {
    return getCell(x, y) == 1;
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

    uint32_t* gScore = new (std::nothrow) uint32_t[totalCells];
    int32_t* cameFrom = new (std::nothrow) int32_t[totalCells];
    uint8_t* closedSet = new (std::nothrow) uint8_t[(totalCells + 7U) / 8U];
    uint32_t* openHeap = new (std::nothrow) uint32_t[totalCells];
    int32_t* heapPos = new (std::nothrow) int32_t[totalCells];

    if (!gScore || !cameFrom || !closedSet || !openHeap || !heapPos) {
        delete[] gScore;
        delete[] cameFrom;
        delete[] closedSet;
        delete[] openHeap;
        delete[] heapPos;
        return false;
    }

    for (size_t i = 0; i < totalCells; ++i) {
        gScore[i] = 0xFFFFFFFFUL;
        cameFrom[i] = -1;
        heapPos[i] = -1;
    }
    memset(closedSet, 0, (totalCells + 7U) / 8U);

    const auto closedGet = [&](size_t idx) -> bool {
        return (closedSet[idx / 8U] & (1U << (idx % 8U))) != 0;
    };

    const auto closedSetBit = [&](size_t idx) {
        closedSet[idx / 8U] |= static_cast<uint8_t>(1U << (idx % 8U));
    };

    const auto heuristic = [&](size_t idx) -> uint32_t {
        const Position p = positionFromIndex(idx);
        const unsigned int dx = (p.x > targetX) ? (p.x - targetX) : (targetX - p.x);
        const unsigned int dy = (p.y > targetY) ? (p.y - targetY) : (targetY - p.y);
        return static_cast<uint32_t>(dx + dy);
    };

    const auto fScore = [&](size_t idx) -> uint32_t {
        return gScore[idx] + heuristic(idx);
    };

    size_t heapSize = 0;

    const auto heapSwap = [&](size_t a, size_t b) {
        const uint32_t tmp = openHeap[a];
        openHeap[a] = openHeap[b];
        openHeap[b] = tmp;
        heapPos[openHeap[a]] = static_cast<int32_t>(a);
        heapPos[openHeap[b]] = static_cast<int32_t>(b);
    };

    const auto siftUp = [&](size_t idx) {
        while (idx > 0) {
            const size_t parent = (idx - 1U) / 2U;
            if (fScore(openHeap[parent]) <= fScore(openHeap[idx])) {
                break;
            }
            heapSwap(parent, idx);
            idx = parent;
        }
    };

    const auto siftDown = [&](size_t idx) {
        while (true) {
            const size_t left = idx * 2U + 1U;
            const size_t right = left + 1U;
            size_t smallest = idx;

            if (left < heapSize && fScore(openHeap[left]) < fScore(openHeap[smallest])) {
                smallest = left;
            }
            if (right < heapSize && fScore(openHeap[right]) < fScore(openHeap[smallest])) {
                smallest = right;
            }
            if (smallest == idx) {
                break;
            }
            heapSwap(idx, smallest);
            idx = smallest;
        }
    };

    const auto heapPushOrDecrease = [&](size_t nodeIdx) {
        if (heapPos[nodeIdx] >= 0) {
            siftUp(static_cast<size_t>(heapPos[nodeIdx]));
            return;
        }

        openHeap[heapSize] = static_cast<uint32_t>(nodeIdx);
        heapPos[nodeIdx] = static_cast<int32_t>(heapSize);
        siftUp(heapSize);
        ++heapSize;
    };

    gScore[startIndex] = 0;
    heapPushOrDecrease(startIndex);

    bool found = false;

    while (heapSize > 0) {
        const size_t current = openHeap[0];
        heapPos[current] = -1;

        --heapSize;
        if (heapSize > 0) {
            openHeap[0] = openHeap[heapSize];
            heapPos[openHeap[0]] = 0;
            siftDown(0);
        }

        if (closedGet(current)) {
            continue;
        }

        closedSetBit(current);

        if (current == goalIndex) {
            found = true;
            break;
        }

        const Position c = positionFromIndex(current);

        const int dirX[4] = {1, -1, 0, 0};
        const int dirY[4] = {0, 0, 1, -1};

        for (size_t i = 0; i < 4; ++i) {
            const int nx = static_cast<int>(c.x) + dirX[i];
            const int ny = static_cast<int>(c.y) + dirY[i];

            if (nx < 0 || ny < 0) {
                continue;
            }

            const unsigned int ux = static_cast<unsigned int>(nx);
            const unsigned int uy = static_cast<unsigned int>(ny);
            if (!isInBounds(ux, uy) || isOccupied(ux, uy)) {
                continue;
            }

            const size_t neighbor = linearIndex(ux, uy);
            if (closedGet(neighbor)) {
                continue;
            }

            const uint32_t tentativeG = gScore[current] + 1U;
            if (tentativeG < gScore[neighbor]) {
                cameFrom[neighbor] = static_cast<int32_t>(current);
                gScore[neighbor] = tentativeG;
                heapPushOrDecrease(neighbor);
            }
        }
    }

    if (!found) {
        delete[] gScore;
        delete[] cameFrom;
        delete[] closedSet;
        delete[] openHeap;
        delete[] heapPos;
        return false;
    }

    size_t pathLen = 1;
    for (int32_t cur = static_cast<int32_t>(goalIndex); cur != static_cast<int32_t>(startIndex); cur = cameFrom[cur]) {
        if (cur < 0) {
            delete[] gScore;
            delete[] cameFrom;
            delete[] closedSet;
            delete[] openHeap;
            delete[] heapPos;
            return false;
        }
        ++pathLen;
    }

    if (pathLen > maxPathLen) {
        delete[] gScore;
        delete[] cameFrom;
        delete[] closedSet;
        delete[] openHeap;
        delete[] heapPos;
        return false;
    }

    int32_t cur = static_cast<int32_t>(goalIndex);
    for (size_t writePos = pathLen; writePos > 0; --writePos) {
        outPath[writePos - 1U] = positionFromIndex(static_cast<size_t>(cur));
        cur = cameFrom[cur];
    }

    outPathLen = pathLen;

    delete[] gScore;
    delete[] cameFrom;
    delete[] closedSet;
    delete[] openHeap;
    delete[] heapPos;
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
