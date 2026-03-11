#ifndef MAP_H
#define MAP_H

#include <Arduino.h>
#include <SPIFFS.h>

class Map {
public:
    struct Position {
        unsigned int x;
        unsigned int y;
    };

    Map(unsigned int m, unsigned int n);
    Map();
    ~Map();

    bool begin(bool formatOnFail = false);

    bool resize(unsigned int m, unsigned int n);
    bool resizeFromRealDimensions(float realWidth, float realHeight, float cellWidth, float cellHeight);
    static bool calculateGridSizeFromRealDimensions(
        float realWidth,
        float realHeight,
        float cellWidth,
        float cellHeight,
        unsigned int& outM,
        unsigned int& outN
    );

    bool setCell(unsigned int x, unsigned int y, uint8_t value);
    uint8_t getCell(unsigned int x, unsigned int y) const;

    bool setPosition(unsigned int x, unsigned int y);
    Position getPosition() const;

    bool isOccupied(unsigned int x, unsigned int y) const;

    bool findPathAStar(
        unsigned int targetX,
        unsigned int targetY,
        Position* outPath,
        size_t maxPathLen,
        size_t& outPathLen
    ) const;

    bool loadFromFile(const char* path);
    bool saveToFile(const char* path) const;

    unsigned int getWidth() const;
    unsigned int getHeight() const;
    size_t getDataSizeBytes() const;

private:
    struct FileHeader {
        uint32_t magic;
        uint16_t version;
        uint16_t reserved;
        uint32_t width;
        uint32_t height;
        uint32_t posX;
        uint32_t posY;
        uint32_t dataSize;
    };

    static const uint32_t FILE_MAGIC = 0x4D415031; // "MAP1"
    static const uint16_t FILE_VERSION = 1;

    unsigned int _m;
    unsigned int _n;
    unsigned int _posX;
    unsigned int _posY;
    uint8_t* _data;

    bool _spiffsReady;

    bool allocateGrid(unsigned int m, unsigned int n);
    size_t bitIndex(unsigned int x, unsigned int y) const;
    size_t linearIndex(unsigned int x, unsigned int y) const;
    Position positionFromIndex(size_t index) const;
    size_t byteIndex(unsigned int x, unsigned int y) const;
    uint8_t bitMask(unsigned int x, unsigned int y) const;
    bool isInBounds(unsigned int x, unsigned int y) const;
};

#endif
