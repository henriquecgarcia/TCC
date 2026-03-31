# Map (ESP32 + Arduino)

Este documento descreve como usar a classe `Map` implementada em:

- `lib/Mapping/Map.h`
- `lib/Mapping/Map.cpp`

A classe foi projetada para microcontroladores com foco em:

- baixo uso de RAM (1 bit por celula)
- leitura/escrita rapida no SPIFFS
- operacao segura com verificacao de limites
- pathfinding com A* para rota em grade

## 1. Conceito da grade

Cada celula armazena apenas:

- `0`: livre
- `1`: ocupado

A memoria e compactada com 1 bit por celula.

Exemplo de consumo aproximado:

- mapa 64x64 -> 4096 bits -> 512 bytes
- mapa 200x200 -> 40000 bits -> 5000 bytes

## 2. Criacao e inicializacao

### Opcao A: tamanho direto (M x N)

```cpp
#include "Map.h"

Map mapGrid(64, 64);

void setup() {
    Serial.begin(115200);

    if (!mapGrid.begin(false)) {
        Serial.println("Falha ao iniciar SPIFFS");
        return;
    }
}
```

### Opcao B: calcular tamanho por dimensao real

Use a mesma unidade para tudo (cm, m, etc).

Exemplo:

- planta: `300 cm x 500 cm`
- celula: `30 cm x 20 cm`
- grade: `M = ceil(300/30) = 10`, `N = ceil(500/20) = 25`

```cpp
Map mapGrid;

void setup() {
    Serial.begin(115200);

    if (!mapGrid.begin(false)) {
        Serial.println("Falha ao iniciar SPIFFS");
        return;
    }

    if (!mapGrid.resizeFromRealDimensions(300.0f, 500.0f, 30.0f, 20.0f)) {
        Serial.println("Falha ao calcular tamanho da grade");
        return;
    }

    Serial.print("Mapa: ");
    Serial.print(mapGrid.getWidth());
    Serial.print(" x ");
    Serial.println(mapGrid.getHeight());
}
```

## 3. Operacoes basicas

```cpp
// Marcar celula ocupada
mapGrid.setCell(10, 10, 1);

// Marcar celula livre
mapGrid.setCell(10, 10, 0);

// Ler celula
uint8_t v = mapGrid.getCell(10, 10);

// Posicao atual do robo
mapGrid.setPosition(2, 3);
Map::Position p = mapGrid.getPosition();

// Teste de ocupacao
if (mapGrid.isOccupied(10, 10)) {
    // celula bloqueada
}
```

## 4. Persistencia no SPIFFS

### Salvar

```cpp
if (!mapGrid.saveToFile("/map.bin")) {
    Serial.println("Erro ao salvar mapa");
}
```

### Carregar

```cpp
if (!mapGrid.loadFromFile("/map.bin")) {
    Serial.println("Erro ao carregar mapa");
}
```

Formato do arquivo:

- cabecalho binario fixo (magic, versao, dimensoes, posicao, tamanho)
- bloco continuo de bits do mapa

Isso permite leitura rapida no ESP32.

## 5. A* (pathfinding)

A funcao principal e:

```cpp
bool findPathAStar(
    unsigned int targetX,
    unsigned int targetY,
    Position* outPath,
    size_t maxPathLen,
    size_t& outPathLen
) const;
```

### Regras

- origem: `getPosition()`
- destino: `(targetX, targetY)`
- movimentos: 4 direcoes (cima, baixo, esquerda, direita)
- celulas ocupadas sao obstaculos
- heuristica: Manhattan (boa para grade 4-direcoes)

### Exemplo completo

```cpp
Map::Position path[512];
size_t pathLen = 0;

mapGrid.setPosition(0, 0);

if (mapGrid.findPathAStar(9, 24, path, 512, pathLen)) {
    Serial.print("Caminho encontrado. Tamanho: ");
    Serial.println(pathLen);

    for (size_t i = 0; i < pathLen; ++i) {
        Serial.print("Passo ");
        Serial.print(i);
        Serial.print(": (");
        Serial.print(path[i].x);
        Serial.print(", ");
        Serial.print(path[i].y);
        Serial.println(")");
    }
} else {
    Serial.println("Nao foi possivel gerar caminho");
}
```

### Falhas comuns do A*

A funcao retorna `false` quando:

- origem/destino fora do mapa
- origem ou destino ocupados
- `outPath` nulo
- `maxPathLen` pequeno para o caminho encontrado
- falta de memoria temporaria para o algoritmo

## 6. Boas praticas no ESP32

- manter mapa o menor possivel para reduzir uso de RAM
- evitar recalcular A* a cada loop; recalcular apenas em mudancas relevantes
- salvar no SPIFFS em momentos controlados (nao em alta frequencia)
- validar limites antes de qualquer leitura/escrita manual

## 7. API resumida

- `Map(unsigned int m, unsigned int n)`
- `Map()`
- `bool begin(bool formatOnFail = false)`
- `bool resize(unsigned int m, unsigned int n)`
- `bool resizeFromRealDimensions(float realWidth, float realHeight, float cellWidth, float cellHeight)`
- `bool setCell(unsigned int x, unsigned int y, uint8_t value)`
- `uint8_t getCell(unsigned int x, unsigned int y) const`
- `bool setPosition(unsigned int x, unsigned int y)`
- `Position getPosition() const`
- `bool isOccupied(unsigned int x, unsigned int y) const`
- `bool findPathAStar(unsigned int targetX, unsigned int targetY, Position* outPath, size_t maxPathLen, size_t& outPathLen) const`
- `bool loadFromFile(const char* path)`
- `bool saveToFile(const char* path) const`
- `unsigned int getWidth() const`
- `unsigned int getHeight() const`
- `size_t getDataSizeBytes() const`
