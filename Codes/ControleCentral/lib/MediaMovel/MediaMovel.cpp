#include "MediaMovel.h"
#include <algorithm> // Para std::copy
#include <new>       // Para o operador new (boa prática incluir)

// Construtor
MediaMovel::MediaMovel(int initial_size) : values(nullptr), size(initial_size), index(0) {
    if (this->size < 1) {
        this->size = 1; // Garante que o tamanho seja pelo menos 1
    }
    // Aloca memória e inicializa com zeros (o () após new double[size] faz isso)
    this->values = new double[this->size]();
}

// Destrutor
MediaMovel::~MediaMovel() {
    delete[] values;
    values = nullptr; // Boa prática para evitar ponteiros pendurados
}

// Construtor de cópia
MediaMovel::MediaMovel(const MediaMovel& other)
    : values(nullptr), size(other.size), index(other.index) {
    this->values = new double[this->size];
    std::copy(other.values, other.values + this->size, this->values);
}

// Operador de atribuição por cópia
MediaMovel& MediaMovel::operator=(const MediaMovel& other) {
    if (this != &other) { // Proteção contra auto-atribuição
        // Libera recursos antigos
        delete[] values;

        // Copia os dados do outro objeto
        this->size = other.size;
        this->index = other.index;
        this->values = new double[this->size];
        std::copy(other.values, other.values + this->size, this->values);
    }
    return *this;
}

// Adiciona um novo valor
void MediaMovel::addValue(double value) {
    if (this->values == nullptr || this->size == 0) {
        return; // Objeto não inicializado corretamente (embora o construtor deva evitar isso)
    }
    this->values[this->index] = value;
    this->index = (this->index + 1) % this->size;
}

// Calcula a média
double MediaMovel::getAverage() const {
    if (this->values == nullptr || this->size == 0) {
        return 0.0; // Retorna 0 se não houver valores ou tamanho for 0
    }

    double sum = 0.0;
    for (int i = 0; i < this->size; ++i) {
        sum += this->values[i];
    }
    // A média é sempre calculada sobre o número total de elementos na janela,
    // mesmo que alguns sejam os zeros iniciais.
    return sum / this->size;
}