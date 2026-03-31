#ifndef MEDIA_MOVEL_H
#define MEDIA_MOVEL_H

// Para Arduino, muitas inclusões padrão são gerenciadas pelo toolchain,
// mas é uma boa prática ser explícito se souber que precisa de algo como std::copy
// que vem de <algorithm>. No entanto, para a declaração da classe em si,
// não são necessárias inclusões aqui. std::copy é usado na implementação.

class MediaMovel {
private:
    double* values; // Ponteiro para o array de valores
    int size;       // Número de elementos no array (tamanho da janela da média)
    int index;      // Próximo índice para inserir um novo valor (de forma circular)

public:
    // Construtor: Especifica o tamanho da janela da média móvel
    explicit MediaMovel(int size); // 'explicit' para evitar conversões implícitas

    // Destrutor: Libera a memória alocada
    ~MediaMovel();

    // Construtor de cópia: Cria uma cópia profunda do objeto
    MediaMovel(const MediaMovel& other);

    // Operador de atribuição por cópia: Permite atribuição segura entre objetos
    MediaMovel& operator=(const MediaMovel& other);

    // Adiciona um novo valor à janela da média móvel
    // Substitui o valor mais antigo se a janela estiver cheia
    void addValue(double value);

    // Alias para addValue para conveniência
    void add(double value) {
        addValue(value);
    }

    // Calcula e retorna a média dos valores na janela
    double getAverage() const;

    // Alias para getAverage para conveniência
    double get() const {
        return getAverage();
    }

    // Método para obter o tamanho da janela (opcional, mas útil)
    int getWindowSize() const {
        return size;
    }
};

#endif // MEDIA_MOVEL_H