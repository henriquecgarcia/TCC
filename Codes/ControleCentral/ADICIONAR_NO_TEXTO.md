# Adicionar ao Texto
> Atualizado: 17.03.2026

## Problema 1: Sistema de Energia e Estabilidade

### Visão geral:
Ao colocar o carrinho em movimento, o ESP32 apresentava quedas de tensão e resets.

### Causa aparente:
Algum dos sensores (MPU6050, Infrared) estava consumindo mais corrente do que o regulador de 3.3V do ESP32 podia fornecer, causando uma queda de tensão que resultava em resets.
- Problema identificado com auxilo de um multímetro, monitorando a tensão durante o comando de movimento.
- Auxilio dos tecnicos de laboratório para entender a distribuição de energia e os limites do regulador onboard do ESP32.

### Solução:
O problema foi resolvido na troca da corrente utilizada para alimentar os sensores do 3.3V do ESP32 para 5V, direto da Ponte H.

## Problema 2: Controle de "Curvas":

### Visão geral:
Uma vez que o "Problema 1" foi resolvido, pude testar melhor o "ciclo de caminhada" do carrinho, inicialmente com comandos de movimento simples (ex: andar para frente, virar 90 graus). Percebi que o carrinho não estava conseguindo realizar curvas precisas, apresentando um comportamento instável e errático. Durante as curvas, o carrinho tendia a ir mais do que os 90* desejados.

### Causa aparente:
Inicialmente, a analise do código mostrou que o controle de curvas estava comparando radianos (do giroscópio) com graus (setpoint), o que causava um erro de controle. Após corrigir essa questão, o comportamento melhorou, mas ainda apresentava instabilidade de virar mais do que o desejado (Comando ex: direita 90 graus, o carrinho virava mais do que o desejado, digamos +- 10 graus).

### Solução:
1. Primeiro foi alterado o comando "virar para direita" para não ser giros fixos de 90*, mas sim um comando de "virar para direita X graus", onde o valor de X é transformado em radianos e comparado com a leitura do giroscópio, garantindo que o carrinho pare exatamente no ângulo desejado.
2. Para resolver o problema do overshoot (ainda não resolvido completamente), foi necessário ajustar os ganhos do PID, reduzindo o ganho proporcional e aumentando o ganho derivativo para melhorar a resposta do sistema e reduzir a tendência de ultrapassar o setpoint.


## Problema 3: Navegação:

### Visão geral:
Conversando com um os técnicos de eletrônica do laboratório, chegamos ao assunto de navegação, ele sugeriu o uso de g-code para controlar o carrinho, o que não é muito diferente do que "foi" implementado, mas pensando durante a conversa, percebi que o constante liga e desliga dos motores para realizar as curvas, ir para frente, para trás, etc, não é a melhor forma de controlar o carrinho. O ideal seria um sistema que evite muitas curvas, limitando para o máximo possível o movimento em linha reta, e realizando curvas "fixas" (ex: virar 90 graus para direita, virar 90 graus para esquerda) apenas quando necessário, e não como parte do processo de navegação.

### Causa aparente:
O algoritmo selecionado para controle de navegação é um algoritmo de "caminho mais curto" (A*), que é um algoritmo de busca que encontra o caminho mais curto entre um ponto inicial e um ponto final em um grafo. O problema é que o A* não leva em consideração a dinâmica do robô, ou seja, ele pode sugerir um caminho que exige muitas curvas, o que não é ideal para o controle do carrinho.

### Solução pesquisada:
1. Entender um pouco melhor um algoritmo chamado "Jump Point Search" (JPS), que é uma otimização do A* para grids, que reduz o número de nós expandidos e pode gerar caminhos mais "suaves", com menos curvas.
   1. Uma solução, seria colocar uma "punição" para curvas no algoritmo A*, ou seja, aumentar o custo de realizar uma curva, para que o algoritmo prefira caminhos mais retos, e só realize curvas quando realmente necessário.