# Proximos passos sugeridos:
> Fonte: Google Gemini Pro, gema customizada para o projeto e com conhecimento atualizado até março de 2026. (Commit anterior a este)

## 1. Sistema de Energia e Estabilidade
Segundo, o Professor:
> Item resolvido com a troca da corrente utilizada para alimentar os sensores do 3.3V do ESP32 para 5V, direto da Ponte H, nos testes iniciais funcionou bem, mas é importante monitorar a tensão da bateria durante o uso para garantir que não haja quedas que possam causar instabilidade ou resets do ESP32.
* Verificar a tensão da bateria sem carga e com carga.
* Verificar o uso de duas baterias (implementar um regulador de tensão).

Segundo a LLM:
* **Hardware:** Utilizar um circuito isolado: manter a potência bruta das baterias (ex: LiPo 7.4V) direto para a Ponte H e adicionar um regulador Step-Down (ex: LM2596) cravado em 5V exclusivo para o ESP32, evitando *brownouts*.
* **Firmware:** Implementar um divisor de tensão lido por um pino analógico (ADC) do ESP32 para monitorar a tensão em tempo real e criar uma trava de segurança (desligar motores se a bateria ficar muito baixa).
* **Texto Acadêmico:** Criar a seção "Dimensionamento Energético e Isolamento de Ruídos", incluindo o esquemático elétrico do divisor de tensão e do regulador, justificando a separação lógica/potência para estabilidade do sistema.

## 2. Controle das Rodas (PID) e Odometria
Segundo, o Professor:
* Colocar a roda boba (diminui a incerteza dos outros motores).
	- Eu: Uma ideia interessante, porém com a nova estabilidade após arrumar o item #1, pode ser que isso não seja mais necessário. Vale a pena testar o carrinho com as rodas atuais e verificar se a estabilidade é suficiente para o controle preciso.
* Tirar o PID, aplicar PWMs iguais, medir rotação na porta Serial e escolher motores parecidos.
	- Eu: Acho interessante para fazer os testes com os motores atuais, para entender o comportamento do sistema. Se os motores forem muito diferentes, pode ser necessário ajustar o PID ou considerar a substituição de um dos motores para garantir melhor desempenho.
* Regular os ganhos do PID e verificar o resultado final.
	- Eu: Acho que é importante fazer uma análise detalhada do comportamento do carrinho com os ajustes atuais, antes de decidir sobre a necessidade de uma roda boba ou substituição de motores. Se o controle PID estiver funcionando bem e o carrinho for estável, pode ser que não seja necessário fazer mudanças drásticas.
* Avaliar o uso de um Magnetômetro.
	- Eu: Realmente melhoraria a precisão da odometria, especialmente para correção de deriva. Vale a pena considerar a adição de um magnetômetro para complementar os dados do IMU e melhorar a navegação.

Segundo a LLM:
* **Hardware:** A instalação da roda boba (*caster wheel*) é mandatória para evitar o arraste do eixo traseiro, que gera um atrito não-linear imprevisível para o PID.
	- Eu: Mesma coisa que disse antes.
* **Firmware:** Criar um script temporário em malha aberta para aplicar degraus de PWM, exportar os logs da Serial para Excel/Python e aplicar um "fator de correção" no PWM base antes mesmo do PID atuar, compensando as assimetrias mecânicas.
	- Eu: Estudo mais que válido para entender o comportamento dos motores e ajustar o sistema, mas... Vamos ver...
* **Firmware (PID):** Garantir que o PID rode a uma frequência de amostragem perfeitamente cravada (ex: usando interrupção de hardware ou *Task* fixada em um *Core* pelo FreeRTOS a 50Hz) e habilitar a proteção *Anti-Windup*.
	- Eu: Não entendi nada disso... Verificar com Marcorin.
* **Sensores:** O magnetômetro deve ser usado como filtro complementar ao giroscópio (MPU6050) para anular o *drift* (desvio acumulado) no eixo Z ao longo do tempo.
	- Eu: Mas o magnetômetro não seria um substituto para o giroscópio? | Verificar...
* **Texto Acadêmico:** Adicionar seção "Sintonia do Controlador PID", exibindo os gráficos do ensaio de malha aberta e o gráfico "antes vs depois" do controle em malha fechada demonstrando a redução do erro.
	- Eu: Soon (TM).

## 3. Comandos de Navegação de Alto Nível
Segundo, o Professor:
* Virar 90 graus a direção (direita).
	- Eu: Com os ajustes de estabilidade e controle, o carrinho deve ser capaz de realizar giros precisos. Ajustes devem ser feitos para garantir que o giro seja de apenas 90 graus, sem overshoot ou undershoot.
* Virar 90 graus para esquerda.
	- Eu: Com os ajustes de estabilidade e controle, o carrinho deve ser capaz de realizar giros precisos. Ajustes devem ser feitos para garantir que o giro seja de apenas 90 graus, sem overshoot ou undershoot.
* Andar X.
	- Eu: Odometria...
* Voltar X.
	- Eu: Odometria...

Segundo a LLM:
* **Firmware (Arquitetura):** Abstrair esses movimentos através de uma Máquina de Estados Finitos (FSM).
	- Eu: Tf this be?
* **Firmware (Rotação):** Criar a função `virar_angulo(float angulo)` que lê continuamente o integrador do giroscópio (eixo Z) e, ao atingir o alvo (ex: 90º), aplica um freio reverso momentâneo nos motores para zerar a inércia.
	- Eu: Potencia muito fraca pra precisar do freio reverso, mas a função é essencial para o controle de orientação.
* **Firmware (Translação):** Criar a função `andar_distancia(float cm)` que converte o raio da roda em "ticks de encoder por centímetro" e alimenta o *setpoint* do PID até o alvo ser alcançado.
	- Eu: Odometria... + Mapas e Trajetória...
* **Texto Acadêmico:** Desenhar a Máquina de Estados em LaTeX e apresentar as equações de Cinemática Direta do robô diferencial (relacionando a rotação das rodas com a velocidade linear do centro do robô).
	- Eu: Sure? Soon (TM).

## 4. Mapa, Trajetória e Localização
Segundo, o Professor:
* Ir do ponto A ao ponto B.
	- Eu: Fazer um algoritimo que controla o mapa, a trajetória e a localização do carrinho. Ideal: bitmaps para o mapa, A* para a trajetória e correção de localização com as faixas no chão.
* Fazer a localização simultânea no mapa em grid.
	- Eu: Fazer o algoritimo a cima + Odometria
* Usar faixas no chão?
	- Eu: Acho que perde um pouco da ideia de 100% autônomo, mas é uma solução prática e eficaz para o ambiente de teste. As faixas no chão funcionariam como "marcos" para correção de posição, permitindo que o carrinho se localize com mais precisão e compense os erros acumulados pela odometria.
* Chegar ao destino B com o mínimo erro de localização possível.
	- Eu: A combinação de um mapa em grid, o algoritmo A* para planejamento de trajetória.

Segundo a LLM:
* **Firmware (Algoritmo):** Implementar o mapa como uma matriz 2D embarcada (`int map[10][10]`) e utilizar o algoritmo A* (A-Star) para calcular a rota mais curta em grid.
	- Eu: A* é o algoritmo clássico para esse tipo de problema, mas... Será que o ESP32 aguenta? Vamos ver...
* **Hardware/Navegação:** A ideia do professor das faixas no chão é excelente. Instalar sensores reflexivos (como TCRT5000) apontados para o chão. Isso substitui um SLAM complexo e pesado por "Localização Topológica".
	- Eu: Avaliar... Mas como disse antes... Perde a ideia de ser 100% autônomo...
* **Firmware (Correção):** Toda vez que o robô cruza uma fita preta no chão, o software força a atualização da sua posição na matriz e "zera" a incerteza acumulada pela odometria morta (*dead reckoning*).
* **Texto Acadêmico:** Escrever um capítulo "Planejamento de Trajetória e Correção Odometrica", defendendo teoricamente por que a navegação puramente baseada nos encoders falha com o tempo (derrapagens/folgas) e justificando o uso de marcos fixos (faixas) para robustez no ambiente de teste.
	- Eu: Soon (TM).

