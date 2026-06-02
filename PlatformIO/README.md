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


## Navegação por células de 30 cm x 30 cm

Esta versão adiciona um comando de navegação discreta para o TCC: **"Vá X células para frente"**.
Cada célula representa **30 cm x 30 cm** no mundo real.

### Como funciona

O robô não tenta andar todas as células em um único movimento contínuo. Ele executa uma máquina de estados não bloqueante:

1. Registra a pose inicial da célula atual pelo `PoseEKF`.
2. Anda para frente até completar aproximadamente `0,30 m`.
3. Para os motores.
4. Aguarda uma pequena estabilização mecânica e de leitura.
5. Mede o erro de angulação em relação ao heading inicial do comando.
6. Corrige a angulação com uma curva curta, caso o erro seja maior que 2 graus.
7. Repete o ciclo até completar a quantidade solicitada.

Esse comportamento facilita validação experimental, logs por célula, futura integração com mapa ocupacional e controle mais estável para navegação em grade.

### Comandos aceitos

Via WebSocket do carro, é possível usar texto:

```text
vá 3 células para frente
va 3 celulas para frente
3 celulas para frente
```

Ou JSON:

```json
{"action":"forward_cells","cells":3}
```

Também é aceito:

```json
{"action":"go_cells_forward","cells":3}
```

### Telemetria adicionada

A telemetria passa a incluir o objeto `cell_nav`:

```json
{
  "cell_nav": {
    "state": "moving_cell",
    "active": true,
    "requested": 3,
    "completed": 1,
    "cell_size_cm": 50
  }
}
```

### Próximos passos sugeridos para o TCC

1. **Calibração experimental da célula de 30 cm**: medir o erro médio real em 10, 20 e 30 execuções e ajustar `CELL_TARGET_TOLERANCE_M`, PID dos motores e ruídos do EKF.
2. **Validação de odometria por encoder**: registrar ticks por célula em linha reta para verificar se o raio da roda, base entre rodas e `encoderTicksPerRev` estão coerentes.
3. **Correção angular mais robusta**: substituir a correção por curva relativa simples por um controlador de heading contínuo durante o avanço da célula.
4. **Registro de trajetória por célula**: salvar em SPIFFS/CSV a célula inicial, célula final, erro linear, erro angular e leitura do VL53L0X.
5. **Integração com A\***: transformar cada comando de avanço em um passo de execução de rota, usando a mesma lógica de parar, corrigir e avançar para a próxima célula.
6. **Detecção dinâmica de obstáculo**: quando o VL53L0X parar o robô antes de completar a célula, marcar a próxima célula como ocupada no `Map` e recalcular rota.
7. **Dashboard de TCC**: exibir na Web UI a célula atual, células solicitadas/concluídas, erro angular e estado da máquina de navegação.

---

## Versão TCC — implementação dos itens 3, 4, 6, 8, 9, 11 e 12

Esta versão implementa as próximas etapas solicitadas para evolução do TCC.

### 3. PID angular contínuo durante cada célula

Durante o estado `moving_cell`, o `GridCellNavigator` calcula continuamente:

```cpp
headingError = poseEKF->getTheta() - targetHeadingRad;
```

Esse erro é enviado para a `PonteH` por:

```cpp
ponte->setExternalHeadingError(headingError);
```

A `PonteH` aplica o PID `pidCellHeading` de forma diferencial nos motores, reduzindo o desvio antes de chegar ao final da célula. A correção residual ao final da célula continua existindo como segunda camada de precisão.

### 4. Logs experimentais em CSV

Foi adicionada a classe:

```txt
lib/ExperimentLogger/
```

Ela grava o arquivo:

```txt
/tcc_experiment_log.csv
```

Download pela interface HTTP:

```txt
/api/tcc-log.csv
```

Limpeza do log:

```txt
POST /api/tcc-log/clear
```

Comandos WebSocket:

```json
{"action":"experiment_log","mode":"enable"}
{"action":"experiment_log","mode":"disable"}
{"action":"experiment_log","mode":"clear"}
```

O CSV registra eventos, pose, erro angular, célula atual, distância frontal, modo A*, replanejamento e progresso da navegação.

### 6. Ensaios formais para a monografia

Foi criado o arquivo:

```txt
EXPERIMENTS.md
```

Ele descreve ensaios de:

- deslocamento por célula;
- erro angular;
- execução de caminho A*;
- obstáculo dinâmico e replanejamento.

### 8. Detecção de obstáculo e atualização do mapa

Durante `moving_cell`, o VL53L0X é lido continuamente. Caso a distância frontal seja menor ou igual a:

```cpp
FRONT_OBSTACLE_REPLAN_MM = 220;
```

O robô:

1. para os motores;
2. identifica a célula à frente pelo heading atual;
3. marca a célula como ocupada no bitmap do `Map`;
4. publica evento WebSocket;
5. registra o evento no CSV.

### 9. Replanejamento automático

O comando `path_to` agora pode executar automaticamente o caminho planejado:

```json
{"action":"path_to","x":8,"y":4,"execute":true}
```

Durante a execução, se um obstáculo for detectado, o sistema tenta recalcular o caminho até o alvo final usando o mapa atualizado.

Para apenas calcular sem executar:

```json
{"action":"path_to","x":8,"y":4,"execute":false}
```

### 11. Organização da arquitetura

Foi criada a documentação:

```txt
ARCHITECTURE.md
```

A nova classe `ExperimentLogger` já está em `lib/`, seguindo a arquitetura modular do PlatformIO. O `GridCellNavigator` ainda permanece em `src/carrinho.ino` porque depende diretamente da `PonteH`, que também está no arquivo principal. A próxima refatoração segura é mover `PonteH` para `lib/DriveBase/` e depois mover `GridCellNavigator` para `lib/Navigator/`.

### 12. Documentação adicionada

Novos documentos:

```txt
INSTALL.md
PINOUTS.md
CALIBRATION.md
EXPERIMENTS.md
ARCHITECTURE.md
RESULTS.md
```

Esses arquivos ajudam a transformar o projeto em um pacote defendível academicamente: instalação, pinagem, calibração, metodologia de testes, arquitetura e registro de resultados.


## Mapa binário 7x7

O projeto agora inicializa um mapa binário 7x7 para os testes do TCC:

```txt
1111111
1000001
1000001
1000001
1000001
1000001
1111111
```

O robô começa na célula `(1,1)`. A célula `(0,0)` é o canto superior esquerdo da matriz e faz parte da borda ocupada. Cada célula mede 30 cm x 30 cm.

Para mandar o robô andar usando o planejador A*, envie pelo WebSocket:

```json
{"action":"path_to","x":5,"y":5,"execute":true}
```

Para andar manualmente por células:

```json
{"action":"forward_cells","cells":3}
```

A telemetria WebSocket possui `pose` para posição contínua em metros e `location` para posição discreta no mapa.

## Correção de navegação por múltiplas células

Esta versão corrige um problema observado em testes físicos: após andar uma célula, o robô podia ficar parado, acumular drift de localização e depois se perder ou iniciar correções angulares excessivas.

Correções aplicadas:

- Leitura do MPU6050 em thread FreeRTOS própria.
- Leitura do VL53L0X em thread FreeRTOS própria.
- Cache thread-safe para sensores.
- Travamento da pose quando a PonteH está parada.
- Snap da pose para a célula discreta esperada ao fim de cada célula.
- Redução do intervalo de controle dos motores para 100 ms.
- A Web UI agora recebe `sensor_age.mpu_ms` e `sensor_age.tof_ms` para diagnosticar atraso nas leituras.

Teste recomendado:

1. Envie `{"action":"forward_cells","cells":1}` e confirme que a posição para exatamente na próxima célula.
2. Aguarde 30 segundos sem enviar comandos e verifique se `location.grid_x`, `location.grid_y`, `world_x_m`, `world_y_m` e `theta_rad` não derivam.
3. Envie `{"action":"forward_cells","cells":3}` e verifique se ele executa uma célula por vez sem iniciar giro contínuo.

## Atualização: UI moderna e ré em caminho A*

Esta versão substitui o painel principal `dashboard.html/css/js` por uma interface moderna, sem tema LCARS/Star Trek. O comando de avanço direto da UI agora é **Vá 1 célula para frente**, usando o comando seguro por célula:

```json
{"action":"forward_cells","cells":1}
```

O algoritmo de execução de caminho A* também foi ajustado: quando o próximo passo planejado está diretamente atrás do robô, o navegador não executa giro de 180°. Ele mantém o heading atual e percorre a célula **dando ré**. Isso reduz giros desnecessários e melhora a estabilidade em mapas pequenos.

Também foi adicionada uma camada de ticks assinados para o EKF. Como os encoders contam pulsos absolutos, o firmware agora aplica o sinal correto conforme o movimento comandado:

- frente: esquerdo `+`, direito `+`;
- ré: esquerdo `-`, direito `-`;
- giro esquerda: esquerdo `+`, direito `-`;
- giro direita: esquerdo `-`, direito `+`.

Isso impede que o EKF interprete um movimento de ré como avanço para frente.


## Atualização da interface Web

A dashboard principal em `/` foi substituída por uma UI moderna, clara e responsiva.

Correções aplicadas nesta revisão:

- a página não depende mais de `dashboard.css` e `dashboard.js` externos; CSS e JavaScript foram embutidos no próprio `dashboard.html`, evitando tela quebrada quando o ESP32 não serve arquivos estáticos;
- removido visual escuro/LCARS/Star Trek;
- reduzido o título gigante que ocupava a tela;
- os controles foram reorganizados em cards;
- o comando principal continua sendo apenas `Vá 1 célula para frente`;
- o mapa 7x7 agora aparece em um painel centralizado, com legenda e células arredondadas.

Depois de gravar o firmware, envie novamente os arquivos da pasta `data/` para o SPIFFS/LittleFS. No PlatformIO, use:

```bash
pio run -t uploadfs
```

## Ajuste de giro após testes físicos

Nos testes reais, foi identificado que o carrinho estava girando aproximadamente o dobro do ângulo solicitado. Para estabilizar a execução, a PonteH agora aplica `TURN_COMMAND_SCALE = 0.50` sobre o alvo interno de giro. Assim, os comandos de alto nível continuam usando 90°, 45° etc., mas o controle de motor corta o giro pela metade.

Esse ajuste fica em `src/carrinho.ino` e pode ser recalibrado em incrementos pequenos, por exemplo `0.45`, `0.50`, `0.55`.


## Correções pós-log de 2026-05-28

Esta revisão corrige os problemas observados no log de teste:

- giro à esquerda acumulando ângulo negativo e aumentando o erro até timeout;
- primeira célula registrando distância absurda, como 1,226 m para alvo de 0,30 m;
- pose/célula do mapa sendo atualizada no meio da célula por ruído do EKF;
- falta de proteção quando a distância medida da célula passa muito do alvo esperado.

As principais constantes ficam em `src/carrinho.ino`:

```cpp
GYRO_Z_LEFT_TURN_SIGN
GYRO_Z_RIGHT_TURN_SIGN
CELL_OVERRUN_ABORT_FACTOR
ODOM_WHEEL_RADIUS_M
ODOM_TICKS_PER_REV
```

Para validar, faça primeiro testes isolados:

1. `{"action":"forward_cells","cells":1}` — deve registrar aproximadamente `0.30 m`.
2. `left` ou `right` — o `Turning Angle Z` deve crescer positivo até o alvo interno, sem ficar negativo.
3. `{"action":"path_to","x":5,"y":5,"execute":true}` — o caminho não deve mais entrar em timeout de giro repetidamente.
