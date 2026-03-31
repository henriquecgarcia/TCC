# PATTERNS (PlatformIO)

Este arquivo define padroes de projeto para manter consistencia, desempenho e seguranca no firmware do carrinho (ESP32 + Arduino).

## 1. Estrutura de pastas

- `src/`: logica principal da aplicacao (`carrinho.ino`)
- `lib/<Modulo>/`: modulos isolados por responsabilidade
- `include/`: headers compartilhados e constantes globais
- `data/`: arquivos estaticos para SPIFFS (dashboard, monitor)
- `extra_scripts/`: scripts de build/deploy

Padrao recomendado de modulo:

- `lib/NomeModulo/NomeModulo.h`
- `lib/NomeModulo/NomeModulo.cpp`

## 2. Design de classes (embarcado)

- uma classe por responsabilidade principal
- evitar dependencia circular entre modulos
- evitar alocacoes dinamicas frequentes no `loop()`
- preferir estruturas fixas e compactas
- expor API pequena e clara

## 3. Convencoes de codigo

- nome de classes: `PascalCase` (ex: `Map`, `PowerManager`)
- nome de metodos/variaveis: `camelCase` (ex: `setCell`, `currentX`)
- constantes: `UPPER_SNAKE_CASE` quando globais
- nomes de arquivo seguem nome da classe principal

## 4. Erros e seguranca

- toda API publica deve validar parametros de entrada
- coordenadas fora de limite devem falhar com retorno seguro
- toda operacao de arquivo deve validar abertura, leitura e escrita
- em falha de hardware/IO, retornar `false` e registrar no `Serial` quando util

## 5. Memoria e desempenho

- preferir representacoes compactas (bitfield para mapas binarios)
- evitar `String` em fluxos criticos (telemetria frequente)
- evitar `std::vector` e containers pesados em codigo de tempo real
- priorizar buffers alocados uma vez e reutilizados

## 6. Padrao para persistencia (SPIFFS)

- chamar `begin()` de storage no setup
- usar formato binario com cabecalho versionado
- validar `magic`, `version` e tamanho antes de carregar
- escrever de forma atomica quando possivel (arquivo temporario + rename)

## 7. Padrao para pathfinding

- usar grade ocupada/livre (`0`/`1`)
- usar A* com heuristica Manhattan para 4-direcoes
- recalcular rota somente quando:
  - destino mudou
  - obstaculo relevante foi atualizado
  - posicao atual divergiu da rota

## 8. Padrao para controle de loop

- evitar blocos longos no `loop()`
- usar timers por `millis()` para tarefas periodicas
- separar ciclos de:
  - leitura de sensores
  - controle de motores
  - comunicacao web/ws
  - planejamento de rota

## 9. Testabilidade

- isolar logica pura em classes de biblioteca (`lib/`)
- manter integracao de hardware no `src/` ou adaptadores
- criar funcoes pequenas com retorno verificavel

## 10. Checklist de PR/alteracao

- compilou sem erros
- validou limites de entradas
- nao introduziu bloqueio desnecessario no `loop()`
- documentou uso no README do modulo
- manteve compatibilidade com ESP32 DOIT DEVKIT V1
