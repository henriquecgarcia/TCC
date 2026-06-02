# RESULTS.md — Registro de resultados experimentais

Use este arquivo para consolidar os dados medidos durante os testes do TCC.

## 1. Precisão de deslocamento

| Ensaio | Comando | Esperado cm | Medido cm | Erro cm | Erro % |
|---|---|---:|---:|---:|---:|
| 1 | 1 célula | 50 |  |  |  |
| 2 | 2 células | 100 |  |  |  |
| 3 | 3 células | 150 |  |  |  |
| 4 | 5 células | 250 |  |  |  |

## 2. Erro angular

| Ensaio | Células | Erro máximo ° | Erro final ° | Correções residuais | Observação |
|---|---:|---:|---:|---:|---|
| 1 | 1 |  |  |  |  |
| 2 | 3 |  |  |  |  |
| 3 | 5 |  |  |  |  |

## 3. A* e replanejamento

| Ensaio | Alvo | Obstáculo dinâmico | Replanejou | Chegou ao alvo | Observação |
|---|---|---|---|---|---|
| 1 |  | Não |  |  |  |
| 2 |  | Sim |  |  |  |
| 3 |  | Sim |  |  |  |

## 4. Dados brutos

Anexe ou referencie os arquivos baixados de:

```txt
/api/tcc-log.csv
```

Sugestão de análise posterior:

- importar o CSV no Python/pandas;
- gerar gráfico de `heading_error_rad` por tempo;
- gerar gráfico de `travelled_cell_m` por célula;
- comparar eventos `cell_completed`, `obstacle_replanned` e `aborted`.


## Correção de drift parado e célula de 30 cm

Nesta revisão, a célula de navegação foi reduzida para **30 cm x 30 cm**. Também foi adicionada uma trava de odometria parada: quando a PonteH não está comandando movimento físico, o EKF não integra predição por encoder nem correção por giroscópio. Em vez disso, ele sincroniza os ticks atuais como nova referência interna.

Essa mudança evita que ruídos elétricos, pequenos pulsos espúrios de encoder ou deriva do MPU6050 façam a posição discreta avançar no mapa enquanto o robô está parado.

### Como validar

1. Ligue o robô e deixe parado por 2 minutos.
2. Abra a Web UI e observe `location.grid_x`, `location.grid_y`, `world_x_m` e `world_y_m`.
3. A célula deve permanecer em `(1,1)` e a posição em metros não deve avançar sozinha.
4. Execute `{"action":"forward_cells","cells":1}`.
5. O robô deve andar aproximadamente 30 cm, parar, corrigir heading e registrar a nova célula.
