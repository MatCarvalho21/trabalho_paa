# Trabalho de Projeto e Análise em Algoritmos
Repositório destinado ao trabalho relacionado a A2 da disciplina de Projeto e Análise de Algorítimos.

[Descrição do Trabalho](./data/descricao-trabalho.pdf)

# Modo de usar:

Para exemplificar o problema em um caso real, usamos como base o bairro L'Esquerra de l'Eixample de Barcelona, a forma de coleta pode ser encontrada em [Coleta do bairro](./data/create_mapa.ipynb), deste modo, para executar, primeiro, observe o mapa a seguir, o número do vértice de partida e o número do vértice de destino:

![](./barcelona_map.png)

Após selecionado, deverá ser executado o seguinte comando:

```bash
$ g++ main.cpp -o main.out && ./main.out
```

Assim, o usuário deverá inserir os dois vértices conforme solicitado, além de sua verba, e o algoritmo retornará o caminho percorrido, em junção com o meio de locomoção.

# Estrutura dos Módulos do Projeto

## **Algoritmos.cpp/.h**
Este módulo contém as implementações finais dos algoritmos principais do projeto. Cada tarefa é resolvida por um algoritmo específico:
- **Tarefa 1**: `subway`
- **Tarefa 2**: `bus`
- **Tarefa 3**: `melhorRota` (executa o algoritmo `dijkstra_custo`).

## **AlgoritmosBase.cpp/.h**
Módulo dedicado à implementação de algoritmos auxiliares, responsáveis por etapas intermediárias na resolução dos problemas. Um exemplo é o **`nearestNeighbor`**, que utiliza a heurística dos vizinhos mais próximos para encontrar um ciclo mínimo.

## **Estrutura.cpp/.h**
Este módulo contém todas as estruturas base do projeto, como:
- **`Planta`**
- **`Segmento`**

Além disso, ele inclui funções para inicialização/construção dessas estruturas e variáveis globais que regulam o funcionamento do código.

## **Main.cpp**
É o módulo responsável pela interface principal do programa. O usuário, após clonar o repositório, deve seguir as instruções fornecidas para testar as funcionalidades desenvolvidas.

## **Mapa.cpp/.h**
Módulo auxiliar que implementa funções específicas para a construção do grafo de Barcelona. Este grafo é utilizado no arquivo `main` para testar os algoritmos em um cenário real.

## **MapaRandom.cpp/.h**
Este módulo contém funções para a criação de `Plantas` aleatórias, utilizadas na análise de tempos de execução dos algoritmos.

## **times.cpp**
Módulo responsável por executar testes de tempo de execução dos três algoritmos principais. Os resultados são salvos em um arquivo **`times.csv`**, posteriormente usados na construção de gráficos e análise da complexidade das tarefas (detalhada no relatório).

## **utils.cpp/.h**
Módulo auxiliar com funções de suporte à execução principal. Dentre as funções implementadas, destaca-se **`printSegmento`**, que exibe uma instância de `Segmento` no console.

# Detalhes Adicionais sobre a Implementação

## **Tempo de Espera**
O tempo de espera foi adicionado de forma aleatória, considerando um intervalo determinado. Por exemplo:
- Para os ônibus: se eles passam a cada 10 minutos, é gerado um incremento entre **0** e **1** no tempo total toda vez que o algoritmo **Dijkstra** decide usar um ônibus.
- Para o metrô: o mesmo conceito é aplicado, mas com um tempo de espera diferente, específico ao contexto do metrô.

## **Trânsito**
A simulação de trânsito é implementada de forma aleatória:
1. Para cada segmento, é gerado um **multiplicador** aleatório a partir de uma distribuição uniforme no intervalo **(0.2, 1)**.
2. Uma função auxiliar ajusta os valores na `Planta`:
   - O campo **`limVel`** de cada `Segmento` é atualizado multiplicando seu valor original pelo multiplicador gerado.

