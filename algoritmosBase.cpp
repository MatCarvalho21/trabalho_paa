/**
 * @file algoritmosBase.cpp
 * @brief Módulo para definição das funções dos algoritmos base de grafos.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <limits>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <utility>

#include "algoritmosBase.h"
#include "estrutura.h"
#include "mapaRandom.h"
#include "algoritmos.h"
#include "utils.h"

using namespace std;


/// @brief Executa o algoritmo de Dijkstra em uma planta com vértice inicial definido.
/// @param v0 Vértice inicial
/// @param parents Vetor de pais.
/// @param distances Vetor de distâncias.
/// @param numVertices Número de vértices da planta
/// @param planta Planta a ser percorrida.
void dijkstra(int v0, vector<int>& parents, vector<int>& distances, int numVertices, Planta* planta)
{
    // Vetor de verificação que marca se um vértice já foi processado
    bool checked[numVertices];

    // Definição de uma fila de prioridade que armazena tuplas (distância, vértice),
    // onde a fila é organizada de acordo com a distância crescente
    priority_queue<tuple<int, int>, vector<tuple<int, int>>, greater<tuple<int, int>>> heap;

    // Inicializando os vetores parents, distances e checked
    for (int i = 0; i < numVertices; i++)
    {
        // Inicializando os pais como -1
        parents[i] = -1;
        // Inicializando as distâncias como infinito
        distances[i] = INT_MAX; 
        // Marcando todos os vértices como não verificados
        checked[i] = false; 
    }

    // Configurando o vértice de origem
    parents[v0] = v0;
    distances[v0] = 0;

    // Adicionando o vértice de origem à fila de prioridade
    heap.push(make_tuple(distances[v0], v0));

    // Enquanto o heap não estiver vazio...
    while (!heap.empty())
    {
        // Obtém o vértice com a menor distância da fila
        int v1 = get<1>(heap.top());
        // Obtém sua distância
        int currentDist = get<0>(heap.top());
        // Remove o vértice da fila
        heap.pop(); 

        // Se a distância do vértice extraído for maior que a distância conhecida, ignora
        if (currentDist > distances[v1])
        {
            continue;
        }

        // Se a distância do vértice for infinita, significa que não há mais vértices alcançáveis
        if (distances[v1] == INT_MAX)
        {
            break;
        }

        // Obtém a lista de arestas adjacentes ao vértice atual
        vector<Segmento*> edges = (planta -> listaAdj)[v1];

        // Para cada aresta de saída desse vértice...
        for (int i = 0; i < edges.size(); i++)
        {
            // Pega a aresta atual
            Segmento* edge = edges[i];

            // Pega o vértice de destino da aresta
            int v2 = edge -> vEntrada;

            // Se o vértice de destino ainda não foi processado...
            if (!checked[v2])
            {
                // Pega o custo da aresta
                int cost = edge -> tamanho; 

                // Se o caminho através do vértice v1 oferece uma distância menor para v2...
                if (distances[v1] + cost < distances[v2])
                {
                    // Atualiza o pai de v2
                    parents[v2] = v1; 
                    // Atualiza a distância de v2
                    distances[v2] = distances[v1] + cost; 
                    // Adiciona v2 à fila de prioridade
                    heap.push(make_tuple(distances[v2], v2)); 
                }
            }
        }

        // Marca o vértice v1 como processado
        checked[v1] = true;
    }
}

/// @brief Função para calcular o peso de um segmento com base nos tipos de imóveis.
/// @param start Vértice inicial.
/// @param parents Vetor de pais.
/// @param numVertices Número de vértices da planta.
/// @param planta Planta a ser percorrida.
void primMST(int start, vector<int>& parents, int numVertices, Planta* planta)
{
    // Criando os vetores de custo mínimo e de se está na árvore
    vector<int> minEdgeCost(numVertices, INT_MAX);
    vector<bool> inTree(numVertices, false);

    // Fila de prioridade que armazena tuplas (peso, vértice)
    priority_queue<tuple<int, int>, vector<tuple<int, int>>, greater<tuple<int, int>>> heap;

    for (int i = 0; i < numVertices; i++) {parents[i] = -1;}

    // Configurando o vértice inicial
    parents[start] = start;
    minEdgeCost[start] = 0;
    heap.push(make_tuple(0, start)); 

    while (!heap.empty())
    {
        // Obtém o vértice com menor custo de aresta para a árvore
        int v1 = get<1>(heap.top());
        heap.pop();

        // Se o vértice já está na árvore, ignora
        if (inTree[v1]) continue;

        // Marca o vértice como incluído na árvore
        inTree[v1] = true;

        // Obtém a lista de arestas adjacentes ao vértice atual
        vector<Segmento*> edges = planta->listaAdj[v1];

        // Para cada aresta adjacente...
        for (int i = 0; i < edges.size(); i++)
        {
            Segmento* edge = edges[i];
            int v2 = edge->vEntrada;
            int cost = edge->tamanho;

            // Se v2 ainda não está na árvore e o custo da aresta é menor que o custo conhecido...
            if (!inTree[v2] && cost < minEdgeCost[v2])
            {
                // Atualiza o custo e o pai de v2
                minEdgeCost[v2] = cost;
                parents[v2] = v1;

                // Adiciona v2 à fila de prioridade
                heap.push(make_tuple(minEdgeCost[v2], v2));
            }
        }
    }
}

/// @brief Função para calcular o peso de um segmento com base nos tipos de imóveis.
/// @param segmento Estrutura Segmento contendo o vetor de imóveis.
/// @return Peso calculado como (turísticos + comerciais) - (residenciais + industriais).
int calcula_peso(const Segmento* segmento) {
    int comerciais = 0;
    int industriais = 0;
    int turisticos = 0;
    int residenciais = 0;

    for (Imovel* imovel : segmento->imoveis) {
        if (imovel->tipo == "comercial") {
            comerciais++;
        } else if (imovel->tipo == "industrial") {
            industriais++;
        } else if (imovel->tipo == "turismo") {
            turisticos++;
        } else if (imovel->tipo == "residencial") {
            residenciais++;
        }
    }

    return (turisticos + comerciais) - (residenciais + industriais);
}

/// @brief Função para construir um grafo virtual ajustando os pesos dos segmentos e identificar os vértices de borda.
/// @param planta Ponteiro para a estrutura Planta representando o grafo original.
/// @param limiar Inteiro representando o limiar base para ajuste dos pesos dos segmentos.
/// @return Um par contendo o ponteiro para a nova Planta virtual e o conjunto de vértices de borda.
pair<Planta*, set<int>> construir_grafo_virtual(Planta* planta, int limiar) {
    // Conjunto para armazenar os vértices de borda
    int num_vertices = planta->listaAdj.size();
    set<int> vertices_borda;
    vector<set<int>> set_aux_ceps (num_vertices);
    // Número de vértices na planta

    // Cria a planta virtual (grafo virtual)
    Planta* planta_virtual = newPlanta(num_vertices);

    // Itera sobre os vértices da planta original
    for (int i = 0; i < num_vertices; i++) {
        vector<Segmento*> lista_vizinhos = planta->listaAdj[i]; // Lista de adjacência do vértice i

        // Itera sobre os segmentos (arestas) de saída do vértice i
        for (Segmento* temp_node : lista_vizinhos) {
            set_aux_ceps[temp_node->vSaida].insert(temp_node->CEP);
            set_aux_ceps[temp_node->vEntrada].insert(temp_node->CEP);

            // Calcula o novo peso ajustado
            int novo_peso = calcula_peso(temp_node);

            // Cria um novo segmento com o peso ajustado
            Segmento* temp_segmento = newSegmento(
                temp_node->vSaida,       // Vértice de saída
                temp_node->vEntrada,     // Vértice de entrada
                temp_node->limVel,       // Limite de velocidade
                limiar + novo_peso,     // Peso ajustado com o limiar
                temp_node->CEP,          // CEP (identificador da rua/região)
                temp_node->rua,          // Nome da rua
                temp_node->dupla         // Dupla direção (se aplicável)
            );

            temp_segmento->imoveis = temp_node->imoveis;

            // Adiciona o novo segmento ao grafo virtual (ajusta a lista de adjacência)
            adicionaSegmentoAPlanta(temp_segmento, planta_virtual);
        }
    }

    for (int i = 0; i < num_vertices; i++)
    {
        if (set_aux_ceps[i].size() > 1) { vertices_borda.insert(i); }
    }

    // Retorna o grafo virtual e o conjunto de vértices de borda
    return {planta_virtual, vertices_borda};
}


/// @brief Aplica o algoritmo de Dijkstra para calcular distâncias mínimas dentro de uma região específica.
/// @param planta Ponteiro para a estrutura Planta representando o grafo da região.
/// @param origem Inteiro representando o vértice inicial para o cálculo das distâncias.
/// @param cep_regiao Inteiro identificador da região para restringir os cálculos.
/// @return Um par contendo um vetor com as distâncias mínimas e um vetor de predecessores para reconstrução dos caminhos mínimos.
pair<vector<int>, vector<int>> dijkstra_regional(Planta* planta, int origem, int cep_regiao)
{
    int num_vertices = planta->listaAdj.size();
    vector<int> distancias(num_vertices, numeric_limits<int>::max());
    vector<bool> visitados(num_vertices, false);
    vector<int> anteriores(num_vertices, -1);

    distancias[origem] = 0;

    while (true)
    {
        int menor_distancia = numeric_limits<int>::max();
        int vertice_atual = -1;

        // Encontrar o vértice com a menor distância não visitado
        for (int i = 0; i < num_vertices; i++)
        {
            if (!visitados[i] && distancias[i] < menor_distancia)
            {
                menor_distancia = distancias[i];
                vertice_atual = i;
            }
        }

        if (vertice_atual == -1 || menor_distancia == numeric_limits<int>::max()) { break; } // Todos os vértices foram visitados

        // Explorar os vizinhos do vértice atual
        for (Segmento* segmento : planta->listaAdj[vertice_atual])
        {
            if (segmento->CEP != cep_regiao)
                continue;

            int vizinho = segmento->vEntrada; // Considerando o vEntrada como o vizinho
            int peso_aresta = segmento->tamanho; // Usando a limitação de velocidade como o peso da aresta

            if (!visitados[vizinho])
            {
                int nova_distancia = distancias[vertice_atual] + peso_aresta;
                if (nova_distancia < distancias[vizinho])
                {
                    distancias[vizinho] = nova_distancia;
                    anteriores[vizinho] = vertice_atual;
                }
            }
        }

        visitados[vertice_atual] = true;
    }

    return make_pair(distancias, anteriores);
}

/// @brief Encontra os vértices ótimos que minimizam a média das distâncias aos vértices de borda em uma região específica.
/// @param planta Ponteiro para a estrutura Planta representando o grafo da região.
/// @param verticesBorda Conjunto de vértices de borda do grafo.
/// @param cepRegiao Inteiro identificador da região para restringir os cálculos.
/// @return Vetor de inteiros contendo os índices dos vértices ótimos.
int encontrarVerticeOtimo(Planta* planta, const set<int>& verticesBorda, int cepRegiao)
{
    int numVertices = planta->listaAdj.size();
    int verticeOtimo = -1;
    float menorMediaDistancias = numeric_limits<float>::max();

    for (int vertice = 0; vertice < numVertices; vertice++)
    {
        pair<vector<int>, vector<int>> distParent = dijkstra_regional(planta, vertice, cepRegiao);
        vector<int> distancias = distParent.first;

        // Obter as distâncias apenas dos vértices de borda
        vector<int> distanciasBorda;
        for (int borda : verticesBorda)
        {
            if (distancias[borda] < numeric_limits<int>::max() && distParent.second[borda] != -1)
            {
                distanciasBorda.push_back(distancias[borda]);
            }
        }

        // Ignorar se não houver distâncias válidas
        if (distanciasBorda.empty()) { continue; }

        // Calcular a média das distâncias
        float mediaDistancias = std::accumulate(distanciasBorda.begin(), distanciasBorda.end(), 0.0f) / distanciasBorda.size();

        // Atualizar os vértices ótimos se encontrarmos uma menor média
        if (mediaDistancias < menorMediaDistancias)
        {
            menorMediaDistancias = mediaDistancias;
            verticeOtimo = vertice;
        }
    }

    return verticeOtimo;
}

/// @brief Encontra os vértices ótimos para todas as regiões especificadas.
/// @param planta Ponteiro para a estrutura Planta representando o grafo da região.
/// @param verticesBorda Conjunto de vértices de borda do grafo.
/// @return Vetor de vetores, onde cada vetor interno contém os índices dos vértices ótimos de uma região.
set<int> achaVerticesRegionais(Planta* planta, const set<int>& verticesBorda)
{
    set<int> verticesOtimosPorRegiao;

    // Iterar diretamente sobre os CEPs do set na estrutura Planta
    for (int cep : planta->CEPs)
    {
        // Chama a função encontrarVerticeOtimo para cada CEP
        int verticeOtimo = encontrarVerticeOtimo(planta, verticesBorda, cep);

        // Adiciona os vértices ótimos dessa região à lista final
        if (verticeOtimo != -1) { verticesOtimosPorRegiao.insert(verticeOtimo); }
    }

    return verticesOtimosPorRegiao;
}

/// @brief Executa o algoritmo de Dijkstra para encontrar as menores distâncias e os caminhos mais curtos.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @param origem Índice do vértice de origem.
/// @return Um par contendo:
///         - Um vetor de distâncias mínimas do vértice de origem para todos os outros.
///         - Um vetor de predecessores usado para reconstruir os caminhos.
pair<vector<int>, vector<int>> dijkstra_normal(Planta* planta, int origem)
{
    int num_vertices = planta->listaAdj.size();
    vector<int> distancias(num_vertices, numeric_limits<int>::max());
    vector<bool> visitados(num_vertices, false);
    vector<int> anteriores(num_vertices, -1);

    distancias[origem] = 0;

    while (true)
    {
        int menorDistancia = numeric_limits<int>::max();
        int verticeAtual = -1;

        for (int i = 0; i < num_vertices; i++)
        {
            if (!visitados[i] && distancias[i] < menorDistancia)
            {
                menorDistancia = distancias[i];
                verticeAtual = i;
            }
        }

        if (verticeAtual == -1 || menorDistancia >= numeric_limits<int>::max()) { break; } // Todos os vértices foram visitados


        for (Segmento* segmento : planta->listaAdj[verticeAtual])
        {
            int vizinho = segmento->vEntrada;
            int peso = segmento->tamanho;

            if (!visitados[vizinho])
            {
                int novaDistancia = distancias[verticeAtual] + peso;
                if (novaDistancia < distancias[vizinho])
                {
                    distancias[vizinho] = novaDistancia;
                    anteriores[vizinho] = verticeAtual;
                }
            }
        }
        visitados[verticeAtual] = true;
    }

    return {distancias, anteriores};
}

/// @brief Constrói um grafo completo conectando vértices ótimos, onde o peso das arestas é a distância mínima calculada pelo Dijkstra.
/// @param plantaOriginal Ponteiro para a planta original contendo todos os segmentos e vértices.
/// @param verticesOtimos Vetor de vértices ótimos identificados.
/// @return Ponteiro para o grafo completo das regiões (planta virtual).
pair<Planta*, vector<vector<int>>> construirGrafoRegioes(Planta* planta, set<int> verticesRegioes)
{
    int numVertices = planta->listaAdj.size();
    // Criar a planta virtual (grafo da região)

    Planta* plantaRegioes = newPlanta(numVertices);
    vector<vector<int>> listaPredecessores(numVertices);

    // Iterar sobre todos os pares de vértices ótimo
    for (int verticeOtimo1 : verticesRegioes)
    {
        pair<vector<int>, vector<int>> resultado = dijkstra_normal(planta, verticeOtimo1);
        vector<int> distancias = resultado.first;
        vector<int> predecessores = resultado.second;
        
        listaPredecessores[verticeOtimo1] = predecessores;

        for (int verticeOtimo2 : verticesRegioes)
        {
            if (verticeOtimo1 == verticeOtimo2) { continue; }

            Segmento* segmentoVirtual = newSegmento(verticeOtimo1, verticeOtimo2, 0, distancias[verticeOtimo2], 0, "Virtual", false);
            adicionaSegmentoAPlanta(segmentoVirtual, plantaRegioes);
        }
    }

    return make_pair(plantaRegioes, listaPredecessores);
}

/// @brief Aplica a heurística do vizinho mais próximo para encontrar um ciclo no grafo.
/// @param plantaRegioes Ponteiro para a estrutura Planta que representa o grafo das regiões, com as listas de adjacências.
/// @param verticeInicial Inteiro representando o vértice inicial do ciclo. O valor padrão é 0.
/// @return Um par contendo:
///         - Um vetor de inteiros representando o ciclo encontrado, começando e terminando no vértice inicial.
///         - Um vetor de ponteiros para os segmentos (arestas) que compõem o ciclo, na ordem em que são visitados.
vector<int> nearestNeighbor(Planta* plantaRegioes, int verticeInicial = 0)
{
    int numVertices = plantaRegioes->listaAdj.size();
    vector<int> ciclo;
    ciclo.push_back(verticeInicial);
    vector<bool> visitados(numVertices, false);

    int verticeAtual = verticeInicial;
    visitados[verticeAtual] = true;

    while (true)
    {
        vector<Segmento*> listaAdjAtual = plantaRegioes->listaAdj[verticeAtual];
        float menorPeso = numeric_limits<float>::infinity();
        int proximoVertice = -1;

        for (Segmento* segmento : listaAdjAtual)
        {
            int vizinho = segmento->vEntrada; // Considerando o vértice de entrada como vizinho
            float peso = segmento->tamanho;   // Usando tamanho como peso da aresta

            if (!visitados[vizinho] && peso < menorPeso)
            {
                menorPeso = peso;
                proximoVertice = vizinho;
            }
        }

        if (proximoVertice == -1)
        {
            break; // Não há mais vizinhos não visitados
        }

        ciclo.push_back(proximoVertice);
        verticeAtual = proximoVertice;
        visitados[verticeAtual] = true;
    }

    ciclo.push_back(verticeInicial); // Retorna ao vértice inicial para formar o ciclo
    return ciclo;
}


/// @brief Gera uma matriz de adjacência a partir da lista de adjacência de uma planta.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @return Uma matriz de adjacência (vetor de vetores) com os pesos das arestas.
vector<vector<int>> gerarMatrizAdjacencia(Planta* planta)
{
    int numVertices = planta->listaAdj.size();
    // Inicializar a matriz de adjacência com infinito (indicando ausência de arestas)
    vector<vector<int>> matriz(numVertices, vector<int>(numVertices, numeric_limits<int>::max()));

    // Preencher a matriz com os pesos das arestas a partir da lista de adjacência
    for (int i = 0; i < numVertices; i++)
    {
        for (Segmento* segmento : planta->listaAdj[i])
        {
            int vizinho = segmento->vEntrada;
            float peso = segmento->tamanho; // Exemplo: usar o tamanho como peso

            matriz[i][vizinho] = peso; // Define o peso da aresta de i para vEntrada
            if (segmento->dupla)
            {
                matriz[vizinho][i] = peso; // Se o segmento for bidirecional, adiciona também a aresta reversa
            }
        }
    }

    return matriz;
}

/// @brief Calcula o custo total de um ciclo em um grafo direcionado.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @param ciclo Vetor de inteiros representando o ciclo (sequência de vértices).
/// @return Um par contendo o custo total na ordem de ida e na ordem de volta.
pair<int, int> calcularCustoDirecionado(const vector<vector<int>> matrizAdj, const vector<int>& ciclo)
{
    int custoIda = 0, custoVolta = 0;
    int n = ciclo.size();

    for (int i = 0; i < n - 1; ++i)
    {
        int j = n - 1 - i;

        int ida = matrizAdj[ciclo[i]][ciclo[i + 1]];
        int volta = matrizAdj[ciclo[j]][ciclo[j - 1]];

        custoIda += ida;
        custoVolta += volta;
    }
    custoIda += matrizAdj[ciclo[n - 1]][ciclo[0]];
    custoVolta += matrizAdj[ciclo[0]][ciclo[n - 1]];

    return make_pair(custoIda, custoVolta);
}

/// @brief Otimiza um ciclo direcionado em um grafo usando a técnica Two-Opt.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @param cicloInicial Vetor de inteiros representando o ciclo inicial.
/// @return Um par contendo o ciclo otimizado e o menor custo total do ciclo.
pair<vector<int>, int> twoOptDirected(Planta* planta, const vector<int>& cicloInicial)
{
    int n = cicloInicial.size() - 1;
    vector<vector<int>> matrizAdj = gerarMatrizAdjacencia(planta);
    vector<int> melhorCiclo;
    for (int i = 0; i < n; ++i) { melhorCiclo.push_back(cicloInicial[i]); }

    pair<int, int> custos = calcularCustoDirecionado(matrizAdj, cicloInicial);
    int melhorCustoIda = custos.first;
    int melhorCustoVolta = custos.second;

    int melhorCusto = min(melhorCustoIda, melhorCustoVolta);
    bool melhorado = true;

    if (n < 3) { return make_pair(melhorCiclo, melhorCusto); }

    while (melhorado)
    {
        melhorado = false;
        for (int i = 0; i < n - 2; ++i)
        {
            for (int j = i + 2; j < n; ++j)
            {
                vector<int> novoCiclo = melhorCiclo;

                int temp = novoCiclo[j];
                novoCiclo[j] = novoCiclo[i + 1];
                novoCiclo[i + 1] = temp;

                novoCiclo.push_back(novoCiclo[0]);
                pair<int, int> custos = calcularCustoDirecionado(matrizAdj, novoCiclo);
                novoCiclo.pop_back();

                int novoCustoIda = custos.first;
                int novoCustoVolta = custos.second;

                int novoCusto = min(novoCustoIda, novoCustoVolta);

                if (novoCusto < melhorCusto)
                {
                    melhorCiclo = novoCiclo;
                    melhorCusto = novoCusto;
                    melhorado = true;
                    if (novoCustoVolta < novoCustoIda)
                    {
                        reverse(melhorCiclo.begin(), melhorCiclo.end());
                    }
                }
            }
        }
    }
    melhorCiclo.push_back(melhorCiclo[0]);
    return make_pair(melhorCiclo, melhorCusto);
}

/// @brief Vai construi uma MST com base nos seus segmentos.
/// @param mstMetroSeg Vetor de ponteiros para Segmento representando a MST do metrô.
/// @param numVertices Inteiro representando o número de vértices do grafo.
/// @param estacoesMetro Conjunto de inteiros representando os vértices que são estações de metrô.
/// @return Ponteiro para a Planta representando a MST do metrô.
Planta* refazMst(vector<Segmento*> mstMetroSeg, int numVertices, set<int> estacoesMetro)
{
    Planta* mstMetro = newPlanta(numVertices);

    for (Segmento* segmento : mstMetroSeg)
    {
        segmento->dupla = true;
        adicionaSegmentoAPlanta(segmento, mstMetro);
        Segmento* segmentoInverso = newSegmento(
            segmento->vEntrada,
            segmento->vSaida,
            segmento->limVel,
            segmento->tamanho,
            segmento->CEP,
            segmento->rua,
            segmento->dupla
        );
        adicionaSegmentoAPlanta(segmentoInverso, mstMetro);
    }

    return mstMetro;
}

// O((V + E) * log(V))
/// @brief Executa o algoritmo de Dijkstra em uma planta com vértice inicial definido.
/// @param mstMetro Ponteiro para a estrutura Planta representando a MST do metrô.
/// @param origem Inteiro representando o vértice inicial para o cálculo das distâncias.
/// @return Um par contendo um vetor com as distâncias mínimas e um vetor de predecessores para reconstrução dos caminhos mínimos.
pair<vector<int>, vector<int>> dijkstraMetro(Planta* mstMetro, int origem)
{
    int numVertices = mstMetro->listaAdj.size();

    vector<int> distancias(numVertices, INT_MAX);
    vector<int> predecessores(numVertices, -1); // Para reconstruir o caminho
    
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> filaPrioridade;
    
    distancias[origem] = 0;
    filaPrioridade.push({0, origem});

    while (!filaPrioridade.empty())
    {
        pair<int, int> topo = filaPrioridade.top();
        int distAtual = topo.first;
        int verticeAtual = topo.second;

        filaPrioridade.pop();

        if (distAtual > distancias[verticeAtual]) { continue; }

        vector<Segmento*> segmentos = mstMetro->listaAdj[verticeAtual];
        for (Segmento* segmento : segmentos)
        {
            int vizinho = segmento->vEntrada;
            int peso = segmento->tamanho;

            if (distancias[verticeAtual] + peso < distancias[vizinho])
            {
                distancias[vizinho] = distancias[verticeAtual] + peso;
                predecessores[vizinho] = verticeAtual;
                filaPrioridade.push({distancias[vizinho], vizinho});
            }
        }
    }
    return {distancias, predecessores};
}

// O(V^2 * log(V))
/// @brief Encontra as arestas entre as estações de metrô.
/// @param mstMetro Ponteiro para a estrutura Planta representando a MST do metrô.
/// @param estacoesMetroSet Conjunto de inteiros representando os vértices que são estações de metrô.
/// @return Vetor de pares de pares de inteiros representando as arestas entre as estações de metrô e o peso das arestas.
vector<pair<pair<int, int>, int>> achaArestasMetro(Planta* mstMetro, set<int> estacoesMetroSet)
{
    vector<pair<pair<int, int>, int>> arestasMetro;

    for (int estacao1 : estacoesMetroSet)
    {
        pair<vector<int>, vector<int>> resultado = dijkstraMetro(mstMetro, estacao1);
        vector<int> distancias = resultado.first;
        vector<int> predecessores = resultado.second;

        for (int estacao2 : estacoesMetroSet)
        {
            if (estacao1 == estacao2) { continue; }
            arestasMetro.push_back({{estacao1, estacao2}, distancias[estacao2]});
        }
    }
    return arestasMetro;
}

// O(E)
pair<int, double> calculaDistTemp(Planta* planta, int vOrigem, int vDestino)
{
    vector<Segmento*> segmentos = planta->listaAdj[vOrigem];
    for (Segmento* segmento : segmentos)
    {
        if (segmento->vEntrada == vDestino)
        {
            double limite = segmento->limVel / normalizacao;
            return {segmento->tamanho, (segmento->tamanho / limite) * segmento->transito};
        }
    }
    
    return {0, 0};
}

// O(V+E)
/// @brief Calcula as distâncias e tempos de um ciclo em uma planta.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @param ciclo Vetor de inteiros representando o ciclo.
/// @param start Inteiro representando o vértice inicial do ciclo.
/// @return Vetor de pares de inteiros e doubles representando as distâncias e tempos do ciclo.
vector<pair<int, double>> calculaDistTempoCiclo(Planta* planta, vector<int> ciclo, int start)
{
    int n = ciclo.size();
    vector<pair<int, double>> distanciasTempos;
    distanciasTempos.resize(n);

    int startIndex = -1;
    for (int i = 0; i < n; i++)
    {
        if (ciclo[i] == start)
        {
            startIndex = i;
            break;
        }
    }
    distanciasTempos[startIndex] = {0, 0};

    int endIndex = startIndex - 1;
    if (endIndex < 0) { endIndex = n - 1; }

    for (int i = 0; i < n - 1; i++)
    {
        int nextIndex = (startIndex + i + 1) % n;
        int currentIndex = (startIndex + i) % n;

        vector<Segmento*> segmentos = planta->listaAdj[ciclo[currentIndex]];

        distanciasTempos[nextIndex].first = 0;
        distanciasTempos[nextIndex].second = 0;

        distanciasTempos[nextIndex].first += distanciasTempos[currentIndex].first;
        distanciasTempos[nextIndex].second += distanciasTempos[currentIndex].second;

        pair<int, double> distTemp = calculaDistTemp(planta, ciclo[currentIndex], ciclo[nextIndex]);

        distanciasTempos[nextIndex].first += distTemp.first;
        distanciasTempos[nextIndex].second += distTemp.second;
    }

    return distanciasTempos;
}

// O(V * ( V + E))
/// @brief Encontra as arestas entre os vértices de um ciclo de ônibus.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @param cicloBus Vetor de inteiros representando o ciclo de ônibus.
/// @return Vetor de pares de pares de inteiros e doubles representando as arestas entre os vértices de um ciclo de ônibus e o peso das arestas.
vector<pair<pair<int, int>, pair<int, double>>> achaArestasOnibus(Planta* planta, vector<int> cicloBus)
{
    vector<int> cicloTemp = cicloBus;
    cicloTemp.pop_back();

    vector<pair<pair<int, int>, pair<int, double>>> arestasOnibus;

    for (int i = 0; i < cicloTemp.size(); i++)
    {
        vector<pair<int, double>> distanciasTempos = calculaDistTempoCiclo(planta, cicloTemp, cicloTemp[i]);

        for (int j = 0; j < cicloTemp.size(); j++)
        {
            if (i == j) { continue; }
            arestasOnibus.push_back({{cicloTemp[i], cicloTemp[j]}, {distanciasTempos[j].first, distanciasTempos[j].second}});
        }
    }
    return arestasOnibus;
}

// O(V * ((V + E) + V * log(V)))
/// @brief Constrói uma planta de busca a partir de uma planta original.
/// @param planta Ponteiro para a estrutura Planta representando o grafo original.
/// @param cicloBus Vetor de inteiros representando o ciclo de ônibus.
/// @param mstSegs Vetor de ponteiros para Segmento representando a MST do metrô.
/// @param estacoesMetro Vetor de inteiros representando os vértices que são estações de metrô.
/// @return Ponteiro para a PlantaBusca representando a planta de busca.
PlantaBusca* constroiPlantaBusca(Planta* planta, vector<int> cicloBus, vector<Segmento*> mstSegs, vector<int> estacoesMetro)
{
    int nVertices = planta->listaAdj.size();
    PlantaBusca* plantaBusca = newPlantaBusca(nVertices * 2);
    // O(V + E)
    for (int i = 0; i < nVertices; i++)
    {
        vector<Segmento*> segmentos = planta->listaAdj[i];
        for (Segmento* segmento : segmentos)
        {
            SegmentoBusca* segmentoAndar = newSegmentoBusca(
                i,
                segmento->vEntrada,
                segmento->tamanho,
                segmento->tamanho / VelocidadeAndar,
                "andar"
            );

            SegmentoBusca* segmentoTaxi = newSegmentoBusca(
                i + nVertices,
                segmento->vEntrada + nVertices,
                segmento->tamanho,
                (segmento->tamanho / (segmento->limVel / normalizacao)) * segmento->transito,
                "taxi"
            );

            plantaBusca->adicionaSegmento(segmentoAndar);
            plantaBusca->adicionaSegmento(segmentoTaxi);

            SegmentoBusca* segmentoConexaoIda = newSegmentoBusca(
                i,
                i + nVertices,
                0,
                0,
                "taxi"
            );

            SegmentoBusca* segmentoConexaoVolta = newSegmentoBusca(
                i + nVertices,
                i,
                0,
                0,
                "andar"
            );
            segmentoConexaoIda->vertical = true;
            segmentoConexaoVolta->vertical = true;

            plantaBusca->adicionaSegmento(segmentoConexaoIda);
            plantaBusca->adicionaSegmento(segmentoConexaoVolta);

            if (segmento->dupla) { continue; }
            else
            {
                SegmentoBusca* segmentoAndar2 = newSegmentoBusca(
                    segmento->vEntrada,
                    i,
                    segmento->tamanho,
                    segmento->tamanho / VelocidadeAndar,
                    "andar"
                );
                plantaBusca->adicionaSegmento(segmentoAndar2);
            }
        }
    }
    set<int> estacoesMetroSet;
    // O(V)
    for (int estacao : estacoesMetro)
    {
        if (estacao != -1) { estacoesMetroSet.insert(estacao); }
    }
    // O(V + E)
    Planta* mstMetro = refazMst(mstSegs, nVertices, estacoesMetroSet);
    // O(V^2 * log(V))
    vector<pair<pair<int, int>, int>> arestasMetro = achaArestasMetro(mstMetro, estacoesMetroSet);
    // O(E)
    for (pair<pair<int, int>, int> arestaMetro : arestasMetro)
    {
        pair<int, int> aresta = arestaMetro.first;
        int distancia = arestaMetro.second;

        SegmentoBusca* segmentoMetro = newSegmentoBusca(
            aresta.first,
            aresta.second,
            distancia,
            distancia / VelocidadeMetro,
            "metro"
        );
        plantaBusca->adicionaSegmento(segmentoMetro);
    }
    // O(V * (V + E))
    vector<pair<pair<int, int>, pair<int, double>>> arestasOnibus = achaArestasOnibus(planta, cicloBus);
    // O(E)
    for (pair<pair<int, int>, pair<int, double>> arestaOnibus : arestasOnibus)
    {
        pair<int, int> aresta = arestaOnibus.first;
        pair<int, double> distTempo = arestaOnibus.second;

        SegmentoBusca* segmentoOnibus = newSegmentoBusca(
            aresta.first,
            aresta.second,
            distTempo.first,
            distTempo.second,
            "onibus"
        );
        plantaBusca->adicionaSegmento(segmentoOnibus);
    }

    return plantaBusca;
}
// O(1)
/// @brief Calcula o custo de um segmento de táxi.
/// @param origem Inteiro representando o vértice de origem.
/// @param destino Inteiro representando o vértice de destino.
/// @param dist_taxi Double representando a distância percorrida até o momento.
/// @param adjacente Ponteiro para o SegmentoBusca adjacente.
/// @return Par de doubles representando o custo e a nova distância.
pair<double, double> calcula_custo_taxi(int origem, int destino, double dist_taxi, SegmentoBusca* adjacente) {
    double segmento_tamanho = adjacente->distancia;
    double nova_distancia = dist_taxi + segmento_tamanho;
    double custo = 0.0;

    // Calcula custo variável caso exceda o limite de km gratuitos
    if (nova_distancia > limite_metro) {
        custo = taxa_variavel * (nova_distancia - limite_metro);
    }

    return {custo, nova_distancia};
}
// O(1)
/// @brief Calcula o custo de um segmento de transporte.
/// @param atual Ponteiro para o SegmentoBusca atual.
/// @param adjacente Ponteiro para o SegmentoBusca adjacente.
/// @param distancia_taxi Double representando a distância percorrida até o momento.
/// @return Par de doubles representando o custo e a nova distância.
pair<double, double> calcula_custo(SegmentoBusca* atual, SegmentoBusca* adjacente, double distancia_taxi) {
    if (atual->meioTransporte != adjacente->meioTransporte) {
        if (adjacente->meioTransporte == "metro") {
            return {passagem_metro, distancia_taxi}; // Exemplo: custo fixo para mudar para o metrô
        }
        if (adjacente->meioTransporte == "onibus") {
            return {passagem_onibus, distancia_taxi}; // Exemplo: custo fixo para mudar para ônibus
        }
    }
    if (adjacente->meioTransporte == "taxi" && adjacente->vertical == true) {
        return {taxa_fixa, distancia_taxi}; // Exemplo: custo fixo para mudar para táxi
    }

    if (adjacente->meioTransporte == "taxi" && adjacente->vertical == false) {
        return calcula_custo_taxi(atual->vDestino, adjacente->vDestino, distancia_taxi, adjacente);
    }
    return {0.0, distancia_taxi}; // Não muda de meio de transporte
}


