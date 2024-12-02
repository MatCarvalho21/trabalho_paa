#include "estrutura.h"
#include "ex2.h"
#include <vector>
#include <limits>
#include <algorithm>
#include <set>
#include <numeric>
#include <iostream>
#include <queue>
#include <utility>

using namespace std;

int main(){
    cout << "\nTESTE: calcula_peso() \n" <<endl;
    // Criação da planta com 6 vértices
    Planta* plantaTeste = newPlanta(6);  // Cria a planta com 6 vértices

    // Criação dos segmentos entre os vértices
    Segmento* segmento1 = newSegmento(0, 1, 60, 250, 2004, "Rua A", true);
    Segmento* segmento2 = newSegmento(1, 2, 60, 250, 2004, "Rua B", true);
    Segmento* segmento3 = newSegmento(2, 3, 60, 250, 2004, "Rua C", true);
    Segmento* segmento4 = newSegmento(3, 4, 60, 250, 2004, "Rua D", true);
    Segmento* segmento5 = newSegmento(4, 5, 60, 250, 2004, "Rua E", true);
    Segmento* segmento6 = newSegmento(5, 0, 60, 250, 2004, "Rua F", true);

    // Criando mais segmentos para obter 18 arestas no total
    Segmento* segmento7 = newSegmento(0, 2, 60, 250, 2004, "Rua G", true);
    Segmento* segmento8 = newSegmento(1, 3, 60, 250, 2004, "Rua H", true);
    Segmento* segmento9 = newSegmento(2, 4, 60, 250, 2004, "Rua I", true);
    Segmento* segmento10 = newSegmento(3, 5, 60, 250, 2004, "Rua J", true);
    Segmento* segmento11 = newSegmento(4, 0, 60, 250, 2004, "Rua K", true);
    Segmento* segmento12 = newSegmento(5, 1, 60, 250, 2004, "Rua L", true);

    // Segmentos adicionais para completar o número de arestas
    Segmento* segmento13 = newSegmento(0, 3, 60, 250, 2004, "Rua M", true);
    Segmento* segmento14 = newSegmento(1, 4, 60, 250, 2004, "Rua N", true);
    Segmento* segmento15 = newSegmento(2, 5, 60, 250, 2004, "Rua O", true);
    Segmento* segmento16 = newSegmento(3, 0, 60, 250, 2004, "Rua P", true);
    Segmento* segmento17 = newSegmento(4, 1, 60, 250, 2004, "Rua Q", true);
    Segmento* segmento18 = newSegmento(5, 2, 60, 250, 2004, "Rua R", true);

    // Adicionando os segmentos à planta
    adicionaSegmentoAPlanta(segmento1, plantaTeste);
    adicionaSegmentoAPlanta(segmento2, plantaTeste);
    adicionaSegmentoAPlanta(segmento3, plantaTeste);
    adicionaSegmentoAPlanta(segmento4, plantaTeste);
    adicionaSegmentoAPlanta(segmento5, plantaTeste);
    adicionaSegmentoAPlanta(segmento6, plantaTeste);
    adicionaSegmentoAPlanta(segmento7, plantaTeste);
    adicionaSegmentoAPlanta(segmento8, plantaTeste);
    adicionaSegmentoAPlanta(segmento9, plantaTeste);
    adicionaSegmentoAPlanta(segmento10, plantaTeste);
    adicionaSegmentoAPlanta(segmento11, plantaTeste);
    adicionaSegmentoAPlanta(segmento12, plantaTeste);
    adicionaSegmentoAPlanta(segmento13, plantaTeste);
    adicionaSegmentoAPlanta(segmento14, plantaTeste);
    adicionaSegmentoAPlanta(segmento15, plantaTeste);
    adicionaSegmentoAPlanta(segmento16, plantaTeste);
    adicionaSegmentoAPlanta(segmento17, plantaTeste);
    adicionaSegmentoAPlanta(segmento18, plantaTeste);

    // Criando os imóveis
    Imovel* im1 = newImovel(250, 5, "residencial");
    Imovel* im2 = newImovel(200, 10, "comercial");
    Imovel* im3 = newImovel(150, 15, "industrial");
    Imovel* im4 = newImovel(100, 20, "residencial");
    Imovel* im5 = newImovel(50, 25, "industrial");
    Imovel* im6 = newImovel(0, 30, "turismo");

    // Adicionando imóveis diferentes para cada segmento
    adicionaImovelASegmento(newImovel(300, 5, "comercial"), segmento1);
    adicionaImovelASegmento(newImovel(350, 10, "residencial"), segmento1);
    adicionaImovelASegmento(newImovel(400, 15, "industrial"), segmento1);
    adicionaImovelASegmento(newImovel(450, 20, "comercial"), segmento1);
    adicionaImovelASegmento(newImovel(500, 25, "industrial"), segmento1);

    adicionaImovelASegmento(newImovel(250, 5, "residencial"), segmento2);
    adicionaImovelASegmento(newImovel(200, 10, "comercial"), segmento2);
    adicionaImovelASegmento(newImovel(150, 15, "industrial"), segmento2);
    adicionaImovelASegmento(newImovel(100, 20, "residencial"), segmento2);
    adicionaImovelASegmento(newImovel(50, 25, "industrial"), segmento2);

    adicionaImovelASegmento(newImovel(500, 5, "turismo"), segmento3);
    adicionaImovelASegmento(newImovel(400, 10, "residencial"), segmento3);
    adicionaImovelASegmento(newImovel(350, 15, "industrial"), segmento3);
    adicionaImovelASegmento(newImovel(300, 20, "comercial"), segmento3);
    adicionaImovelASegmento(newImovel(450, 25, "industrial"), segmento3);

    adicionaImovelASegmento(newImovel(350, 5, "comercial"), segmento4);
    adicionaImovelASegmento(newImovel(300, 10, "residencial"), segmento4);
    adicionaImovelASegmento(newImovel(400, 15, "industrial"), segmento4);
    adicionaImovelASegmento(newImovel(250, 20, "residencial"), segmento4);
    adicionaImovelASegmento(newImovel(100, 25, "turismo"), segmento4);

    adicionaImovelASegmento(newImovel(200, 5, "residencial"), segmento5);
    adicionaImovelASegmento(newImovel(500, 10, "industrial"), segmento5);
    adicionaImovelASegmento(newImovel(350, 15, "comercial"), segmento5);
    adicionaImovelASegmento(newImovel(250, 20, "industrial"), segmento5);
    adicionaImovelASegmento(newImovel(400, 25, "comercial"), segmento5);

    adicionaImovelASegmento(newImovel(100, 5, "industrial"), segmento6);
    adicionaImovelASegmento(newImovel(150, 10, "turismo"), segmento6);
    adicionaImovelASegmento(newImovel(250, 15, "turismo"), segmento6);
    adicionaImovelASegmento(newImovel(300, 20, "industrial"), segmento6);
    adicionaImovelASegmento(newImovel(450, 25, "comercial"), segmento6);

    cout << "Peso do segmento 1: " << calcula_peso(*segmento1) << endl;
    cout << "Peso do segmento 2: " << calcula_peso(*segmento2) << endl;
    cout << "Peso do segmento 3: " << calcula_peso(*segmento3) << endl;
    cout << "Peso do segmento 4: " << calcula_peso(*segmento4) << endl;
    cout << "Peso do segmento 5: " << calcula_peso(*segmento5) << endl;
    cout << "Peso do segmento 6: " << calcula_peso(*segmento6) << endl;
    
    cout << "\nTESTE: construir_grafo_virtual() \n" <<endl;

    cout << "\nTESTE: dijkstra_regional() \n" <<endl;

    cout << "\nTESTE: encontrarVerticesOtimos() \n" <<endl;

    return 0;
}

/// @brief Função para calcular o peso de um segmento com base nos tipos de imóveis.
/// @param segmento Estrutura Segmento contendo o vetor de imóveis.
/// @return Peso calculado como (turísticos + comerciais) - (residenciais + industriais).
int calcula_peso(const Segmento& segmento) {
    int comerciais = 0;
    int industriais = 0;
    int turisticos = 0;
    int residenciais = 0;

    for (Imovel* imovel : segmento.imoveis) {
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
    set<int> vertices_borda;

    // Número de vértices na planta
    int num_vertices = planta->listaAdj.size();

    // Cria a planta virtual (grafo virtual)
    Planta* planta_virtual = newPlanta(num_vertices);

    // Itera sobre os vértices da planta original
    for (int i = 0; i < num_vertices; i++) {
        set<int> aux_set; // Conjunto auxiliar para armazenar os CEPs únicos
        vector<Segmento*> lista_aux = planta->listaAdj[i]; // Lista de adjacência do vértice i

        // Itera sobre os segmentos (arestas) de saída do vértice i
        for (Segmento* temp_node : lista_aux) {
            aux_set.insert(temp_node->CEP); // Adiciona o CEP do segmento ao conjunto auxiliar

            // Calcula o novo peso ajustado
            int novo_peso = calcula_peso(*temp_node);

            // Cria um novo segmento com o peso ajustado
            Segmento* temp_segmento = newSegmento(
                temp_node->vSaida,
                temp_node->vEntrada,
                temp_node->limVel,
                limiar + novo_peso,
                temp_node->CEP,
                temp_node->rua,
                temp_node->dupla
            );

            // Adiciona o novo segmento ao grafo virtual
            planta_virtual->listaAdj[temp_node->vSaida].push_back(temp_segmento);
        }

        // Verifica se há mais de um CEP único no conjunto auxiliar
        if (aux_set.size() > 1) {
            vertices_borda.insert(i); // Marca o vértice como de borda
        }
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
        for (int i = 0; i < num_vertices; ++i)
        {
            if (!visitados[i] && distancias[i] < menor_distancia)
            {
                menor_distancia = distancias[i];
                vertice_atual = i;
            }
        }

        if (menor_distancia == numeric_limits<int>::max()) // Todos os vértices foram visitados
            break;

        // Explorar os vizinhos do vértice atual
        for (Segmento* segmento : planta->listaAdj[vertice_atual])
        {
            if (segmento->CEP != cep_regiao)
                continue;

            int vizinho = segmento->vEntrada; // Considerando o vEntrada como o vizinho
            int peso_aresta = segmento->limVel; // Usando a limitação de velocidade como o peso da aresta

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
vector<int> encontrarVerticesOtimos(Planta* planta, const set<int>& verticesBorda, int cepRegiao)
{
    int numVertices = planta->listaAdj.size();
    vector<int> verticesOtimos;
    float menorMediaDistancias = numeric_limits<float>::infinity();

    for (int vertice = 0; vertice < numVertices; ++vertice)
    {
        auto [distancias, _] = dijkstra_regional(planta, vertice, cepRegiao);

        // Obter as distâncias apenas dos vértices de borda
        vector<int> distanciasBorda;
        for (int borda : verticesBorda)
        {
            if (distancias[borda] != numeric_limits<int>::max())
            {
                distanciasBorda.push_back(distancias[borda]);
            }
        }

        // Ignorar se não houver distâncias válidas
        if (distanciasBorda.empty())
        {
            continue;
        }

        // Calcular a média das distâncias
        float mediaDistancias = std::accumulate(distanciasBorda.begin(), distanciasBorda.end(), 0.0f) / distanciasBorda.size();

        // Atualizar os vértices ótimos se encontrarmos uma menor média
        if (mediaDistancias < menorMediaDistancias)
        {
            menorMediaDistancias = mediaDistancias;
            verticesOtimos.clear();
            verticesOtimos.push_back(vertice);
        }
    }

    return verticesOtimos;
}

/// @brief Aplica a heurística do vizinho mais próximo para encontrar um ciclo no grafo.
/// @param plantaRegioes Ponteiro para a estrutura Planta representando o grafo das regiões.
/// @param verticeInicial Inteiro representando o vértice inicial do ciclo. Padrão é 0.
/// @return Vetor de inteiros contendo o ciclo encontrado, passando por todos os vértices e retornando ao inicial.
vector<int> nearestNeighbor(Planta* plantaRegioes, int verticeInicial = 0)
{
    int numVertices = plantaRegioes->listaAdj.size();
    vector<int> ciclo;
    vector<bool> visitados(numVertices, false);

    int verticeAtual = verticeInicial;
    visitados[verticeAtual] = true;

    while (true)
    {
        auto& listaAdjAtual = plantaRegioes->listaAdj[verticeAtual];
        float menorPeso = numeric_limits<float>::infinity();
        int proximoVertice = -1;

        for (Segmento* segmento : listaAdjAtual)
        {
            int vizinho = segmento->vEntrada; // Considerando o vértice de entrada como vizinho
            float peso = segmento->limVel;   // Usando limVel como peso da aresta

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

/// @brief Calcula o custo total de um ciclo em um grafo direcionado usando as estruturas Planta e Segmento.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @param ciclo Vetor de inteiros representando o ciclo.
/// @return Custo total do ciclo.
int calcularCustoDirecionado(Planta* planta, const vector<int>& ciclo)
{
    int custoTotal = 0;
    int n = ciclo.size();

    for (int i = 0; i < n; ++i)
    {
        int vAtual = ciclo[i];
        int vProximo = ciclo[(i + 1) % n]; // Considera o ciclo retornando ao início no último passo
        bool arestaEncontrada = false;

        // Verifica os segmentos que saem de vAtual
        for (Segmento* segmento : planta->listaAdj[vAtual])
        {
            if (segmento->vEntrada == vProximo)
            {
                custoTotal += segmento->limVel; // Usando limVel como custo
                arestaEncontrada = true;
                break;
            }
        }

        // Se não encontrar uma aresta válida, o ciclo é inválido
        if (!arestaEncontrada)
        {
            return numeric_limits<int>::max();
        }
    }

    return custoTotal;
}

/// @brief Otimiza um ciclo direcionado em um grafo usando a técnica Two-Opt com as estruturas Planta e Segmento.
/// @param planta Ponteiro para a estrutura Planta representando o grafo.
/// @param cicloInicial Vetor representando o ciclo inicial.
/// @return Um par contendo o ciclo otimizado e o custo total do ciclo.
pair<vector<int>, int> twoOptDirected(Planta* planta, const vector<int>& cicloInicial)
{
    int n = cicloInicial.size();
    vector<int> melhorCiclo = cicloInicial;
    int melhorCusto = calcularCustoDirecionado(planta, melhorCiclo);
    bool melhorado = true;

    while (melhorado)
    {
        melhorado = false;
        for (int i = 0; i < n - 1; ++i)
        {
            for (int j = i + 2; j < n; ++j) // j deve ser pelo menos dois índices à frente de i
            {
                // Gera um novo ciclo invertendo os nós entre i+1 e j
                vector<int> novoCiclo = melhorCiclo;
                reverse(novoCiclo.begin() + i + 1, novoCiclo.begin() + j + 1);

                // Calcula o custo do novo ciclo
                int novoCusto = calcularCustoDirecionado(planta, novoCiclo);

                // Se o novo ciclo for melhor, atualiza as variáveis
                if (novoCusto < melhorCusto)
                {
                    melhorCiclo = novoCiclo;
                    melhorCusto = novoCusto;
                    melhorado = true;
                }
            }
        }
    }

    return {melhorCiclo, melhorCusto};
}