#include "estrutura.h"
#include "algoritmos.h"
#include "algoritmosBase.h"

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <climits>
#include <utility> // Necessário para usar std::pair

using namespace std;

/// @brief Função para gerar as estações de metrô em um grafo, bem como a menor rota de metrô entre todas as estações.
/// @param planta Planta original que contém os segmentos.
/// @param numVertices Número de vértices que a planta possui.
/// @return Um par contendo:
/// - Vetor de vértices que compõem a menor rota de metrô.
/// - Vetor com os segmentos pertencentes à rota de metrô.
pair<vector<int>, vector<Segmento*>> subway(Planta* planta, int numVertices)
{
    // Cria uma nova planta com o número de vértices especificado
    Planta* plantaND = newPlanta(numVertices);

    // Criamos um planta com ida e volta, pois para o metro não interessa a direção
    for(int v = 0; v < numVertices; v++){
        vector<Segmento*> edges = planta -> listaAdj[v];
        for(int e = 0; e< edges.size(); e++){
            Segmento* edge = edges[e]; 
            adicionaSegmentoAPlanta(edge, plantaND);
            if (edge -> dupla == false){
                Segmento* newEdge = newSegmento(edge->vEntrada,
                                                edge -> vSaida,
                                                edge -> limVel,
                                                edge -> tamanho,
                                                edge -> CEP,
                                                edge -> rua,
                                                true);
                // Não iremos criar os imóveis pois não será usado para o metro
                adicionaSegmentoAPlanta(newEdge, plantaND);
                }
            }
        }
    // Salvamos o números de regiões
    int numReg = (planta -> CEPs).size();
    //Começamos o vetor de vertores todos como false
    vector< vector<bool> > regioes(numReg, vector<bool>(numVertices, false));

    // Setamos como true caso aquele vértice esteja na região
    for(int i =0; i<numVertices; i++){
        vector<Segmento*> edges = plantaND -> listaAdj[i];
        for(int e = 0; e< edges.size(); e++){
            Segmento* edge = edges[e];
            regioes[edge->CEP][i] = true; 
        }
    }

    // Vetores para as distancias máximas de cada região
    vector<int> minMaxDistances(numReg, INT_MAX);
    vector<int> minMaxDistancesVertices(numReg, -1);
    // Vetores para guardar os parents e os tamanhos das distâncias máximas
    vector<vector<int> > minMaxDistancesParents(numReg, vector<int>(numVertices, -1));
    vector<vector<int> > minMaxDistancesLengths(numReg, vector<int>(numVertices, INT_MAX));

    // Agora iteramos sobre todas as arestas
    for(int v = 0; v < numVertices; v++){
        // Criamos um vetor de parents e um de distances
        vector<int> parents(numVertices, -1);
        vector<int> distances(numVertices, INT_MAX);
        
        // Fazemos o djikistra saindo de v
        dijkstra(v, parents, distances, numVertices, plantaND);

        // Pra cada região
        for(int r = 0; r < numReg; r++){
            // Pegamos o vetor de booleanos o qual diz se o vértice esta na região r
            vector<bool> regiao = regioes[r];
            // Se estiver, é elegível para a região
            if (regiao[v] == true){
                // Setmaos a distância máxima como 0 e o vertice de -1
                int maxDistance = 0;
                int maxDistanceVertice = -1;
                // para cada vertice
                for (int vj = 0; vj<numVertices; vj++)
                {
                    // Se ele for elegível para a região e for maior que a distância máxima 
                    if (regiao[vj] == true && distances[vj] > maxDistance)
                    {
                        // Comçamos novamente
                        maxDistance = distances[vj];
                        maxDistanceVertice = vj;
                    }
                }
                // Checamos agora que a max_distance é menor que a menor máxima distancia
                if (maxDistance < minMaxDistances[r]){
                    // Se for redefinimos
                    minMaxDistances[r] = maxDistance;
                    minMaxDistancesVertices[r] = v;
                    // Atualizamos o vetor de pais e vetor de distâncias
                    minMaxDistancesParents[r] = parents;
                    minMaxDistancesLengths[r] = distances;
            }
        }
        }
    }

    // Agora que temos os menores caminhos, nossa ideia será gerar uma MST com arestas virtuais para cobrir todas as rotas de metrô
    Planta* plantaVirtual = newPlanta(numReg);

    // Nesse for, criamos esses segmentos virtuais, os quais possuem apenas a distância entre as estações de metro
    for (int i = 0; i < numReg; i++)
    {
        for (int j = 0; j < numReg; j++)
        {
            if (i != j)
            {
                Segmento* seg = newSegmento(i, j, 0, minMaxDistancesLengths[i][minMaxDistancesVertices[j]], 0, "null", false);
                adicionaSegmentoAPlanta(seg, plantaVirtual);
            }
        }
    }

    // Agora queremos saber quais sao os elementos da árvore, e para isso temos um set
    vector<int> parents(numReg, -1);

    // Fazemos a mst na planta virtual
    primMST(0, parents, numReg, plantaVirtual);

    // criamos um vetor de result
    vector<Segmento*> result;

    // Agora vamos iterar sobre a mst virtual e achar as arestas reais do nosso grafo
    // Começamos do 1, porque começamos nossa mst antes com 0
    for (int i = 1; i < numReg; i++)
    {
        // Encontramos o pai virtual desse índice, 
        int virtualParent = parents[i];
        // Encontramos agora o pai real, (do índice)
        int realParent = minMaxDistancesVertices[virtualParent];
        // Pegamos o vetor de parents dos pais reais
        vector<int> currentRealParents = minMaxDistancesParents[virtualParent];
        // O começo é o ponto atual
        int realStart = minMaxDistancesVertices[i];

        // Enquanto não encontrarmos o pai
        while (realStart != realParent)
        {
            // Obtém o pai de realStart (isto é, o vértice anterior no caminho da árvore geradora mínima)
            int currentParent = currentRealParents[realStart];
            // Obtém a lista de segmentos (arestas) do vértice current_parent
            vector<Segmento*> edges = (plantaND -> listaAdj)[currentParent];
            
            // Percorre todos os segmentos do vértice current_parent
            for (int e = 0; e < edges.size(); e++)
            {
                Segmento* edge = edges[e];
                // Verifica se o segmento é a aresta que conecta v1 ao seu pai
                if (edge -> vEntrada == realStart)
                {
                    // Se for a aresta que conecta v1 ao seu pai, adicionamos ela
                    result.push_back(edge);
                    break;
                }
            }
            // "Andamos" para trás no caminho
            realStart = currentParent;
        }
    }

    // Deletamos para não ocupar espaço
    delete plantaND;
    delete plantaVirtual;

    // Retornamos o par
    return make_pair(minMaxDistancesVertices, result);
}

/// @brief Função que calcula o ciclo de ônibus.
/// @param planta Planta original que contém os segmentos.
/// @return Vetor com os vértices que compõem o ciclo de ônibus.
vector<int> bus(Planta* planta)
{
    pair<Planta*, set<int>> grafoVirutal = construir_grafo_virtual(planta, LIMIAR);

    set<int> verticesRegionais = achaVerticesRegionais(grafoVirutal.first, grafoVirutal.second);

    pair<Planta*, vector<vector<int>>> grafoRegioes = construirGrafoRegioes(grafoVirutal.first, verticesRegionais);

    delete grafoVirutal.first;

    int origem = *verticesRegionais.begin();
    vector<int> cicloInicial = nearestNeighbor(grafoRegioes.first, origem);

    pair<vector<int>, int> cicloOtimizado = twoOptDirected(grafoRegioes.first, cicloInicial);
    
    vector<int> ciclo;

    if (cicloOtimizado.first.size() < 3)
    {
        return cicloOtimizado.first;
    }
    
    for (int i = 0; i < cicloOtimizado.first.size() - 1; i++)
    {
        int verticeAtual = cicloOtimizado.first[i];
        int verticeProximo = cicloOtimizado.first[i + 1];
        vector<int> predecessores = grafoRegioes.second[verticeAtual];
        vector<int> path;

        while (verticeProximo != -1)
        {
            path.push_back(verticeProximo);
            verticeProximo = predecessores[verticeProximo];
        }

        for (int j = path.size() - 1; j >= 0; j--)
        {
            if (ciclo.empty() || path[j] != ciclo.back())
            {
                ciclo.push_back(path[j]);
            }
        }
    }
    delete grafoRegioes.first;

    return ciclo;
}

/// @brief Função que calcula o custo de um segmento de acordo com o meio de transporte.
/// @param atual Segmento de origem.
/// @param adjacente Segmento de destino.
/// @param distancia_taxi Distância acumulada de táxi.
/// @return Par com o custo e a nova distância de táxi.
vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca* grafo, int vertice_inicial, int vertice_destino, double lim_dinheiro) {
    // Mapas para armazenar o menor tempo e o "pai" de cada segmento
    unordered_map<SegmentoBusca*, double> tempo_minimo;
    unordered_map<SegmentoBusca*, double> custo_acumulado;
    unordered_map<SegmentoBusca*, SegmentoBusca*> segmento_pai;

    // Fila de prioridade (menor custo no topo)
    priority_queue<Estado, vector<Estado>, greater<>> fila;

    // Inicialização com todos os tempos e custos como infinito
    for (const auto& adjacencias : grafo->listaAdj) {
        for (SegmentoBusca* segmento : adjacencias) {
            tempo_minimo[segmento] = INF;
            custo_acumulado[segmento] = INF;
        }
    }

    // Processa os segmentos saindo do vértice inicial
    for (SegmentoBusca* segmento : grafo->listaAdj[vertice_inicial]) {
        tempo_minimo[segmento] = 0.0; // Tempo inicial é zero
        custo_acumulado[segmento] = 0.0; // Custo inicial é zero
        fila.push({segmento, 0.0, 0.0, 0}); // Inicializa com distância_taxi = 0
        segmento_pai[segmento] = nullptr; // Sem pai para o primeiro segmento
    }

    // Processamento do algoritmo de Dijkstra
    while (!fila.empty()) {
        Estado estado_atual = fila.top();
        fila.pop();

        SegmentoBusca* segmento_atual = estado_atual.segmento;

        // Se o custo acumulado atual for maior que o limite, ignorar
        if (estado_atual.custo_acumulado > lim_dinheiro) {
            continue;
        }

        // Se o segmento atual for o destino, interrompa
        if (segmento_atual->vDestino == vertice_destino) {
            break;
        }

        // Iterar sobre os segmentos adjacentes
        for (SegmentoBusca* adjacente : grafo->listaAdj[segmento_atual->vDestino]) {
            double custo_aux, nova_distancia_taxi;

            // Calcula o custo para o segmento adjacente
            tie(custo_aux, nova_distancia_taxi) = calcula_custo(segmento_atual, adjacente, estado_atual.distancia_taxi);

            double novo_custo = estado_atual.custo_acumulado + custo_aux;
            double novo_tempo = 0.0;

            // Se o meio de transporte mudar, adiciona tempo de espera
            if (segmento_atual->meioTransporte != "onibus" && adjacente->meioTransporte == "onibus") {

                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> espera(0, tempo_espera_onibus);
                double tempo_espera_aux = espera(gen);
                novo_tempo = estado_atual.tempo_acumulado + adjacente->tempo; + tempo_espera_aux;
            }
            if (segmento_atual->meioTransporte != "metro" && adjacente->meioTransporte == "metro") {

                std::random_device rd;
                std::mt19937 gen(rd());
                std::uniform_int_distribution<> espera(0, tempo_espera_metro);
                double tempo_espera_aux = espera(gen);
                novo_tempo = estado_atual.tempo_acumulado + adjacente->tempo; + tempo_espera_aux;
            }
            else
            {
                novo_tempo = estado_atual.tempo_acumulado + adjacente->tempo;
            }
            

            // Atualiza se encontrar um custo menor e dentro do limite de dinheiro
            if (novo_custo <= lim_dinheiro) {
                if (novo_tempo < tempo_minimo[adjacente]) {
                    tempo_minimo[adjacente] = novo_tempo;
                    custo_acumulado[adjacente] = novo_custo;
                    fila.push({adjacente, novo_custo, nova_distancia_taxi, novo_tempo});
                    segmento_pai[adjacente] = segmento_atual;
                }
            }
        }
    }

    // Reconstruir o caminho com base no "pai" de cada segmento
    vector<SegmentoBusca*> caminho;
    SegmentoBusca* segmento_atual = nullptr;

    // Encontrar o segmento de destino com menor tempo
    double menor_tempo = INF;
    for (const auto& [segmento, tempo] : tempo_minimo) {
        if (tempo < menor_tempo && segmento->vDestino == vertice_destino) {
            menor_tempo = tempo;
            segmento_atual = segmento;
        }
    }

    // Reconstrução do caminho
    while (segmento_atual != nullptr) {
        caminho.push_back(segmento_atual);
        segmento_atual = segmento_pai[segmento_atual];
    }

    // Inverte o caminho para começar do vértice inicial
    reverse(caminho.begin(), caminho.end());

    return caminho;
}

/// @brief Função que calcula a melhor rota entre dois vértices de uma planta.
/// @param planta Planta original que contém os segmentos.
/// @param cicloBus Vetor com os vértices que compõem o ciclo de ônibus.
/// @param mstSegs Vetor com os segmentos que compõem a MST.
/// @param estacoesMetro Vetor com os vértices que representam as estações de metrô.
/// @param origem Vértice de origem.
/// @param destino Vértice de destino.
/// @param dinheiro Limite de dinheiro disponível.
/// @return Vetor com os segmentos que compõem a melhor rota.
vector<SegmentoBusca*> melhorRota(Planta* planta, vector<int> cicloBus, 
                            vector<Segmento*> mstSegs, vector<int> estacoesMetro, 
                            int origem, int destino, double dinheiro) 
{
    PlantaBusca* plantaBusca = constroiPlantaBusca(planta, cicloBus, mstSegs, estacoesMetro);
    vector<SegmentoBusca*> caminho = dijkstra_custo(plantaBusca, origem, destino, dinheiro);
    delete plantaBusca;
    return caminho;
}

