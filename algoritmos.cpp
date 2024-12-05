#include "estrutura.h"
#include "algoritmos.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <climits>
#include "algoritmosBase.h"
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