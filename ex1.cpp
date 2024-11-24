#include "estrutura.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <climits>
#include "algoritmosBase.h"

using namespace std;

/// @brief Função para gerar o vértice
/// @param planta Planta original que contém os segmentos.
/// @param numVertices Número de vértices que a nova planta terá.
/// @return Vetor
set<Segmento*> subway(Planta* planta, int numVertices)
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
                // Não iremos criar os imóveis pois não será usado
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
            }
        }
        }
    }

    // Pegamos um ponto de partida par ao algorítimo
    int v0 = minMaxDistancesVertices[0];
    vector<int> parents(numVertices, -1);

    // Fazemos a mst
    primMST(v0, parents, numVertices, plantaND);

    // Resultado é um set 
    set<Segmento*> result;

    // Para cada região
    for (int i = 1; i < numReg; i++)
    {
        // Pegamos o vertice do metro   
        int v1 = minMaxDistancesVertices[i];

        // o pai atual é o pai de v1, e iteramos até encontrar v0
        while (v1 != v0)
        {
            // Obtém o pai de v1 (isto é, o vértice anterior no caminho da árvore geradora mínima)
            int current_parent = parents[v1];
            // Obtém a lista de segmentos (arestas) do vértice current_parent
            vector<Segmento*> edges = (plantaND -> listaAdj)[current_parent];
            
            // Percorre todos os segmentos do vértice current_parent
            for (int e = 0; e < edges.size(); e++)
            {
                Segmento* edge = edges[e];
                // Verifica se o segmento é a aresta que conecta v1 ao seu pai
                if (edge -> vEntrada == v1)
                {
                    // Se for a aresta que conecta v1 ao seu pai, adicionamos ela
                    result.insert(edge);
                    break;
                }
            }
            // Setamos v1 como o pai
            v1 = current_parent;
        }
    }

    delete plantaND;

    return result;
}