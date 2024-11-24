#include "estrutura.h"
#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <climits>
#include "algoritmosBase.h"

using namespace std;

/// @brief 
/// @param planta Planta original que contém os segmentos.
/// @param numVertices Número de vértices que a nova planta terá.
/// @return Vetor
set<Segmento*> subway(Planta* planta, int numVertices)
// void subway(Planta* planta, int numVertices)
{
    // Cria uma nova planta com o número de vértices especificado
    Planta* plantaND = newPlanta(numVertices);

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
    int numReg = (planta -> CEPs).size();
    vector< vector<bool> > regioes(numReg, vector<bool>(numVertices, false));

    for(int i =0; i<numVertices; i++){
        vector<Segmento*> edges = plantaND -> listaAdj[i];
        for(int e = 0; e< edges.size(); e++){
            Segmento* edge = edges[e];
            regioes[edge->CEP][i] = true; 
        
        }
    }
    vector<int> minMaxDistances(numReg, INT_MAX);
    vector<int> minMaxDistancesVertices(numReg, -1);

    for(int v = 0; v < numVertices; v++){
        vector<int> parents(numVertices, -1);
        vector<int> distances(numVertices, INT_MAX);
        
        dijkstra(v, parents, distances, numVertices, plantaND);
        for(int r = 0; r < numReg; r++){
            vector<bool> regiao = regioes[r];
            if (regiao[v] == true){
                int maxDistance = 0;
                int maxDistanceVertice = -1;
                for (int vj = 0; vj<numVertices; vj++)
                {
                    if (regiao[vj] == true && distances[vj] > maxDistance)
                    {
                        maxDistance = distances[vj];
                        maxDistanceVertice = vj;
                    }
                }
                 if (maxDistance < minMaxDistances[r]){
                    minMaxDistances[r] = maxDistance;
                    minMaxDistancesVertices[r] = v;
            }
        }
        }
    }

    int v0 = minMaxDistancesVertices[0];

    vector<int> parents(numVertices, -1);

    primMST(v0, parents, numVertices, plantaND);

    set<Segmento*> result;

    for (int i = 1; i < numReg; i++)
    {
        int v1 = minMaxDistancesVertices[i];

        while (v1 != v0)
        {
            int current_parent = parents[v1];
            vector<Segmento*> edges = (plantaND -> listaAdj)[current_parent];
            
            for (int e = 0; e < edges.size(); e++)
            {
                Segmento* edge = edges[e];
                if (edge -> vEntrada == v1)
                {
                    result.insert(edge);
                    break;
                }
            }

            v1 = current_parent;
        }
    }

    delete plantaND;

    return result;
}