#include "estrutura.h"
#include <vector>
#include <set>
#include <iostream>
#include <climits>
#include <utility>
#include <queue>

using namespace std;

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

vector<pair<pair<int, int>, int>> achaArestasMetro(Planta* mstMetro, vector<int> estacoesMetro)
{
    vector<pair<pair<int, int>, int>> arestasMetro;

    for (int i = 0; i < estacoesMetro.size(); i++)
    {
        pair<vector<int>, vector<int>> resultado = dijkstraMetro(mstMetro, estacoesMetro[i]);
        vector<int> distancias = resultado.first;
        vector<int> predecessores = resultado.second;

        for (int j = 0; j < estacoesMetro.size(); j++)
        {
            if (i == j) { continue; }

            int peso = distancias[estacoesMetro[j]];
            arestasMetro.push_back({{estacoesMetro[i], estacoesMetro[j]}, peso});
        }
    }
    return arestasMetro;
}



vector<pair<pair<int, int>, int>> achaArestasOnibus(Planta* planta, vector<int> cicloBus)
{
    
}



PlantaBusca* constroiPlantaBusca(Planta* planta, vector<int> cicloBus, Planta* mstMetro, vector<int> estacoesMetro)
{
    int nVertices = planta->listaAdj.size();
    PlantaBusca* plantaBusca = newPlantaBusca(nVertices * 2);



}






int main()
{
    return 0;
}

