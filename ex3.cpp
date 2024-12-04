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

vector<pair<pair<int, int>, float>> achaArestasMetro(Planta* mstMetro, vector<int> estacoesMetro)
{
    vector<pair<pair<int, int>, float>> arestasMetro;

    for (int i = 0; i < estacoesMetro.size(); i++)
    {
        pair<vector<int>, vector<int>> resultado = dijkstraMetro(mstMetro, estacoesMetro[i]);
        vector<int> distancias = resultado.first;
        vector<int> predecessores = resultado.second;

        for (int j = 0; j < estacoesMetro.size(); j++)
        {
            if (i == j) { continue; }

            float peso = (distancias[estacoesMetro[j]])/1000.0;
            arestasMetro.push_back({{estacoesMetro[i], estacoesMetro[j]}, peso});
        }
    }
    return arestasMetro;
}

vector<pair<int, float>> calculaDistTempoCiclo(Planta* planta, vector<int> ciclo, int start)
{
    int n = ciclo.size();
    vector<pair<int, float>> distanciasTempos;
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

    while (startIndex != endIndex)
    {
        vector<Segmento*> segmentos = planta->listaAdj[ciclo[startIndex]];

        int nextIndex = (startIndex + 1) % n;
        distanciasTempos[nextIndex] = distanciasTempos[startIndex];

        for (Segmento* segmento : segmentos)
        {
            if (segmento->vEntrada == ciclo[nextIndex])
            {
                float distanciaKM = segmento->tamanho/1000.0;
                distanciasTempos[nextIndex].first += distanciaKM;
                distanciasTempos[nextIndex].second += (distanciaKM / segmento->limVel);
                break;
            }
        }
    }

    return distanciasTempos;
}

vector<pair<pair<int, int>, pair<int, float>>> achaArestasOnibus(Planta* planta, vector<int> cicloBus)
{
    vector<int> cicloTemp = cicloBus;
    cicloTemp.pop_back();

    vector<pair<pair<int, int>, pair<int, float>>> arestasOnibus;

    for (int i = 0; i < cicloTemp.size(); i++)
    {
        vector<pair<int, float>> distanciasTempos = calculaDistTempoCiclo(planta, cicloTemp, cicloTemp[i]);

        for (int j = 0; j < cicloTemp.size(); j++)
        {
            if (i == j) { continue; }
            arestasOnibus.push_back({{cicloTemp[i], cicloTemp[j]}, {distanciasTempos[j].first, distanciasTempos[j].second}});
        }
    }
    return arestasOnibus;
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

