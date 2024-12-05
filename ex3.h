/**
 * @file ex3.h
 * @brief Módulo para definição da função do algoritmo do problema 3.
 */

#ifndef EX3_H
#define EX3_H

#include "estrutura.h"
#include <vector>
#include <set>
#include <utility>

using namespace std;

struct Estado {
    SegmentoBusca* segmento;
    float custo_acumulado;
    float distancia_taxi;
    float tempo_acumulado;

    // Comparação pelo operador '<' para a fila de prioridade
    bool operator<(const Estado& outro) const {
        return custo_acumulado > outro.custo_acumulado; // Menor custo tem maior prioridade
    }

    // Comparação pelo operador '>'
    bool operator>(const Estado& outro) const {
        return custo_acumulado < outro.custo_acumulado; // Inverso para std::greater
    }
};

pair<vector<int>, vector<int>> dijkstraMetro(Planta*, int);
vector<pair<pair<int, int>, float>> achaArestasMetro(Planta*, vector<int>);
vector<pair<int, float>> calculaDistTempoCiclo(Planta*, vector<int>, int);
vector<pair<pair<int, int>, pair<int, float>>> achaArestasOnibus(Planta*, vector<int>);
PlantaBusca* constroiPlantaBusca(Planta*, vector<int>, Planta*, vector<int>);
pair<float, float> calcula_custo_taxi(int, int, float, SegmentoBusca*);
pair<float, float> calcula_custo(SegmentoBusca* atual, SegmentoBusca* adjacente, float distancia_taxi);
vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca&, int, int, float);


#endif // EX3_H