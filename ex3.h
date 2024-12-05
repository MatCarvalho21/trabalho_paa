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
    double custo_acumulado;
    double distancia_taxi;
    double tempo_acumulado;

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
vector<pair<pair<int, int>, int>> achaArestasMetro(Planta*, set<int>);
vector<pair<int, double>> calculaDistTempoCiclo(Planta*, vector<int>, int);
vector<pair<pair<int, int>, pair<int, double>>> achaArestasOnibus(Planta*, vector<int>);
PlantaBusca* constroiPlantaBusca(Planta*, vector<int>, Planta*, vector<int>);
pair<double, double> calcula_custo_taxi(int, int, double, SegmentoBusca*);
pair<double, double> calcula_custo(SegmentoBusca* atual, SegmentoBusca* adjacente, double distancia_taxi);
vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca*, int, int, double);
vector<SegmentoBusca*> melhorRota(Planta*, vector<int>, vector<Segmento*>, vector<int>, int, int, double);

#include "ex3.cpp"

#endif // EX3_H