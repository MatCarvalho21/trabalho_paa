/**
 * @file algoritmos.h
 * @brief Módulo para definição das função do algoritmos dos problemas.
 */

#include "estrutura.h"
#include <vector>
#include <utility> 

#ifndef ALGORITMOS_H
#define ALGORITMOS_H

using namespace std;

// Declaração da função com a ordem invertida no par
pair<vector<int>, vector<Segmento*>> subway(Planta*, int);
vector<int> bus(Planta*);
vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca*, int, int, double);
vector<SegmentoBusca*> melhorRota(Planta*, vector<int>, vector<Segmento*>, vector<int>, int, int, double);

#include "algoritmos.cpp"

#endif // ALGORITMOS_H
