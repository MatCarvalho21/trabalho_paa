/**
 * @file ex2.h
 * @brief Módulo para definição da função do algoritmo do problema 2.
 */

#ifndef EX2_H
#define EX2_H

#include "estrutura.h"
#include <vector>
#include <set>
#include <utility>

using namespace std;

int calcula_peso(const Segmento& segmento);
pair<Planta*, set<int>> construir_grafo_virtual(Planta*, int);
pair<vector<int>, vector<int>> dijkstra_regional(Planta*, int, int);
vector<int> encontrarVerticesOtimos(Planta*, const set<int>&, int);
vector<int> nearestNeighbor(Planta*, int);
int calcularCustoDirecionado(Planta*, const vector<int>&);
pair<vector<int>, int> twoOptDirected(Planta*, const vector<int>&);

#endif // EX2_H