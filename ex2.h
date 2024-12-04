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
vector<int> nearestNeighbor(Planta*, int);
pair<int, int> calcularCustoDirecionado(const vector<vector<int>>, const vector<int>&);
pair<vector<int>, int> twoOptDirected(Planta*, const vector<int>&);
vector<vector<int>> gerarMatrizAdjacencia(Planta*);
int encontrarVerticeOtimo(Planta*, const set<int>&, int);
set<int> achaVerticesRegionais(Planta*, const set<int>&);

#endif // EX2_H