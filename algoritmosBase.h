/**
 * @file algoritmosBase.h
 * @brief Módulo para definição dos protótipos das funções de criação do mapa.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <limits>
#include <algorithm>
#include <numeric>
#include <unordered_map>
#include <utility>

#include "estrutura.h"


#ifndef BASE_H
#define BASE_H

using namespace std;

void dijkstra(int, vector<int>&, vector<int>&, int, Planta*);
void primMST(int, vector<int>&, int, Planta*);


int calcula_peso(const Segmento& segmento);
pair<Planta*, set<int>> construir_grafo_virtual(Planta*, int);
pair<vector<int>, vector<int>> dijkstra_regional(Planta*, int, int);
pair<Planta*, vector<vector<int>>> construirGrafoRegioes(Planta*, set<int>);
vector<int> nearestNeighbor(Planta*, int);
pair<int, int> calcularCustoDirecionado(const vector<vector<int>>, const vector<int>&);
pair<vector<int>, int> twoOptDirected(Planta*, const vector<int>&);
vector<vector<int>> gerarMatrizAdjacencia(Planta*);
int encontrarVerticeOtimo(Planta*, const set<int>&, int);
set<int> achaVerticesRegionais(Planta*, const set<int>&);

pair<vector<int>, vector<int>> dijkstraMetro(Planta*, int);
vector<pair<pair<int, int>, int>> achaArestasMetro(Planta*, set<int>);
vector<pair<int, double>> calculaDistTempoCiclo(Planta*, vector<int>, int);
vector<pair<pair<int, int>, pair<int, double>>> achaArestasOnibus(Planta*, vector<int>);
PlantaBusca* constroiPlantaBusca(Planta*, vector<int>, vector<Segmento*>, vector<int>);
pair<double, double> calcula_custo_taxi(int, int, double, SegmentoBusca*);
pair<double, double> calcula_custo(SegmentoBusca* atual, SegmentoBusca* adjacente, double distancia_taxi);
vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca*, int, int, double);
vector<SegmentoBusca*> melhorRota(Planta*, vector<int>, vector<Segmento*>, vector<int>, int, int, double);

#include "algoritmosBase.cpp"

#endif