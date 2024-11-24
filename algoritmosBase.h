/**
 * @file algoritmosBase.h
 * @brief Módulo para definição dos protótipos das funções de criação do mapa.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>

#include "estrutura.h"

using std::cout;
using std::string;
using std::endl;
using std::vector;
using std::set;

#ifndef BASE_H
#define BASE_H

void dijkstra(int, vector<int>&, vector<int>&, int, Planta*);
void primMST(int, vector<int>&, int, Planta*);


#endif