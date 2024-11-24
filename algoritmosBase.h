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

#ifndef EX1_H
#define EX1_H

void dijkstra(int, vector<int>&, vector<int>&, int, Planta*);
void primMST(vector<int>, int);


#endif