/**
 * @file utils.h
 * @brief Módulo para definição da função de utilidade.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <climits>
#include <random>

#ifndef UTILS_H
#define UTILS_H

#include "estrutura.h"

using namespace std;

void printSegmento(const Segmento*);
void printSet(const set<Segmento*>&);
void adicionaTransito(Planta*);

#include "utils.cpp"

#endif