/**
 * @file algoritmos.h
 * @brief Módulo para definição das função do algoritmos dos problemas.
 */

#ifndef ALGORITMOS_H
#define ALGORITMOS_H

#include "estrutura.h"
#include <vector>
#include <utility> 

using namespace std;

// Declaração da função com a ordem invertida no par
pair<vector<int>, vector<Segmento*>> subway(Planta* planta, int numVertices);

#include "algoritmos.cpp"

#endif // ALGORITMOS_H
