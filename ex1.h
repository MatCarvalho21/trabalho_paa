/**
 * @file ex1.h
 * @brief Módulo para definição da função do algoritmo do problema 1.
 */

#ifndef EX1_H
#define EX1_H

#include "estrutura.h"
#include <vector>
#include <utility> 

using namespace std;

// Declaração da função com a ordem invertida no par
pair<vector<int>, vector<Segmento*>> subway(Planta* planta, int numVertices);

#endif // EX1_H
