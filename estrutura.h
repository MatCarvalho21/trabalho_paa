/**
 * @file estrutura.h
 * @brief Módulo para definição das estruturas Planta, Segmento e Imovel.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>

using std::cout;
using std::string;
using std::endl;
using std::vector;
using std::set;

#ifndef ESTRUTURA_H
#define ESTRUTURA_H

/// @brief Estrutura dos imóveis.
typedef struct Imovel
{
    // Distância até o final do segmento a que pertence
    int dFinalSeg;
    int CEP;
    string rua;
    int num;
    string tipo;    
} Imovel;

/// @brief Estrutura do segmento (aresta do grafo).
typedef struct Segmento
{
    // Vértices de onde ele sai e entra
    int vSaida;
    int vEntrada;
    // Vetor dos imóveis
    vector<Imovel*> imoveis;
    int limVel;
    int tamanho;
    int CEP;
    string rua;
    bool dupla;
} Segmento;

/// @brief Estrutura da planta.
typedef struct Planta
{
    // A lista de adjacência é um vetor de vetores de segmentos
    // Cada entrada i do vetor externo corresponde às arestas de saída do vértice i
    vector<vector<Segmento*> > listaAdj;
    set<int> CEPs;
} Planta;

// Protótipos das funções
Planta* newPlanta(int);
Segmento* newSegmento(int, int, int, int, int, string, bool);
Imovel* newImovel(int, int, string);
void adicionaImovelASegmento(Imovel*, Segmento*);
void adicionaSegmentoAPlanta(Segmento*, Planta*);

#endif