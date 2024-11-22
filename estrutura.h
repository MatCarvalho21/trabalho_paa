#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::string;
using std::endl;
using std::vector;

#ifndef ESTRUTURA_H
#define ESTRUTURA_H

// Estrutura do imóvel
typedef struct Imovel
{
    // Distância até o final do segmento a que pertence
    int dFinalSeg;
    int CEP;
    string rua;
    int num;
    string tipo;    
} Imovel;

// Estrutura do segmento (aresta do grafo)
typedef struct Segmento
{
    // Vértices de onde ele sai e entra
    int vSaida;
    int vEntrada;
    // Vetor dos imóveis
    vector<Imovel*> imoveis;
    int limVel;
    int tamanho;
    int regiao;
    string rua;
} Segmento;

typedef struct Planta
{
    // A lista de adjacência é um vetor de vetores de segmentos
    // Cada entrada i do vetor externo corresponde às arestas de saída do vértice i
    vector<vector<Segmento*> > listaAdj;
} Planta;

Planta* newPlanta(int);
Segmento* newSegmento(int, int, int, int, int, string);
Imovel* newImovel(int, int, string, int, string);
void adicionaImovelASegmento(Imovel*, Segmento*);
void adicionaSegmentoAPlanta(Segmento*, Planta*);

#endif