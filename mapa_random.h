/**
 * @file mapa_random.h
 * @brief Módulo para definição das estruturas Planta, Segmento e Imovel.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include "estrutura.h"

using namespace std;

#ifndef TESTE_H
#define TESTE_H

// Estrutura auxiliar para representar uma rua
struct RuaInfo {
    string nome;
    vector<Segmento*> segmentos;
    int numAtual;  // Próximo número disponível para imóveis
    bool maoUnica; // Se a rua é de mão única ou não
};

// Estrutura auxiliar para representar um CEP
struct CEPInfo {
    int numero;
    vector<int> vertices;  // Vértices que pertencem a este CEP
    int centroide;        // Vértice central desta região
};

template <typename T>
std::string to_string(const T&);
string geraNomeRua();
bool verificaCruzamento(const vector<pair<int,int>>&, pair<int,int>);
void geraImoveis(Segmento*, RuaInfo&);
int determinaCEP(int, const vector<CEPInfo>&, const vector<vector<int>>&);
vector<vector<int>> calculaDistancias(int, const vector<pair<int,int>>&);
Planta* geraPlantaAutomatica(int, int);

#endif