/**
 * @file estrutura.cpp
 * @brief Módulo para definição das funções de criação e manipulação das estruturas Planta, Segmento e Imovel.
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

#include "estrutura.h"

/// @brief Inicializa uma planta.
/// @param numVertices Número de vértices que a planta terá.
/// @return A planta inicializada.
Planta* newPlanta(int numVertices)
{
    Planta* temp = new Planta();
    // Configura o tamanho de sua lista de adjacência
    temp->listaAdj.resize(numVertices);
    return temp;
}

/// @brief Inicializa um segmento (aresta).
/// @param vSaida Vértice do qual a aresta direcionada sai. (Origem)
/// @param vEntrada Vértice no qual a aresta direcionada entra. (Destino)
/// @param limVel Limite de velocidade do segmento.
/// @param tamanho Tamanho do segmento (metros).
/// @param CEP Região à qual o segmento pertence.
/// @param rua Rua à qual o segmento pertence
/// @return O segmento inicializado
Segmento* newSegmento(int vSaida, int vEntrada, int limVel, int tamanho, int CEP, string rua, bool dupla)
{
    Segmento* temp = new Segmento();
    
    temp->vSaida = vSaida;
    temp->vEntrada = vEntrada;

    // Inicializa sua lista de imóveis vazia
    vector<Imovel*> tempImoveis;
    temp->imoveis = tempImoveis;

    temp->limVel = limVel;
    temp->tamanho = tamanho;
    temp->CEP = CEP;
    temp->rua = rua;
    temp->transito = 1;

    return temp;
}

/// @brief Inicializa um imóvel.
/// @param dFinalSeg Distância do final do segmento, ou seja, ao vértice de entrada do segmento.
/// @param num Número do imóvel.
/// @param tipo Tipo do imóvel ("residencial", "comercial", "industrial" ou "turismo")
/// @return Um imóvel inicializado.
Imovel* newImovel(int dFinalSeg, int num, string tipo)
{
    Imovel* temp = new Imovel();
    
    temp->dFinalSeg = dFinalSeg;
    temp->num = num;
    temp->tipo = tipo;

    return temp;
}

/// @brief Adiciona um imóvel a um segmento
/// @param imovel Objeto do tipo Imovel a ser inserido.
/// @param segmento Objeto do tipo Segmento que receberá o imóvel.
void adicionaImovelASegmento(Imovel* imovel, Segmento* segmento)
{
    // Adiciona o imóvel à lista de imóveis do segmento
    segmento -> imoveis.push_back(imovel);
    // Configura o CEP e a rua do imóvel como os mesmos do segmento
    imovel -> CEP = segmento -> CEP;
    imovel -> rua = segmento -> rua;
}

/// @brief Adiciona um segmento a uma planta.
/// @param segmento Objeto do tipo segmento a ser adicionado.
/// @param planta Objeto do tipo planta que receberá o segmento.
void adicionaSegmentoAPlanta(Segmento* segmento, Planta* planta)
{
    // Adiciona o segmento à lista de adjacência da planta na posição correspondente ao seu vértice de saída
    ((planta -> listaAdj)[segmento -> vSaida]).push_back(segmento);
    (planta -> CEPs).insert(segmento -> CEP);
}

SegmentoBusca* newSegmentoBusca(int vOrigem, int vDestino, float distancia, float tempo, string meioTransporte)

{
    SegmentoBusca* temp = new SegmentoBusca(vOrigem, vDestino, distancia, tempo, meioTransporte);
    
    temp->vOrigem = vOrigem;
    temp->vDestino = vDestino;
    temp->distancia = distancia;
    temp->tempo = tempo;
    temp->meioTransporte = meioTransporte;
    temp->vertical = false;

    return temp;
}

PlantaBusca* newPlantaBusca(int numVertices)
{
    PlantaBusca* temp = new PlantaBusca(numVertices);
    return temp;
}
