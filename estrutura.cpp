/**
 * @file estrutura.cpp
 * @brief Módulo para definição das funções de criação e manipulação das estruturas Planta, Segmento e Imovel.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <limits>

#include "estrutura.h"
// Constantes
double INF = numeric_limits<double>::max();

int LIMIAR = 10;

double normalizacao = 3.6;
double VelocidadeMetro = 70.0 / normalizacao;
double VelocidadeAndar = 5.0 / normalizacao;

// variáveis táxi
double limite_metro = 1000;
double taxa_variavel = 0.008;
double taxa_fixa = 10;

// passagens
double passagem_metro = 7.5;
double passagem_onibus = 4.0;

// tempo de espera (segundos)
double tempo_espera_onibus = 420;
double tempo_espera_metro = 300;

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

/// @brief Cria uma instância de um SegmentoBusca.
/// @param vOrigem Vértice de origem do segmento.
/// @param vDestino Vértice de destino do segmento.
/// @param distancia Distância do segmento.
/// @param tempo Tempo do segmento.
/// @param meioTransporte Meio de transporte do segmento.
/// @return O segmento de busca inicializado.
SegmentoBusca* newSegmentoBusca(int vOrigem, int vDestino, float distancia, double tempo, string meioTransporte)

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

/// @brief Inicializa uma planta de busca.
/// @param numVertices Número de vértices que a planta terá.
/// @return A planta de busca inicializada.
PlantaBusca* newPlantaBusca(int numVertices)
{
    PlantaBusca* temp = new PlantaBusca(numVertices);
    return temp;
}
