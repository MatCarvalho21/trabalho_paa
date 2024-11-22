#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::string;
using std::endl;
using std::vector;

#include "estrutura.h"

Planta* newPlanta(int numVertices)
{
    Planta* temp = new Planta();
    temp->listaAdj.resize(numVertices);
    return temp;
}

Segmento* newSegmento(int vSaida, int vEntrada, int limVel, int tamanho, int regiao, string rua)
{
    Segmento* temp = new Segmento();
    
    temp->vSaida = vSaida;
    temp->vEntrada = vEntrada;

    vector<Imovel*> tempImoveis;
    temp->imoveis = tempImoveis;

    temp->limVel = limVel;
    temp->tamanho = tamanho;
    temp->regiao = regiao;
    temp->rua = rua;

    return temp;
}

Imovel* newImovel(int dFinalSeg, int CEP, string rua, int num, string tipo)
{
    Imovel* temp = new Imovel();
    
    temp->dFinalSeg = dFinalSeg;
    temp->CEP = CEP;
    temp->rua = rua;
    temp->num = num;
    temp->tipo = tipo;

    return temp;
}

void adicionaImovelASegmento(Imovel* imovel, Segmento* segmento)
{
    segmento -> imoveis.push_back(imovel);
}

void adicionaSegmentoAPlanta(Segmento* segmento, Planta* planta)
{
    ((planta -> listaAdj)[segmento -> vSaida]).push_back(segmento);
}