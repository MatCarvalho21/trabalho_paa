#include <iostream>
#include <string>
#include <vector>
#include <cstdlib>

using std::cout;
using std::string;
using std::endl;

using namespace std;

// Estrutura do imóvel
typedef struct Imovel{
    // Distância até o final do segmento a que pertence
    int dFinalSeg;
    int CEP;
    string rua;
    int num;
    string tipo;    
} Imovel;

// Estrutura do segmento (aresta do grafo)
typedef struct Segmento{
    // Vértices de onde ele sai e entra
    int vSaida;
    int vEntrada;
    // Vetor dos imóveis
    vector<Imovel> imoveis;
    int limVel;
    int tamanho;
    string rua;
} Segmento;

typedef struct Planta{
    // A lista de adjacência é um vetor de vetores de segmentos
    // Cada entrada i do vetor externo corresponde às arestas de saída do vértice i
    vector<vector<Segmento>> listaAdj;
} Planta;


Planta* newPlanta()
{
    Planta* temp = (Planta*) malloc(sizeof(Planta));
    vector<vector<Segmento>> tempVector;
    temp -> listaAdj = tempVector;

    return temp;
}

Segmento* newSegmento(int vSaida, int vEntrada, vector<Imovel> imoveis, int limVel, int tamanho, string rua)
{
    Segmento* temp = (Segmento*) malloc(sizeof(Segmento));
    
    temp -> vSaida = vSaida;
    temp -> vEntrada = vEntrada;
    temp -> imoveis = imoveis;
    temp -> limVel = limVel;
    temp -> tamanho = tamanho;
    temp -> rua = rua;

    return temp;
}

Imovel* newImovel(int dFinalSeg, int CEP, string rua, int num, string tipo)
{
    Imovel* temp = (Imovel*) malloc(sizeof(Imovel));
    
    temp -> dFinalSeg = dFinalSeg;
    temp -> CEP = CEP;
    temp -> rua = rua;
    temp -> num = num;
    temp -> tipo = tipo;

    return temp;
}


int main(){
    return 0;
}


