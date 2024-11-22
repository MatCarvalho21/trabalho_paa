#include <iostream>
#include <string>
#include <vector>

using namespace std;

// Estrutura do imóvel
typedef struct Imovel{
    // Distância até o final do segmento a que pertence
    int dFinalSeg;
    int CEP;
    string rua;
    int num;
    string tipo;    
} imovel;

// Estrutura do segmento (aresta do grafo)
typedef struct segmento{
    // Vértices de onde ele sai e entra
    int vSaida;
    int vEntrada;
    // Vetor dos imóveis
    vector<imovel> imoveis;
    int limVel;
    int tamanho;
    string rua;
} segmento;

typedef struct planta{
    // A lista de adjacência é um vetor de vetores de segmentos
    // Cada entrada i do vetor externo corresponde às arestas de saída do vértice i
    vector<vector<segmento>> listaAdj;
} planta;

int main(){
    return 0;
}


