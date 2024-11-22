#include <iostream>
#include <string>
#include <vector>

using std::cout;
using std::string;
using std::endl;
using std::vector;

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
    string rua;
} Segmento;

typedef struct Planta
{
    // A lista de adjacência é um vetor de vetores de segmentos
    // Cada entrada i do vetor externo corresponde às arestas de saída do vértice i
    vector<vector<Segmento*> > listaAdj;
} Planta;


Planta* newPlanta(int numVertices)
{
    Planta* temp = new Planta();
    temp->listaAdj.resize(numVertices);
    return temp;
}

Segmento* newSegmento(int vSaida, int vEntrada, int limVel, int tamanho, string rua)
{
    Segmento* temp = new Segmento();
    
    temp->vSaida = vSaida;
    temp->vEntrada = vEntrada;

    vector<Imovel*> tempImoveis;
    temp->imoveis = tempImoveis;

    temp->limVel = limVel;
    temp->tamanho = tamanho;
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


int main()
{
    Imovel* teste = newImovel(3, 20, "rua", 30, "com");
    Segmento* steste = newSegmento(1, 2, 50, 30, "rua");
    Planta* pteste = newPlanta(5);

    // cout << "Imovel: " << endl;
    // cout << "dFinalSeg: " << teste->dFinalSeg << endl;
    // cout << "CEP: " << teste->CEP << endl;
    // cout << "Rua: " << teste->rua << endl;
    // cout << "Número: " << teste->num << endl;
    // cout << "Tipo: " << teste->tipo << endl;

    adicionaImovelASegmento(teste, steste);

    cout << steste->vSaida << endl;

    adicionaSegmentoAPlanta(steste, pteste);

    // printListaAdj(pteste);

    cout << (((pteste -> listaAdj)[1])[0]) -> rua;

    delete teste; // Liberando a memória alocada

    return 0;
}
