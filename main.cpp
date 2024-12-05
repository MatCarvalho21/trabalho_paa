#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"
#include "algoritmos.h"
#include "utils.h"
#include "mapaRandom.h"

#include <chrono>
#include <fstream>
#include <string>

using namespace std::chrono;
using namespace std;

bool verbose = false;

int main()
{
    // Carregando a planta manual
    Planta* planta = newPlanta(130);
    carregaJSON("data/mapa.json", planta);

    // Rodando o algoritmo da questão 1
    pair<vector<int>, vector<Segmento*>> result = subway(planta, 130);

    vector<int> stations = result.first;
    vector<Segmento*> edges = result.second;

    if (verbose)
    {
        cout << "Subway stations: [";

        for (int i = 0; i < stations.size() - 1; i++)
        {
            cout << stations[i] << ", ";
        }

        cout << stations[stations.size() - 1] << "]" << endl;

        cout << "Subway lines: [";

        for (int i = 0; i < edges.size(); i++)
        {
            cout << "(" << edges[i]->vSaida << ", " << edges[i]->vEntrada << "), ";
        }

        cout << "]" << endl;
    }

    // Rodando o algoritmo da questão 2
    vector<int> busStations = bus(planta);

    if (verbose)
    {
        cout << "Bus stations Cycle: [";

        for (int i = 0; i < busStations.size() - 1; i++)
        {
            cout << busStations[i] << ", ";
        }

        cout << busStations[busStations.size() - 1] << "]" << endl;
    }
    
    int iOrigem = 0;
    int iDestino = 129;
    float fVerba = 0;   

    cout << "Acesse o grafo que representa o bairro de L'Esquerra de l'Eixample em Barcelona-ESP em (./barcelona_mapa.png)." << endl;
    cout << "Cada cruzamento e segmento é representado por um número inteiro. " << endl;
    cout << "Escolha o vértice de origem, o vértice de destino e a verba disponível para a viagem." << endl;
    cout << "O algoritmo irá retornar a melhor rota (em relação ao tempo) entre os dois vértices, considerando a verba disponível." << endl << endl;

    cout << "Digite o vértice de origem: ";
    cin >> iOrigem;
    cout << "Digite o vértice de destino: ";
    cin >> iDestino;
    cout << "Digite a verba: ";
    cin >> fVerba;


    // Rodando o algoritmo da questão 3
    vector<SegmentoBusca*> bestRoute = melhorRota(planta, stations, edges, busStations, iOrigem, iDestino, fVerba);


    cout << "Best route: (Entrada, Saída, Meio de Transporte)" << endl;
    for (int i = 0; i < bestRoute.size(); i++)
    { 
        cout << "(" << bestRoute[i]->vOrigem << ", " << bestRoute[i]->vDestino << ", " << bestRoute[i]->meioTransporte <<  ") " << endl;
    }
    cout << endl;

    return 0;
}