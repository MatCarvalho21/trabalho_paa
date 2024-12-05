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

int main()
{
    // Carregando a planta manual
    Planta* planta = newPlanta(130);
    carregaJSON("data/mapa.json", planta);

    // Rodando o algoritmo da questão 1
    pair<vector<int>, vector<Segmento*>> result = subway(planta, 130);

    vector<int> stations = result.first;
    vector<Segmento*> edges = result.second;

    cout << "Subway stations: [";

    for (int i = 0; i < stations.size(); i++)
    {
        cout << stations[i] << ", ";
    }

    cout << "]" << endl;

    cout << "Subway lines: [";

    for (int i = 0; i < edges.size(); i++)
    {
        cout << "(" << edges[i]->vSaida << ", " << edges[i]->vEntrada << "), ";
    }

    cout << "]" << endl;

    // TODO: ALGORITMOS 2 E 3 NO MAPA MANUAL

    vector<int> busStations = bus(planta);

    cout << "Bus stations Cycle: [";

    for (int i = 0; i < busStations.size() - 1; i++)
    {
        cout << busStations[i] << ", ";
    }

    cout << busStations[busStations.size() - 1] << "]" << endl;

    cout << "]" << endl;

    //////////////////////////////////////////////////////////////////////////////////

    return 0;
}