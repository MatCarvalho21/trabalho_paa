#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"

int main()
{
    Planta* planta = newPlanta(130);

    carregaJSON("mapa.json", planta);

    vector<int> parents;
    parents.resize(130);
    vector<int> distances;
    distances.resize(130);

    dijkstra(0, parents, distances, 130, planta);

    for (int i = 0; i < parents.size(); i++)
    {
        cout << parents[i] << endl;
    }

    return 0;
}