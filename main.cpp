#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"
#include "algoritmos.h"
#include "utils.h"
#include "mapaRandom.h"
#include "ex2.h"
#include "ex3.h"

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

    // Calculando os tempos de execução por tamanho de entrada

    ofstream file("times.csv", ios::out);

    if (!file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo!" << std::endl;
        return 1;
    }

    file << "problem,V,E,ratio,time" << endl;

    vector<int> Es(3);
    pair<vector<int>, vector<Segmento*>> return1;
    vector<int> return2;
    vector<SegmentoBusca*> return3;

    for (int V = 100; V <= 1000; V = V + 200)
    {
        Es[0] = V;
        Es[1] = 2*V;
        Es[2] = 3*V - 6;

        for (int i = 0; i <= 2; i++)
        {
            int E = Es[i];
            planta = geraPlantaAutomatica(V, E);

            if (planta == nullptr)
            {
                cout << "Planta inadequada" << endl;
                return 1;
            }
            
            auto timeStart = high_resolution_clock::now();
            return1 = subway(planta, V);
            auto timeStop = high_resolution_clock::now();
            auto timeDuration = duration_cast<nanoseconds>(timeStop - timeStart).count();
            cout << 1 << "," << V << "," << E << "," << i+1 << "," << timeDuration << endl;
            file << 1 << "," << V << "," << E << "," << i+1 << "," << timeDuration << endl;

            timeStart = high_resolution_clock::now();
            return2 = bus(planta);
            timeStop = high_resolution_clock::now();
            timeDuration = duration_cast<nanoseconds>(timeStop - timeStart).count();
            cout << 2 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;
            file << 2 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> verticeAleatorio(0, V-1);
            int v1 = verticeAleatorio(gen);
            int v2 = verticeAleatorio(gen);
            while (v1 == v2) { v2 = verticeAleatorio(gen); }

            std::uniform_real_distribution<> dinheiro(0, 100);
            float dinheiroAleatorio = dinheiro(gen);

            adicionaTransito(planta);

            timeStart = high_resolution_clock::now();
            return3 = melhorRota(planta, return2, return1.second, return1.first, v1, v2, dinheiroAleatorio);
            timeStop = high_resolution_clock::now();
            timeDuration = duration_cast<nanoseconds>(timeStop - timeStart).count();
            cout << 3 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;
            file << 3 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;
        }
    }

    file.close();

    return 0;
}