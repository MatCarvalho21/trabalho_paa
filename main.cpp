#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"
#include "ex1.h"
#include "utils.h"
#include "mapa_random.h"
#include "ex2.h"
#include "estrutura.cpp"
#include "mapa.cpp"
#include "algoritmosBase.cpp"
#include "ex1.cpp"
#include "utils.cpp"
#include "mapa_random.cpp"
// #include "ex2.cpp"

#include <chrono>
#include <fstream>
#include <string>

using namespace std::chrono;
using namespace std;

int main()
{
    // Planta* planta = newPlanta(130);
    // carregaJSON("data/mapa.json", planta);

    ofstream file("times.csv", ios::out);

    if (!file.is_open()) {
        std::cerr << "Erro ao abrir o arquivo!" << std::endl;
        return 1;
    }

    file << "problem, V, E, time;" << endl;

    vector<int> Es(3);
    Planta* planta;
    pair<vector<int>, vector<Segmento*>> return1;
    vector<int> return2;

    for (int V = 10; V <= 10000; V = V * 10)
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
            cout << 1 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;
            file << 1 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;

            // timeStart = high_resolution_clock::now();
            // return2 = bus(planta);
            // timeStop = high_resolution_clock::now();
            // timeDuration = duration_cast<nanoseconds>(timeStop - timeStart).count();
            // cout << 2 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;
            // file << 2 << ", " << V << ", " << E << ", " << timeDuration << ";" << endl;
        }
    }

    file.close();

    return 0;
}