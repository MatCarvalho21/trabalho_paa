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

    for (int V = 150; V <= 800; V = V + 100)
    {
        Es[0] = V;
        Es[1] = 2*V;
        Es[2] = 3*V - 6;

        for (int i = 0; i <= 2; i++)
        {
            int E = Es[i];
            Planta* planta = geraPlantaAutomatica(V, E);

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
            cout << 2 << "," << V << "," << E << "," << i+1 << "," << timeDuration << endl;
            file << 2 << "," << V << "," << E << "," << i+1 << "," << timeDuration << endl;

            std::random_device rd;
            std::mt19937 gen(rd());
            std::uniform_int_distribution<> verticeAleatorio(0, V-1);
            std::uniform_real_distribution<> dinheiro(0, 40);
            adicionaTransito(planta);

            timeDuration = 0;
            int epocas = 5;
            for (int j = 0; j < epocas; j++)
            {
                int v1 = verticeAleatorio(gen);
                int v2 = verticeAleatorio(gen);
                while (v1 == v2) { v2 = verticeAleatorio(gen); }
                float dinheiroAleatorio = dinheiro(gen);

                timeStart = high_resolution_clock::now();
                return3 = melhorRota(planta, return2, return1.second, return1.first, v1, v2, dinheiroAleatorio);
                timeStop = high_resolution_clock::now();
                timeDuration += duration_cast<nanoseconds>(timeStop - timeStart).count();
            }
            timeDuration = timeDuration/epocas;

            cout << 3 << "," << V << "," << E << "," << i+1 << "," << timeDuration << endl;
            file << 3 << "," << V << "," << E << "," << i+1 << "," << timeDuration << endl;

            delete planta;
        }
    }

    file.close();

    return 0;
}