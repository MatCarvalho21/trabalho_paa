#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"
#include "ex1.h"
#include "utils.h"
#include "mapa_random.h"
#include "estrutura.cpp"
#include "mapa.cpp"
#include "algoritmosBase.cpp"
#include "ex1.cpp"
#include "utils.cpp"
#include "mapa_random.cpp"

int main()
{
    // Planta* planta = newPlanta(130);

    // carregaJSON("mapa.json", planta);



    // // Chama a função para imprimir o set
    // printSet(ex1);

    Planta* planta = geraPlantaAutomatica(200, 300);
    

    // for (int i = 0; i < planta -> listaAdj.size(); i++)
    // {
    //     vector<Segmento*> segs = planta -> listaAdj[i];
    //     for (int j = 0; j < segs.size(); j++)
    //     {
    //         Segmento* seg = segs[j];
    //         cout << "Segmento:" << endl;
    //         cout << "   vSaida: " << seg -> vSaida << endl;
    //         cout << "   vEntrada: " << seg -> vEntrada << endl;
    //         cout << "   CEP: " << seg -> CEP << endl;
    //         cout << "   " << endl;
    //     }
    // }

    // Chama a função subway e armazena o resultado em um std::pair
    pair<vector<int>, vector<Segmento*>> resultado = subway(planta, 200);

    // Separando os valores do par
    vector<int> menorRota = resultado.first;
    vector<Segmento*> segmentosRota = resultado.second;


    for (int i = 0; i < segmentosRota.size(); i++)
    {
        Segmento* seg = segmentosRota[i];
        cout << "Segmento:" << endl;
        cout << "   vSaida: " << seg -> vSaida << endl;
        cout << "   vEntrada: " << seg -> vEntrada << endl;
        cout << "   CEP: " << seg -> CEP << endl;
        cout << "   " << endl;
    }

    return 0;
}