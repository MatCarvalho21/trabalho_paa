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

    // set<Segmento*> ex1 = subway(planta, 130);

    // // Chama a função para imprimir o set
    // printSet(ex1);

    Planta* planta = geraPlantaAutomatica(200, 400);

    for (int i = 0; i < 100; i++)
    {
        vector<Segmento*> segs = planta -> listaAdj[i];
        for (int j = 0; j < segs.size(); j++)
        {
            Segmento* seg = segs[j];
            cout << "Segmento:" << endl;
            cout << "   vSaida: " << seg -> vSaida << endl;
            cout << "   vEntrada: " << seg -> vEntrada << endl;
            cout << "   " << endl;
            if (j > 20)
            {
                break;
            }
        }
    }

    return 0;
}