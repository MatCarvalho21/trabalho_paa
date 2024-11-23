#include "estrutura.h"
#include "mapa.h"

int main()
{
    Planta* planta = newPlanta(130);

    carregaJSON("mapa.json", planta);

    // Exibe a lista de adjacência para verificação
    for (size_t i = 0; i < planta->listaAdj.size(); ++i) {
        cout << "Vértice " << i << ":\n";
        for (size_t j = 0; j < planta->listaAdj[i].size(); ++j) {
            Segmento* segmento = planta->listaAdj[i][j];
            cout << "  Segmento de " << segmento->vSaida << " para " << segmento->vEntrada << endl;
            cout << "    Rua: " << segmento->rua << ", Limite de Velocidade: " << segmento->limVel
                 << ", Tamanho: " << segmento->tamanho << endl;
        }
    }

    return 0;
}