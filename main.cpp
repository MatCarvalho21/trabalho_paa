#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"

int main()
{
    Planta* planta = newPlanta(130);

    carregaJSON("mapa.json", planta);

    // Percorre a lista de segmentos na planta
    for (size_t i = 0; i < planta->listaAdj.size(); ++i) {
        // Acessa o segmento atual
        vector<Segmento*> segmentos = (planta->listaAdj)[i];
        
        for (int j = 0; j < segmentos.size(); j++)
        {
            Segmento* segmento = segmentos[j];

            cout << "Segmento " << i << ":" << endl;
            cout << "  vSaida: " << segmento->vSaida << endl;
            cout << "  vEntrada: " << segmento->vEntrada << endl;
            cout << "  limVel: " << segmento->limVel << endl;
            cout << "  tamanho: " << segmento->tamanho << endl;
            cout << "  rua: " << segmento->rua << endl;
            cout << "  CEP: " << segmento->CEP << endl;
            cout << "  Volta (dupla): " << (segmento->dupla ? "Sim" : "Não") << endl;

            // Imprime os imóveis associados ao segmento
            if (!segmento->imoveis.empty()) {
                cout << "  Imóveis:" << endl;
                for (const auto& imovel : segmento->imoveis) {
                    cout << "    Número: " << imovel->num << endl;
                    cout << "    Tipo: " << imovel->tipo << endl;
                    cout << "    Data Final do Segmento: " << imovel->dFinalSeg << endl;
                }
            } else {
                cout << "  Não há imóveis associados a este segmento." << endl;
            }
            cout << endl;
        }
    }

    return 0;
}