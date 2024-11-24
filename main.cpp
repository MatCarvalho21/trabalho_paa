#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"
#include "ex1.h"
#include "utils.h"

int main()
{
    Planta* planta = newPlanta(130);

    carregaJSON("mapa.json", planta);

    set<Segmento*> ex1 = subway(planta, 130);

    // Chama a função para imprimir o set
    printSet(ex1);

    return 0;
}