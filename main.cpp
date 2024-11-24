#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"

int main()
{
    Planta* planta = newPlanta(130);

    carregaJSON("mapa.json", planta);

    return 0;
}