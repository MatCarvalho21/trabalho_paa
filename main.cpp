#include "estrutura.h"
#include "mapa.h"
#include "algoritmosBase.h"
#include "ex1.h"

int main()
{
    Planta* planta = newPlanta(130);

    carregaJSON("mapa.json", planta);

    return 0;
}