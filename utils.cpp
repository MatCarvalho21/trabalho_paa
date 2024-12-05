#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <climits>
#include <random>
#include "estrutura.h"
#include "utils.h"

using namespace std;

void printSegmento(const Segmento* seg) {
    cout << "Segmento:" << endl;
    cout << "  vSaida: " << seg->vSaida << endl;
    cout << "  vEntrada: " << seg->vEntrada << endl;
    cout << "  Transito: " << seg->transito << endl;
    cout << endl;
}

void printSet(const set<Segmento*>& segmentos) {
    for (const Segmento* seg : segmentos) {
        printSegmento(seg);
    }
}

void adicionaTransito(Planta* planta) {
    int nVertices = planta->listaAdj.size();

    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> transitoAleatorio(0.2, 1.0); // Valores entre 0.2 e 1.0

    for (int i = 0; i < nVertices; i++) {
        vector<Segmento*> segmentos = planta->listaAdj[i];

        for (Segmento* seg : segmentos) {
            seg->transito = transitoAleatorio(gen); // Atribui um valor aleatório de trânsito
        }
    }
}
