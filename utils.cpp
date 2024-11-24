#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <climits>
#include "estrutura.h"
#include "utils.h"

using namespace std;

void printSegmento(const Segmento* seg) {
    cout << "Segmento:" << endl;
    cout << "  vSaida: " << seg->vSaida << endl;
    cout << "  vEntrada: " << seg->vEntrada << endl;
    cout << endl;
}

void printSet(const set<Segmento*>& segmentos) {
    for (const Segmento* seg : segmentos) {
        printSegmento(seg);
    }
}