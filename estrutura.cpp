#include <iostream>
#include <string>
#include <vector>

using namespace std;


typedef struct Imovel{
    int CEP;
    string rua;
    int num;
    string tipo;    
} imovel;

typedef struct segmento{
    int id;
    vector<imovel> imoveis;
    int limVel;
    int tamanho;
    string rua;
} segmento;

typedef struct planta{
    vector<vector<segmento>> listaAdj;
} planta;

int main(){
    vector<segmento> todosSegs;

    return 0;
}


