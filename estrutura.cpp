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

typedef struct segmento {
    vector<imovel> imoveis;
    int limVel;
    int tamanho;
    string rua;
}

