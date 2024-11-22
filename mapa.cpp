#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>

#include "estrutura.h"

using std::string;
using std::vector;
using std::cout;
using std::endl;

// Função para converter string para inteiro
int stringToInt(const std::string& str) {
    std::stringstream ss(str);
    int result;
    ss >> result;
    return result;
}

// Função para remover caracteres específicos de uma string
string removeChar(const string& str, char ch) {
    string result;
    for (size_t i = 0; i < str.size(); ++i) {
        if (str[i] != ch) {
            result += str[i];
        }
    }
    return result;
}

// Função para dividir uma string com base em um delimitador
vector<string> split(const string& str, char delimiter) {
    vector<string> tokens;
    std::stringstream ss(str);
    string token;
    while (std::getline(ss, token, delimiter)) {
        tokens.push_back(token);
    }
    return tokens;
}

// Função para carregar o JSON e criar os segmentos
void carregaJSON(const string& caminhoArquivo, Planta* planta) {
    std::ifstream arquivo(caminhoArquivo.c_str()); // Usando c_str() para compatibilidade com C++98
    if (!arquivo.is_open()) {
        cout << "Erro ao abrir o arquivo JSON." << endl;
        return;
    }

    string linha;
    Segmento* segmento = NULL;

    while (std::getline(arquivo, linha)) {
        linha = removeChar(linha, ' ');
        linha = removeChar(linha, '"');
        linha = removeChar(linha, ',');

        if (linha.find("vSaida") != string::npos) {
            vector<string> partes = split(linha, ':');
            int vSaida = stringToInt(partes[1]);
            segmento = new Segmento();
            segmento->vSaida = vSaida;
        }
        else if (linha.find("vEntrada") != string::npos) {
            vector<string> partes = split(linha, ':');
            segmento->vEntrada = stringToInt(partes[1]);
        }
        else if (linha.find("limVel") != string::npos) {
            vector<string> partes = split(linha, ':');
            segmento->limVel = stringToInt(partes[1]);
        }
        else if (linha.find("tamanho") != string::npos) {
            vector<string> partes = split(linha, ':');
            segmento->tamanho = stringToInt(partes[1]);
        }
        else if (linha.find("rua") != string::npos) {
            vector<string> partes = split(linha, ':');
            segmento->rua = partes[1];

            // Adiciona o segmento à planta depois de completar os dados
            if (segmento->vSaida >= planta->listaAdj.size()) {
                planta->listaAdj.resize(segmento->vSaida + 1);
            }
            planta->listaAdj[segmento->vSaida].push_back(segmento);
            segmento = NULL; // Reseta o ponteiro para o próximo segmento
        }
    }
}