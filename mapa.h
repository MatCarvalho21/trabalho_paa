/**
 * @file mapa.h
 * @brief Módulo para definição dos protótipos das funções de criação do mapa.
 */

#ifndef MAPA_H
#define MAPA_H

int stringToInt(const std::string& str);
string removeChar(const string& str, char ch);
vector<string> split(const string& str, char delimiter);
void carregaJSON(const string& caminhoArquivo, Planta* planta);

#endif