/**
 * @file mapa.h
 * @brief Módulo para definição dos protótipos das funções de criação do mapa.
 */

#ifndef MAPA_H
#define MAPA_H

int stringToInt(const std::string&);
string trim(const string&);
string removeQuotes(const string&);
string extractValue(const string&);
void carregaJSON(const string&, Planta*);

#endif