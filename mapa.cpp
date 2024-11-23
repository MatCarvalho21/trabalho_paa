#include <fstream>
#include <sstream>
#include "estrutura.h"

int stringToInt(const string& str) {
    std::istringstream ss(str);
    int num;
    ss >> num;
    return num;
}

string trim(const string& str) {
    size_t first = str.find_first_not_of(" \t\n\r");
    size_t last = str.find_last_not_of(" \t\n\r");
    if (first == string::npos) return "";
    return str.substr(first, (last - first + 1));
}

string removeQuotes(const string& str) {
    string trimmed = trim(str);
    if (trimmed.length() >= 2 && trimmed.front() == '"' && trimmed.back() == '"') {
        return trimmed.substr(1, trimmed.length() - 2);
    }
    return trimmed;
}

string extractValue(const string& line) {
    size_t colonPos = line.find(':');
    if (colonPos == string::npos) return "";
    string value = line.substr(colonPos + 1);
    if (!value.empty() && value.back() == ',') {
        value.pop_back();
    }
    return removeQuotes(value);
}

void carregaJSON(const string& filename, Planta* planta) {
    cout << "Iniciando leitura do arquivo " << filename << endl;
    
    std::ifstream file(filename.c_str());
    if (!file.is_open()) {
        cout << "Erro ao abrir o arquivo " << filename << endl;
        return;
    }

    string line;
    Segmento* currentSegment = nullptr;
    Imovel* currentImovel = nullptr;
    bool insideSegment = false;
    bool insideImovel = false;
    bool insideImoveis = false;
    int segmentCount = 0;
    int imovelCount = 0;

    while (std::getline(file, line)) {
        line = trim(line);
        
        // Início de um novo segmento
        if (line == "{" && !insideImovel && !insideImoveis) {
            insideSegment = true;
            currentSegment = new Segmento();
            currentSegment->vSaida = 0;
            currentSegment->vEntrada = 0;
            currentSegment->limVel = 0;
            currentSegment->tamanho = 0;
            currentSegment->CEP = 0;
            currentSegment->rua = "";
            currentSegment->dupla = false;
            currentSegment->imoveis = vector<Imovel*>();  // Inicializa vetor vazio
            cout << "Novo segmento iniciado" << endl;
            continue;
        }

        // Processamento dos campos do segmento
        if (insideSegment && !insideImovel) {
            // Ignoramos o campo "id" pois não está na estrutura
            if (line.find("\"vSaida\"") != string::npos) {
                currentSegment->vSaida = stringToInt(extractValue(line));
                cout << "vSaida: " << currentSegment->vSaida << endl;
            }
            else if (line.find("\"vEntrada\"") != string::npos) {
                currentSegment->vEntrada = stringToInt(extractValue(line));
                cout << "vEntrada: " << currentSegment->vEntrada << endl;
            }
            else if (line.find("\"limVel\"") != string::npos) {
                currentSegment->limVel = stringToInt(extractValue(line));
                cout << "limVel: " << currentSegment->limVel << endl;
            }
            else if (line.find("\"tamanho\"") != string::npos) {
                currentSegment->tamanho = stringToInt(extractValue(line));
                cout << "tamanho: " << currentSegment->tamanho << endl;
            }
            else if (line.find("\"rua\"") != string::npos) {
                currentSegment->rua = extractValue(line);
                cout << "rua: " << currentSegment->rua << endl;
            }
            else if (line.find("\"CEP\"") != string::npos) {
                currentSegment->CEP = stringToInt(extractValue(line));
                cout << "CEP: " << currentSegment->CEP << endl;
            }
            else if (line.find("\"volta\"") != string::npos) {
                currentSegment->dupla = (extractValue(line) == "true");
                cout << "dupla: " << currentSegment->dupla << endl;
            }
            else if (line.find("\"imoveis\"") != string::npos) {
                insideImoveis = true;
                cout << "Iniciando array de imóveis" << endl;
            }
        }

        // Início de um novo imóvel
        if (line == "{" && insideImoveis) {
            insideImovel = true;
            currentImovel = new Imovel();
            currentImovel->dFinalSeg = 0;
            currentImovel->num = 0;
            currentImovel->tipo = "";
            cout << "Novo imóvel iniciado" << endl;
            continue;
        }

        // Processamento dos campos do imóvel
        if (insideImovel) {
            if (line.find("\"dFinalSeg\"") != string::npos) {
                currentImovel->dFinalSeg = stringToInt(extractValue(line));
                cout << "dFinalSeg: " << currentImovel->dFinalSeg << endl;
            }
            else if (line.find("\"num\"") != string::npos) {
                currentImovel->num = stringToInt(extractValue(line));
                cout << "num: " << currentImovel->num << endl;
            }
            else if (line.find("\"tipo\"") != string::npos) {
                currentImovel->tipo = extractValue(line);
                cout << "tipo: " << currentImovel->tipo << endl;
            }
        }

        // Fim de um imóvel
        if (line == "}" && insideImovel) {
            insideImovel = false;
            cout << "Imóvel finalizado, adicionando ao segmento atual" << endl;
            cout << "Tamanho do vetor antes: " << currentSegment->imoveis.size() << endl;
            currentSegment->imoveis.push_back(currentImovel);
            cout << "Tamanho do vetor depois: " << currentSegment->imoveis.size() << endl;
            imovelCount++;
        }

        // Fim do array de imóveis
        if (line == "]" && insideImoveis) {
            insideImoveis = false;
            cout << "Array de imóveis finalizado com " << currentSegment->imoveis.size() << " imóveis" << endl;
        }

        // Fim de um segmento
        if (line == "}" && !insideImovel && !insideImoveis) {
            insideSegment = false;
            cout << "Finalizando segmento com " << currentSegment->imoveis.size() << " imóveis" << endl;
            adicionaSegmentoAPlanta(currentSegment, planta);
            segmentCount++;
        }
    }

    cout << "Finalizado processamento do arquivo" << endl;
    cout << "Total de segmentos processados: " << segmentCount << endl;
    cout << "Total de imóveis processados: " << imovelCount << endl;

    file.close();
}