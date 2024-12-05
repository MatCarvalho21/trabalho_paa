#include <random>
#include <algorithm>
#include <queue>
#include <map>
#include "estrutura.h"
#include "mapaRandom.h"
#include <sstream>
#include <utility>
#include <climits>

using namespace std;


template <typename T>
std::string to_string(const T& value) {
    std::ostringstream oss;
    oss << value;
    return oss.str();
}

// Função auxiliar para gerar nome aleatório de rua
string geraNomeRua() {
    static int contador = 1;
    return "Rua " + to_string(contador++);
}

// Função auxiliar para verificar se uma aresta cruza com outras existentes
bool verificaCruzamento(const vector<pair<int,int>>& arestas, pair<int,int> novaAresta) {
    for (const auto& aresta : arestas) {
        if (aresta.first != novaAresta.first && aresta.first != novaAresta.second &&
            aresta.second != novaAresta.first && aresta.second != novaAresta.second) {
            if ((aresta.first < novaAresta.first && aresta.second > novaAresta.second) ||
                (aresta.first > novaAresta.first && aresta.second < novaAresta.second)) {
                return true;
            }
        }
    }
    return false;
}

// Função auxiliar para gerar imóveis em um segmento
void geraImoveis(Segmento* seg, RuaInfo& rua) {
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_int_distribution<> numImoveis(1, 5);
    std::uniform_int_distribution<> tipoImovel(0, 3);
    
    int quantidade = numImoveis(gen);
    vector<int> posicoes;
    
    for(int i = 0; i < quantidade; i++) {
        std::uniform_int_distribution<> pos(0, seg->tamanho);
        posicoes.push_back(pos(gen));
    }
    sort(posicoes.begin(), posicoes.end());
    
    const vector<string> tipos = {"residencial", "comercial", "industrial", "turismo"};
    
    for(int pos : posicoes) {
        int num = rua.numAtual;
        rua.numAtual += 2;
        
        string tipo = tipos[tipoImovel(gen)];
        Imovel* imovel = newImovel(seg->tamanho - pos, num, tipo);
        adicionaImovelASegmento(imovel, seg);
    }
}

// Função para determinar o CEP de um vértice baseado na proximidade aos centroides
int determinaCEP(int vertice, const vector<CEPInfo>& ceps, const vector<vector<int>>& distancias) {
    int menorDist = INT_MAX;
    int cepEscolhido = ceps[0].numero;
    
    for (const auto& cep : ceps) {
        int dist = distancias[vertice][cep.centroide];
        if (dist < menorDist) {
            menorDist = dist;
            cepEscolhido = cep.numero;
        }
    }
    return cepEscolhido;
}

// Função para calcular distâncias entre todos os vértices
vector<vector<int>> calculaDistancias(int numVertices, const vector<pair<int,int>>& arestas) {
    vector<vector<int>> dist(numVertices, vector<int>(numVertices, INT_MAX));
    
    // Inicializamos distâncias
    for (int i = 0; i < numVertices; i++) {
        dist[i][i] = 0;
    }
    for (const auto& aresta : arestas) {
        dist[aresta.first][aresta.second] = 1;
        dist[aresta.second][aresta.first] = 1;
    }
    
    for (int k = 0; k < numVertices; k++) {
        for (int i = 0; i < numVertices; i++) {
            for (int j = 0; j < numVertices; j++) {
                if (dist[i][k] != INT_MAX && dist[k][j] != INT_MAX) {
                    dist[i][j] = std::min(dist[i][j], dist[i][k] + dist[k][j]);
                }
            }
        }
    }
    
    return dist;
}

// Função para gerar uma planta dado um núemro de vértices e um número de arestas
Planta* geraPlantaAutomatica(int numVertices, int numArestas) {
    if (numVertices < 2 || numArestas < numVertices - 1 || 
        numArestas > 3 * numVertices - 6) {
        return nullptr;
    }
    
    std::random_device rd;
    std::mt19937 gen(rd());
    
    Planta* planta = newPlanta(numVertices);
    vector<pair<int,int>> arestasExistentes;
    map<string, RuaInfo> ruas;
    vector<CEPInfo> ceps;
    
    // Criamos mais ou menos 1 cep para cada 30
    int numCEPs = std::max(2, numVertices / 30); 
    for (int i = 0; i < numCEPs; i++)
    {
        planta -> CEPs.insert(i);
    }

    // Criamos a árvore geradora para a conexidade
    vector<int> verticesDisponiveis;
    for(int i = 0; i < numVertices; i++) {
        verticesDisponiveis.push_back(i);
    }
    std::shuffle(verticesDisponiveis.begin(), verticesDisponiveis.end(), gen);
    
    // Escolhemos centróides
    for(int i = 0; i < numCEPs; i++) {
        CEPInfo cep;
        cep.numero = i;
        cep.centroide = verticesDisponiveis[i];
        ceps.push_back(cep);
    }
    
    // Criamos árvore geradora
    for(int i = 1; i < numVertices; i++) {
        int vAnterior = std::uniform_int_distribution<>(0, i-1)(gen);
        arestasExistentes.push_back({vAnterior, i});
        
        string nomeRua = geraNomeRua();
        RuaInfo rua;
        rua.nome = nomeRua;
        rua.numAtual = 2;
        rua.maoUnica = std::uniform_int_distribution<>(0, 1)(gen) == 1;
        
        // Temporariamente atribuimos ao primeiro CEP
        Segmento* seg = newSegmento(vAnterior, i, 
                                   std::uniform_int_distribution<>(30, 60)(gen),
                                   std::uniform_int_distribution<>(100, 500)(gen),
                                   ceps[0].numero, 
                                   nomeRua,
                                   !rua.maoUnica);
        
        geraImoveis(seg, rua);
        adicionaSegmentoAPlanta(seg, planta);

        rua.segmentos.push_back(seg);
        ruas[nomeRua] = rua;
        
        if (!rua.maoUnica) {
            Segmento* segInverso = newSegmento(i, vAnterior,
                                             seg->limVel,
                                             seg->tamanho,
                                             seg->CEP,
                                             nomeRua,
                                             true);
            adicionaSegmentoAPlanta(segInverso, planta);
            rua.segmentos.push_back(segInverso);
        }
    }
    
    // Calculamos as distâncias entre todos os vértices
    vector<vector<int>> distancias = calculaDistancias(numVertices, arestasExistentes);
    
    // Reatribuimos os CEPs baseado na proximidade aos centroides
    for (auto& segVec : planta->listaAdj) {
        for (auto& seg : segVec) {
            int cepVSaida = determinaCEP(seg->vSaida, ceps, distancias);
            int cepVEntrada = determinaCEP(seg->vEntrada, ceps, distancias);
            
            // Atribui o CEP baseado no vértice de saída
            seg->CEP = cepVSaida;
            
            // Adiciona os vértices à lista do CEP correspondente
            for (auto& cep : ceps) {
                if (cep.numero == cepVSaida) {
                    if (find(cep.vertices.begin(), cep.vertices.end(), seg->vSaida) == cep.vertices.end()) {
                        cep.vertices.push_back(seg->vSaida);
                    }
                }
            }
        }
    }
    
    // Adicionamos arestas restantes
    int arestasAdicionadas = numVertices - 1;
    int tentativas = 0;
    const int maxTentativas = 1000;
    
    while(arestasAdicionadas < numArestas && tentativas < maxTentativas) {
        int v1 = std::uniform_int_distribution<>(0, numVertices-1)(gen);
        int v2 = std::uniform_int_distribution<>(0, numVertices-1)(gen);
        
        if (v1 != v2) {
            pair<int,int> novaAresta = {std::min(v1,v2), std::max(v1,v2)};
            
            if (find(arestasExistentes.begin(), arestasExistentes.end(), novaAresta) == arestasExistentes.end() &&
                !verificaCruzamento(arestasExistentes, novaAresta)) {
                
                arestasExistentes.push_back(novaAresta);
                
                string nomeRua = geraNomeRua();
                RuaInfo rua;
                rua.nome = nomeRua;
                rua.numAtual = 2;
                rua.maoUnica = std::uniform_int_distribution<>(0, 1)(gen) == 1;
                
                int cepEscolhido = determinaCEP(v1, ceps, distancias);
                
                Segmento* seg = newSegmento(v1, v2,
                                          std::uniform_int_distribution<>(30, 60)(gen),
                                          std::uniform_int_distribution<>(100, 500)(gen),
                                          cepEscolhido,
                                          nomeRua,
                                          !rua.maoUnica);
                
                geraImoveis(seg, rua);
                adicionaSegmentoAPlanta(seg, planta);
                
                arestasAdicionadas++;
            }
        }
        tentativas++;
    }
    
    return planta;
}