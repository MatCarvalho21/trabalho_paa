#include "estrutura.h"
#include "ex2.h"
#include "algoritmos.h"
#include "ex3.h"
#include "mapaRandom.h"
#include <vector>
#include <set>
#include <iostream>
#include <climits>
#include <utility>
#include <queue>
#include <unordered_map>
#include <limits>
#include <algorithm>

using namespace std;
const double INF = numeric_limits<double>::max();

const double VelocidadeMetro = 70.0;
const double VelocidadeAndar = 5.0;

// variáveis táxi
double limite_km = 2.0;
double taxa_variavel = 3.0;
double taxa_fixa = 7.0;

// passagens
double passagem_metro = 2.0;
double passagem_onibus = 1.5;

pair<vector<int>, vector<int>> dijkstraMetro(Planta* mstMetro, int origem)
{
    int numVertices = mstMetro->listaAdj.size();

    vector<int> distancias(numVertices, INT_MAX);
    vector<int> predecessores(numVertices, -1); // Para reconstruir o caminho
    
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> filaPrioridade;
    
    distancias[origem] = 0;
    filaPrioridade.push({0, origem});

    while (!filaPrioridade.empty())
    {
        pair<int, int> topo = filaPrioridade.top();
        int distAtual = topo.first;
        int verticeAtual = topo.second;

        filaPrioridade.pop();

        if (distAtual > distancias[verticeAtual]) { continue; }

        vector<Segmento*> segmentos = mstMetro->listaAdj[verticeAtual];
        for (Segmento* segmento : segmentos)
        {
            int vizinho = segmento->vEntrada;
            int peso = segmento->tamanho;

            if (distancias[verticeAtual] + peso < distancias[vizinho])
            {
                distancias[vizinho] = distancias[verticeAtual] + peso;
                predecessores[vizinho] = verticeAtual;
                filaPrioridade.push({distancias[vizinho], vizinho});
            }
        }
    }
    return {distancias, predecessores};
}

vector<pair<pair<int, int>, double>> achaArestasMetro(Planta* mstMetro, vector<int> estacoesMetro)
{
    vector<pair<pair<int, int>, double>> arestasMetro;

    for (int i = 0; i < estacoesMetro.size(); i++)
    {
        pair<vector<int>, vector<int>> resultado = dijkstraMetro(mstMetro, estacoesMetro[i]);
        vector<int> distancias = resultado.first;
        vector<int> predecessores = resultado.second;

        for (int j = 0; j < estacoesMetro.size(); j++)
        {
            if (i == j) { continue; }

            double peso = (distancias[estacoesMetro[j]])/1000.0;
            arestasMetro.push_back({{estacoesMetro[i], estacoesMetro[j]}, peso});
        }
    }
    return arestasMetro;
}

vector<pair<double, double>> calculaDistTempoCiclo(Planta* planta, vector<int> ciclo, int start)
{
    int n = ciclo.size();
    vector<pair<double, double>> distanciasTempos;
    distanciasTempos.resize(n);

    int startIndex = -1;
    for (int i = 0; i < n; i++)
    {
        if (ciclo[i] == start)
        {
            startIndex = i;
            break;
        }
    }
    distanciasTempos[startIndex] = {0, 0};

    int endIndex = startIndex - 1;
    if (endIndex < 0) { endIndex = n - 1; }

    for (int i = 0; i < n - 1; i++)
    {
        vector<Segmento*> segmentos = planta->listaAdj[ciclo[startIndex]];
        
        int nextIndex = (startIndex + i) % n;
        
        distanciasTempos[nextIndex] = distanciasTempos[startIndex];

        for (Segmento* segmento : segmentos)
        {
            if (segmento->vEntrada == ciclo[nextIndex])
            {
                double distanciaKM = segmento->tamanho/1000.0;
                distanciasTempos[nextIndex].first += distanciaKM;
                distanciasTempos[nextIndex].second += (distanciaKM / segmento->limVel);
                break;
            }
        }
    }

    return distanciasTempos;
}

vector<pair<pair<int, int>, pair<double, double>>> achaArestasOnibus(Planta* planta, vector<int> cicloBus)
{
    vector<int> cicloTemp = cicloBus;
    cicloTemp.pop_back();

    vector<pair<pair<int, int>, pair<double, double>>> arestasOnibus;

    for (int i = 0; i < cicloTemp.size(); i++)
    {
        vector<pair<double, double>> distanciasTempos = calculaDistTempoCiclo(planta, cicloTemp, cicloTemp[i]);

        for (int j = 0; j < cicloTemp.size(); j++)
        {
            if (i == j) { continue; }
            arestasOnibus.push_back({{cicloTemp[i], cicloTemp[j]}, {distanciasTempos[j].first, distanciasTempos[j].second}});
        }
    }
    return arestasOnibus;
}

PlantaBusca* constroiPlantaBusca(Planta* planta, vector<int> cicloBus, Planta* mstMetro, vector<int> estacoesMetro)
{
    int nVertices = planta->listaAdj.size();
    PlantaBusca* plantaBusca = newPlantaBusca(nVertices * 2);

    for (int i = 0; i < nVertices; i++)
    {
        vector<Segmento*> segmentos = planta->listaAdj[i];
        for (Segmento* segmento : segmentos)
        {
            double segmentoKm = segmento->tamanho;
            SegmentoBusca* segmentoAndar = newSegmentoBusca(
                i,
                segmento->vEntrada,
                segmentoKm,
                segmentoKm / VelocidadeAndar,
                "andar"
            );

            SegmentoBusca* segmentoTaxi = newSegmentoBusca(
                i + nVertices,
                segmento->vEntrada + nVertices,
                segmentoKm,
                segmentoKm / segmento->limVel,
                "taxi"
            );

            plantaBusca->adicionaSegmento(segmentoAndar);
            plantaBusca->adicionaSegmento(segmentoTaxi);

            SegmentoBusca* segmentoConexaoIda = newSegmentoBusca(
                i,
                i + nVertices,
                0,
                0,
                "andar"
            );

            SegmentoBusca* segmentoConexaoVolta = newSegmentoBusca(
                i + nVertices,
                i,
                0,
                0,
                "taxi"
            );
            segmentoConexaoIda->vertical = true;
            segmentoConexaoVolta->vertical = true;

            plantaBusca->adicionaSegmento(segmentoConexaoIda);
            plantaBusca->adicionaSegmento(segmentoConexaoVolta);

            if (segmento->dupla) { continue; }
            else
            {
                SegmentoBusca* segmentoAndar2 = newSegmentoBusca(
                    segmento->vEntrada,
                    i,
                    segmentoKm,
                    segmentoKm / VelocidadeAndar,
                    "andar"
                );
                plantaBusca->adicionaSegmento(segmentoAndar2);
            }
        }
    }

    vector<pair<pair<int, int>, double>> arestasMetro = achaArestasMetro(mstMetro, estacoesMetro);
    for (pair<pair<int, int>, double> arestaMetro : arestasMetro)
    {
        pair<int, int> aresta = arestaMetro.first;
        double distancia = arestaMetro.second;

        SegmentoBusca* segmentoMetro = newSegmentoBusca(
            aresta.first,
            aresta.second,
            distancia,
            distancia / VelocidadeMetro,
            "metro"
        );
        plantaBusca->adicionaSegmento(segmentoMetro);
    }

    vector<pair<pair<int, int>, pair<double, double>>> arestasOnibus = achaArestasOnibus(planta, cicloBus);

    for (pair<pair<int, int>, pair<int, double>> arestaOnibus : arestasOnibus)
    {
        pair<int, int> aresta = arestaOnibus.first;
        pair<int, double> distTempo = arestaOnibus.second;

        SegmentoBusca* segmentoOnibus = newSegmentoBusca(
            aresta.first,
            aresta.second,
            distTempo.first,
            distTempo.second,
            "onibus"
        );
        plantaBusca->adicionaSegmento(segmentoOnibus);
    }

    return plantaBusca;
}

pair<double, double> calcula_custo_taxi(int origem, int destino, double dist_taxi, SegmentoBusca* adjacente) {
    double segmento_tamanho = adjacente->distancia;
    double nova_distancia = dist_taxi + segmento_tamanho;
    double custo = 0.0;

    // Calcula custo variável caso exceda o limite de km gratuitos
    if (nova_distancia > limite_km) {
        custo = taxa_variavel * (nova_distancia - limite_km);
    }

    return {custo, nova_distancia};
}

pair<double, double> calcula_custo(SegmentoBusca* atual, SegmentoBusca* adjacente, double distancia_taxi) {
    if (atual->meioTransporte != adjacente->meioTransporte) {
        if (adjacente->meioTransporte == "metro") {
            return {passagem_metro, distancia_taxi}; // Exemplo: custo fixo para mudar para o metrô
        }
        if (adjacente->meioTransporte == "onibus") {
            return {passagem_onibus, distancia_taxi}; // Exemplo: custo fixo para mudar para ônibus
        }
    }
    if (adjacente->meioTransporte == "taxi" && adjacente->vertical == true) {
        return {taxa_fixa, distancia_taxi}; // Exemplo: custo fixo para mudar para táxi
    }

    if (adjacente->meioTransporte == "taxi" && adjacente->vertical == false) {
        return calcula_custo_taxi(atual->vDestino, adjacente->vDestino, distancia_taxi, adjacente);
    }
    return {0.0, distancia_taxi}; // Não muda de meio de transporte
}

vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca& grafo, int vertice_inicial, int vertice_destino, double lim_dinheiro) {
    // Mapas para armazenar o menor tempo e o "pai" de cada segmento
    unordered_map<SegmentoBusca*, double> tempo_minimo;
    unordered_map<SegmentoBusca*, double> custo_acumulado;
    unordered_map<SegmentoBusca*, SegmentoBusca*> segmento_pai;

    // Fila de prioridade (menor custo no topo)
    priority_queue<Estado, vector<Estado>, greater<>> fila;

    // Inicialização com todos os tempos e custos como infinito
    for (const auto& adjacencias : grafo.listaAdj) {
        for (SegmentoBusca* segmento : adjacencias) {
            tempo_minimo[segmento] = INF;
            custo_acumulado[segmento] = INF;
        }
    }

    // Processa os segmentos saindo do vértice inicial
    for (SegmentoBusca* segmento : grafo.listaAdj[vertice_inicial]) {
        tempo_minimo[segmento] = 0.0; // Tempo inicial é zero
        custo_acumulado[segmento] = 0.0; // Custo inicial é zero
        fila.push({segmento, 0.0, 0.0, 0}); // Inicializa com distância_taxi = 0
        segmento_pai[segmento] = nullptr; // Sem pai para o primeiro segmento
    }

    // Processamento do algoritmo de Dijkstra
    while (!fila.empty()) {
        Estado estado_atual = fila.top();
        fila.pop();

        SegmentoBusca* segmento_atual = estado_atual.segmento;

        // Se o custo acumulado atual for maior que o limite, ignorar
        if (estado_atual.custo_acumulado > lim_dinheiro) {
            continue;
        }

        // Se o segmento atual for o destino, interrompa
        if (segmento_atual->vDestino == vertice_destino) {
            break;
        }

        // Iterar sobre os segmentos adjacentes
        for (SegmentoBusca* adjacente : grafo.listaAdj[segmento_atual->vDestino]) {
            double custo_aux, nova_distancia_taxi;

            // Calcula o custo para o segmento adjacente
            tie(custo_aux, nova_distancia_taxi) = calcula_custo(segmento_atual, adjacente, estado_atual.distancia_taxi);

            double novo_custo = estado_atual.custo_acumulado + custo_aux;
            double novo_tempo = estado_atual.tempo_acumulado + adjacente->tempo;

            // Atualiza se encontrar um custo menor e dentro do limite de dinheiro
            if (novo_custo <= lim_dinheiro && novo_tempo < tempo_minimo[adjacente]) {
                tempo_minimo[adjacente] = novo_tempo;
                custo_acumulado[adjacente] = novo_custo;
                fila.push({adjacente, novo_custo, nova_distancia_taxi, novo_tempo});
                segmento_pai[adjacente] = segmento_atual;
            }
        }
    }

    // Reconstruir o caminho com base no "pai" de cada segmento
    vector<SegmentoBusca*> caminho;
    SegmentoBusca* segmento_atual = nullptr;

    // Encontrar o segmento de destino com menor tempo
    double menor_tempo = INF;
    for (const auto& [segmento, tempo] : tempo_minimo) {
        if (tempo < menor_tempo && segmento->vDestino == vertice_destino) {
            menor_tempo = tempo;
            segmento_atual = segmento;
        }
    }

    // Reconstrução do caminho
    while (segmento_atual != nullptr) {
        caminho.push_back(segmento_atual);
        segmento_atual = segmento_pai[segmento_atual];
    }

    // Inverte o caminho para começar do vértice inicial
    reverse(caminho.begin(), caminho.end());

    return caminho;
}

int main(){
    // SegmentoBusca* seg1 = newSegmentoBusca(0, 1, 10, 50.0, "taxi");
    // SegmentoBusca* seg2 = newSegmentoBusca(1, 2, 10, 50.0, "taxi");
    // SegmentoBusca* seg3 = newSegmentoBusca(2, 3, 10, 100.0, "andar");
    // SegmentoBusca* seg4 = newSegmentoBusca(3, 4, 10, 50.0, "onibus");
    // SegmentoBusca* seg5 = newSegmentoBusca(4, 5, 10, 50.0, "onibus");
    // SegmentoBusca* seg6 = newSegmentoBusca(5, 6, 10, 25.0, "metro");
    // SegmentoBusca* seg7 = newSegmentoBusca(6, 7, 10, 100, "andar");
    // SegmentoBusca* seg8 = newSegmentoBusca(7, 8, 0, 0, "taxi");
    // seg8->vertical = true;

    // cout << "TESTE: calcula_custo_taxi()" << endl;
    // cout << "Custo: " << calcula_custo_taxi(0, 1, 0.0, seg2).first << endl;
    // cout << "Distância: " << calcula_custo_taxi(0, 1, 0.0, seg2).second << endl;

    // cout << "Custo: " << calcula_custo_taxi(0, 1, 50, seg2).first << endl;
    // cout << "Distância: " << calcula_custo_taxi(0, 1, 50, seg2).second << endl;

    // cout << "TESTE: calcula_custo() - Táxi to Táxi" << endl;
    // cout << "Custo: " << calcula_custo(seg1, seg2, 100).first << endl;
    // cout << "Distância: " << calcula_custo(seg1, seg2, 100).second << endl;

    // cout << "TESTE: calcula_custo() - Táxi to Andando" << endl;
    // cout << "Custo: " << calcula_custo(seg1, seg3, 100).first << endl;
    // cout << "Distância: " << calcula_custo(seg1, seg3, 100).second << endl;

    // cout << "TESTE: calcula_custo() - Andando to Ônibus" << endl;
    // cout << "Custo: " << calcula_custo(seg3, seg4, 100).first << endl;
    // cout << "Distância: " << calcula_custo(seg3, seg4, 100).second << endl;

    // cout << "TESTE: calcula_custo() - Ônibus to Ônibus" << endl;
    // cout << "Custo: " << calcula_custo(seg4, seg5, 100).first << endl;
    // cout << "Distância: " << calcula_custo(seg4, seg5, 100).second << endl;

    // cout << "TESTE: calcula_custo() - Ônibus to Metrô" << endl;
    // cout << "Custo: " << calcula_custo(seg5, seg6, 100).first << endl;
    // cout << "Distância: " << calcula_custo(seg5, seg6, 100).second << endl;

    // cout << "TESTE: calcula_custo() - Andando to Táxi (vertical)" << endl;
    // cout << "Custo: " << calcula_custo(seg7, seg8, 0).first << endl;
    // cout << "Distância: " << calcula_custo(seg7, seg8, 0).second << endl;

    int origem = 10;
    int destino = 40;
    int dinheiro = 100;

    cout << "TESTE: geraPlantaAutomatica()" << endl;

    Planta* planta = geraPlantaAutomatica(120, 250);

    cout << "CEPs: ";
    for (int cep : planta->CEPs) {
        cout << cep << " ";
    }
    cout << endl;

    cout << "TESTE: bus()" << endl;

    vector<int> cicloOnibus =  bus(planta);

    cout << "Ciclo: ";
    for (int elemento : cicloOnibus) {
        cout << elemento << " ";
    }
    cout << endl;

    cout << "TESTE: subway()" << endl;

    pair<vector<int>, vector<Segmento*>> metro = subway(planta, 120);

    vector<int> estacoesMetro = metro.first;
    vector<Segmento*> mstMetroSeg = metro.second;


    cout << "Estações de metrô: ";
    for (int estacao : estacoesMetro) {
        cout << estacao << " ";
    }
    cout << endl;

    Planta* mstMetro = newPlanta(120);

    cout << "Segmentos de metrô: ";
    for (Segmento* segmento : mstMetroSeg) {
        cout << "(" << segmento->vSaida << ", " << segmento->vEntrada << ") ";
        adicionaSegmentoAPlanta(segmento, mstMetro);
    }
    cout << endl;
    
    cout << "TESTE: achaArestasMetro()" << endl;
    vector<pair<pair<int, int>, double>> resultado = achaArestasMetro(mstMetro, estacoesMetro);

    cout << "Arestas de metrô: ";
    for (pair<pair<int, int>, double> aresta : resultado) {
        cout << "(" << aresta.first.first << ", " << aresta.first.second << ") ";
    }
    cout << endl;

    cout << "Tempo: ";
    for (pair<pair<int, int>, double> aresta : resultado) {
        cout << aresta.second << " ";
    }
    cout << endl;

    cout << "TESTE: achaArestasOnibus()" << endl;

    vector<pair<pair<int, int>, pair<double, double>>> resultado2 = achaArestasOnibus(planta, cicloOnibus);

    cout << "Arestas de ônibus: ";
    for (pair<pair<int, int>, pair<int, double>> aresta : resultado2) {
        cout << "(" << aresta.first.first << ", " << aresta.first.second << ") ";
    }
    cout << endl;

    cout << "Distância e tempo: ";
    for (pair<pair<int, int>, pair<int, double>> aresta : resultado2) {
        cout << "(" << aresta.second.first << ", " << aresta.second.second << ") ";
    }
    cout << endl;




    return 0; 
}


