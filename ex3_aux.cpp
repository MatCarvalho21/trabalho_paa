#include <queue>
#include <vector>
#include <unordered_map>
#include <limits>
#include <iostream>
#include "estrutura.h"
#include <algorithm>

using namespace std;
const float INF = numeric_limits<float>::max();

// variáveis táxi
float limite_km = 2.0;
float taxa_variavel = 3.0;
float taxa_fixa = 7.0;

// passagens
float passagem_metro = 2.0;
float passagem_onibus = 1.5;

pair<float, float> calcula_custo_taxi(int origem, int destino, float dist_taxi, SegmentoBusca* adjacente) {
    float segmento_tamanho = adjacente->distancia;
    float nova_distancia = dist_taxi + segmento_tamanho;
    float custo = 0.0;

    // Calcula custo variável caso exceda o limite de km gratuitos
    if (nova_distancia > limite_km) {
        custo = taxa_variavel * (nova_distancia - limite_km);
    }

    return {custo, nova_distancia};
}

pair<float, float> calcula_custo(SegmentoBusca* atual, SegmentoBusca* adjacente, float distancia_taxi) {
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

struct Estado {
    SegmentoBusca* segmento;
    float custo_acumulado;
    float distancia_taxi;
    float tempo_acumulado;

    // Comparação pelo operador '<' para a fila de prioridade
    bool operator<(const Estado& outro) const {
        return custo_acumulado > outro.custo_acumulado; // Menor custo tem maior prioridade
    }

    // Comparação pelo operador '>'
    bool operator>(const Estado& outro) const {
        return custo_acumulado < outro.custo_acumulado; // Inverso para std::greater
    }
};

vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca& grafo, int vertice_inicial, int vertice_destino, float lim_dinheiro) {
    // Mapas para armazenar o menor tempo e o "pai" de cada segmento
    unordered_map<SegmentoBusca*, float> tempo_minimo;
    unordered_map<SegmentoBusca*, float> custo_acumulado;
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
            float custo_aux, nova_distancia_taxi;

            // Calcula o custo para o segmento adjacente
            tie(custo_aux, nova_distancia_taxi) = calcula_custo(segmento_atual, adjacente, estado_atual.distancia_taxi);

            float novo_custo = estado_atual.custo_acumulado + custo_aux;
            float novo_tempo = estado_atual.tempo_acumulado + adjacente->tempo;

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
    float menor_tempo = INF;
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
    SegmentoBusca* seg1 = newSegmentoBusca(0, 1, 10, 50.0, "taxi");
    SegmentoBusca* seg2 = newSegmentoBusca(1, 2, 10, 50.0, "taxi");
    SegmentoBusca* seg3 = newSegmentoBusca(2, 3, 10, 100.0, "andando");
    SegmentoBusca* seg4 = newSegmentoBusca(3, 4, 10, 50.0, "onibus");
    SegmentoBusca* seg5 = newSegmentoBusca(4, 5, 10, 50.0, "onibus");
    SegmentoBusca* seg6 = newSegmentoBusca(5, 6, 10, 25.0, "metro");
    SegmentoBusca* seg7 = newSegmentoBusca(6, 7, 10, 100, "andando");
    SegmentoBusca* seg8 = newSegmentoBusca(7, 8, 0, 0, "taxi");
    seg8->vertical = true;

    cout << "TESTE: calcula_custo_taxi()" << endl;
    cout << "Custo: " << calcula_custo_taxi(0, 1, 0.0, seg2).first << endl;
    cout << "Distância: " << calcula_custo_taxi(0, 1, 0.0, seg2).second << endl;

    cout << "Custo: " << calcula_custo_taxi(0, 1, 50, seg2).first << endl;
    cout << "Distância: " << calcula_custo_taxi(0, 1, 50, seg2).second << endl;

    cout << "TESTE: calcula_custo() - Táxi to Táxi" << endl;
    cout << "Custo: " << calcula_custo(seg1, seg2, 100).first << endl;
    cout << "Distância: " << calcula_custo(seg1, seg2, 100).second << endl;

    cout << "TESTE: calcula_custo() - Táxi to Andando" << endl;
    cout << "Custo: " << calcula_custo(seg1, seg3, 100).first << endl;
    cout << "Distância: " << calcula_custo(seg1, seg3, 100).second << endl;

    cout << "TESTE: calcula_custo() - Andando to Ônibus" << endl;
    cout << "Custo: " << calcula_custo(seg3, seg4, 100).first << endl;
    cout << "Distância: " << calcula_custo(seg3, seg4, 100).second << endl;

    cout << "TESTE: calcula_custo() - Ônibus to Ônibus" << endl;
    cout << "Custo: " << calcula_custo(seg4, seg5, 100).first << endl;
    cout << "Distância: " << calcula_custo(seg4, seg5, 100).second << endl;

    cout << "TESTE: calcula_custo() - Ônibus to Metrô" << endl;
    cout << "Custo: " << calcula_custo(seg5, seg6, 100).first << endl;
    cout << "Distância: " << calcula_custo(seg5, seg6, 100).second << endl;

    cout << "TESTE: calcula_custo() - Andando to Táxi (vertical)" << endl;
    cout << "Custo: " << calcula_custo(seg7, seg8, 0).first << endl;
    cout << "Distância: " << calcula_custo(seg7, seg8, 0).second << endl;

    return 0; 
}