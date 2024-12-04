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
float limite_metros = 10.0;
float taxa_variavel = 2.0;

// passagens
float passagem_metro = 2.0;
float passagem_onibus = 1.5;

pair<float, float> calcula_custo_taxi(int origem, int destino, float dist_taxi, SegmentoBusca* adjacente) {
    // Exemplo: Encontrar o segmento entre os vértices (simulado para simplificar)
    //float segmento_tamanho = adjacente->distancia;
    float segmento_tamanho = 10.0;
    float nova_distancia = dist_taxi + segmento_tamanho;
    float custo = 0.0;

    // Calcula custo variável caso exceda o limite de metros gratuitos
    if (nova_distancia > limite_metros) {
        custo = taxa_variavel * (nova_distancia - limite_metros);
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
    if (adjacente->meioTransporte == "taxi") {
        return calcula_custo_taxi(atual->vDestino, adjacente->vDestino, distancia_taxi, adjacente);
    }
    return {0.0, distancia_taxi}; // Não muda de meio de transporte
}


struct Estado {
    SegmentoBusca* segmento; // O segmento atual
    float custo_acumulado;   // Custo acumulado até o segmento
    float distancia_taxi;    // Distância acumulada para táxi

    // Ordena pelo menor custo acumulado
    bool operator<(const Estado& outro) const {
        return custo_acumulado > outro.custo_acumulado; // Para a fila de prioridade (menor custo tem maior prioridade)
    }
};

vector<SegmentoBusca*> dijkstra_custo(const PlantaBusca& grafo, int vertice_inicial, int vertice_destino, float lim_dinheiro) {
    // Mapas para armazenar o menor custo e o "pai" de cada segmento
    unordered_map<SegmentoBusca*, float> custo_minimo;
    unordered_map<SegmentoBusca*, SegmentoBusca*> segmento_pai;

    // Fila de prioridade (menor custo no topo)
    priority_queue<Estado> fila;

    // Inicialização com todos os custos como infinito
    for (const auto& adjacencias : grafo.listaAdj) {
        for (SegmentoBusca* segmento : adjacencias) {
            custo_minimo[segmento] = INF;
        }
    }

    // Processa os segmentos saindo do vértice inicial
    for (SegmentoBusca* segmento : grafo.listaAdj[vertice_inicial]) {
        custo_minimo[segmento] = 0.0; // Custo inicial é zero
        fila.push({segmento, 0.0, 0.0});
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

        // Se o segmento atual for o destino, pare o Dijkstra
        if (segmento_atual->vDestino == vertice_destino) {
            break;
        }

        // Iterar sobre os segmentos adjacentes
        for (SegmentoBusca* adjacente : grafo.listaAdj[segmento_atual->vDestino]) {
            float custo_aux, nova_distancia_taxi;

            // Calcula o custo para o segmento adjacente
            tie(custo_aux, nova_distancia_taxi) = calcula_custo(segmento_atual, adjacente, estado_atual.distancia_taxi);

            float novo_custo = estado_atual.custo_acumulado + custo_aux;

            // Atualiza se encontrar um custo menor
            if (novo_custo < custo_minimo[adjacente]) {
                custo_minimo[adjacente] = novo_custo;
                fila.push({adjacente, novo_custo, nova_distancia_taxi});
                segmento_pai[adjacente] = segmento_atual;
            }
        }
    }

    // Reconstruir o caminho com base no "pai" de cada segmento
    vector<SegmentoBusca*> caminho;
    SegmentoBusca* segmento_atual = nullptr;

    // Encontrar o segmento de destino com menor custo
    float menor_custo = INF;
    for (const auto& [segmento, custo] : custo_minimo) {
        if (custo < menor_custo) {
            menor_custo = custo;
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
    SegmentoBusca* seg1 = newSegmentoBusca(0, 1, 50.0, "taxi");
    SegmentoBusca* seg2 = newSegmentoBusca(1, 2, 50.0, "taxi");
    SegmentoBusca* seg3 = newSegmentoBusca(2, 3, 100.0, "andando");
    SegmentoBusca* seg4 = newSegmentoBusca(3, 4, 50.0, "onibus");
    SegmentoBusca* seg5 = newSegmentoBusca(4, 5, 50.0, "onibus");
    SegmentoBusca* seg6 = newSegmentoBusca(5, 6, 25.0, "metro");

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

    return 0; 
}