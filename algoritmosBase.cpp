/**
 * @file algoritmosBase.cpp
 * @brief Módulo para definição das funções dos algoritmos base de grafos.
 */

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <queue>
#include <tuple>
#include <climits>

#include "algoritmosBase.h"
#include "estrutura.h"

using std::cout;
using std::string;
using std::endl;
using std::vector;
using std::set;
using std::priority_queue;
using std::greater;
using std::tuple;
using std::make_tuple;
using std::get;

/// @brief Executa o algoritmo de Dijkstra em uma planta com vértice inicial definido.
/// @param v0 Vértice inicial
/// @param parents Vetor de pais.
/// @param distances Vetor de distâncias.
/// @param numVertices Número de vértices da planta
/// @param planta Planta a ser percorrida.
void dijkstra(int v0, vector<int>& parents, vector<int>& distances, int numVertices, Planta* planta)
{
    // Vetor de verificação que marca se um vértice já foi processado
    bool checked[numVertices];

    // Definição de uma fila de prioridade que armazena tuplas (distância, vértice),
    // onde a fila é organizada de acordo com a distância crescente
    priority_queue<tuple<int, int>, vector<tuple<int, int>>, greater<tuple<int, int>>> heap;

    // Inicializando os vetores parents, distances e checked
    for (int i = 0; i < numVertices; i++)
    {
        // Inicializando os pais como -1
        parents[i] = -1;
        // Inicializando as distâncias como infinito
        distances[i] = INT_MAX; 
        // Marcando todos os vértices como não verificados
        checked[i] = false; 
    }

    // Configurando o vértice de origem
    parents[v0] = v0;
    distances[v0] = 0;

    // Adicionando o vértice de origem à fila de prioridade
    heap.push(make_tuple(distances[v0], v0));

    // Enquanto o heap não estiver vazio...
    while (!heap.empty())
    {
        // Obtém o vértice com a menor distância da fila
        int v1 = get<1>(heap.top());
        // Obtém sua distância
        int currentDist = get<0>(heap.top());
        // Remove o vértice da fila
        heap.pop(); 

        // Se a distância do vértice extraído for maior que a distância conhecida, ignora
        if (currentDist > distances[v1])
        {
            continue;
        }

        // Se a distância do vértice for infinita, significa que não há mais vértices alcançáveis
        if (distances[v1] == INT_MAX)
        {
            break;
        }

        // Obtém a lista de arestas adjacentes ao vértice atual
        vector<Segmento*> edges = (planta -> listaAdj)[v1];

        // Para cada aresta de saída desse vértice...
        for (int i = 0; i < edges.size(); i++)
        {
            // Pega a aresta atual
            Segmento* edge = edges[i];

            // Pega o vértice de destino da aresta
            int v2 = edge -> vEntrada;

            // Se o vértice de destino ainda não foi processado...
            if (!checked[v2])
            {
                // Pega o custo da aresta
                int cost = edge -> tamanho; 

                // Se o caminho através do vértice v1 oferece uma distância menor para v2...
                if (distances[v1] + cost < distances[v2])
                {
                    // Atualiza o pai de v2
                    parents[v2] = v1; 
                    // Atualiza a distância de v2
                    distances[v2] = distances[v1] + cost; 
                    // Adiciona v2 à fila de prioridade
                    heap.push(make_tuple(distances[v2], v2)); 
                }
            }
        }

        // Marca o vértice v1 como processado
        checked[v1] = true;
    }
}
