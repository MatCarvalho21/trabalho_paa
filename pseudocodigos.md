# Item 1

A ideia desse algoritmo é:

- Criamos uma cópia da planta com todas as arestas e suas cópias na direção oposta (basicamente uma versão não direcionada do grafo). Isso é $O(V + E) = O(V)$ (nosso grafo é esparso).

```python
plantaND = Planta()

for v de 0 a |V| - 1:
    edges = planta -> listaAdj[v]
    for aresta edge em edges:
        Adiciona edge a plantaND -> listaAdj[v]
        if edge -> dupla == True:
            continue
        else:
            edge2 = newSegmento(vSaida = edge -> vEntrada, 
                                vEntrada = edge -> vSaida, 
                                limVel = edge -> limVel,
                                tamanho = edge -> tamanho,
                                CEP = edge -> CEP,
                                rua = edge -> rua,
                                dupla = True)
            adicionaSegmentoAPlanta(edge2, plantaND)
```

- Iteramos sobre todos os vértices salvando em uma matriz de {número de regiões} linhas e $|V|$ colunas se cada vértice possui uma aresta que sai ou entra dele que pertence àquela região, ou seja, que tenha aquele CEP. Isso é $O(E \cdot V + V + E) = O(VE) = O(V^2)$.

```python
regioes = vetor de tamanho |planta -> CEPs|
for i de 0 a |planta -> CEPs| - 1:
    regiao = vetor de tamanho |V|
    for j de 0 a |V| - 1:
        regiao[j] = False
    regioes[i] = regiao

for v de 0 a |V| - 1:
    edges = plantaND -> listaAdj[v]
    for aresta edge em edges:
        regioes[edge -> CEP][v] = True
```

- Para determinar os cruzamentos que receberão estações, para cada vértice v, executamos o algoritmo de Dijkstra começando em v, pegamos as regiões a que ele pertence por meio da matriz criada acima, procuramos, em cada uma dessas regiões, qual o vértice mais distante dele e, se essa distância for a menor para aquela região até agora, salvamos esse vértice como o resultado temporário para aquela região. Isso é $O(E \cdot V + V \cdot (V + (V + E) \cdot \log V + E \cdot V)) = O(V^3)$.

```python
min_max_distances, min_max_distances_vertices, min_max_distances_parents, min_max_distances_length = vetores de tamanho |planta -> CEPs|
for i de 0 a |planta -> CEPs| - 1:
    min_max_distances[i] = infinito
    min_max_distances_vertices[i] = -1
    temp_parents = vetor de tamanho v
    for j de 0 a tamanho |V|-1:
        temp_parents[j] = -1
    min_max_distances_parents[i] = temp_parents

    temp_distances = vetor de tamanho v
    for j de 0 a tamanho |V|-1:
        temp_distances[j] = INT_MAX
    min_max_distances_length[i] = temp_distances


for v de 0 a |V| - 1:
    parents, distances = vetores de tamanho |V|
    for i de 0 a |V| - 1:
        parents[i] = -1
        distances[i] = infinito

    Dijkstra(v, parents, distances, |V|)

    for regiao em regioes:
        if regiao[v] == True:
            max_distance = 0
            max_distance_vertice = -1
            for vértice v2 de 0 a |V| - 1:
                if regiao[v2] == True and distances[v2] > max_distance:
                    max_distance = distances[v2]
                    max_distance_vertice = v2
            if max_distance < min_max_distances[regiao]:
                min_max_distances[regiao] = max_distance
                min_max_distances_vertices[regiao] = v
                min_max_distances_parents[regiao] = parents
                min_max_distances_length[regiao] = distances
```

- Por fim, para encontrar uma aproximação da árvore mais barata que conecta esses vértices que receberão estações (o algoritmo para realmente solucionar esse problema é NP-difícil), pegamos os caminhos mais curtos entre cada par de vértices que possuem estações (esses caminhos já ficaram salvos acima na matriz `min_max_distances_parents`) e seus tamanhos, criamos uma planta virtual completa com arestas entre esses vértices e com os tamanhos sendo os tamanhos desses caminhos mais curtos, encontramos a MST dessa nova planta e pegamos as arestas utilizadas nos caminhos mais curtos usados na MST encontrada. Essas arestas são nossa resposta. Isso é $O(E \cdot E + E \cdot V + (V + E) \cdot \log V + E \cdot V) = O(V^2)$.

```python
plantaVirtual = Planta() // com o número de regiões

for i de 0 a |planta -> CEPs|-1:
    for j de 0 a |planta -> CEPs|:
        if i != j:
            v_saida = min_max_distances_vertices[i],
            v_entrada = min_max_distances_vertices[j]
            newSegmento(vSaida = i,
                        vEntrada = j,
                        tamanho = min_max_distances_length[i][v_entrada])

for v de 0 a |planta -> CEPs| - 1:
    parents = vetores de tamanho |V|
    for i de 0 a |V| - 1:
        parents[i] = -1

MST(parents, 0, plantaVirtual)

result = lista de arestas vazia

for i de 1 a |planta -> CEPs| - 1:
    virtual_parent = parents[i]
    real_parent = min_max_distances_vertices[virtual_parent]
    current_real_parents = min_max_distances_parents[virtual_parent]
    real_start = min_max_distances_vertices[i]

    while real_start != real_parent:
        Adiciona (current_real_parents[real_start], real_start) ao result
        real_start = current_real_parents[real_start]

return result
```

Portanto, esse algoritmo é $O(V^3)$.