# Item 1

```
plantaND = Planta()

para v de 0 a |V| - 1:
    edges = planta -> listaAdj[v]
    para cada aresta edge em edges:
        Adiciona edge a plantaND -> listaAdj[v]
        if edge -> dupla == True:
            continue
        else:
            edge' = newSegmento(vSaida = edge -> vEntrada, 
                                vEntrada = edge -> vSaida, 
                                limVel = edge -> limVel,
                                tamanho = edge -> tamanho,
                                CEP = edge -> CEP,
                                rua = edge -> rua,
                                dupla = True)
            imoveis = edge -> imoveis
            for i de imoveis.size() - 1 a 0:
                imovel = imoveis[i]
                imovel' = newImovel(dFinalSeg = edge -> tamanho - imovel -> dFinalSeg,
                                    num = imovel -> num, 
                                    tipo = imovel -> tipo)
                adicionaImovelASegmento(imovel', edge')
            adicionaSegmentoAPlanta(edge', plantaND)
            
regioes = vetor de tamanho |planta -> CEPs|
for i de 0 a |planta -> CEPs| - 1:
    regiao = vetor de tamanho |V|
    for j de 0 a |V| - 1:
        regiao[j] = False
    regioes[i] = regiao

para v de 0 a |V| - 1:
    edges = plantaND -> listaAdj[v]
    for aresta edge em edges:
        regioes[edge -> CEP][v] = True

min_max_distances, min_max_distances_vertices, min_max_distances_parents, min_max_distances_length = vetor de tamanho |planta -> CEPs|
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
            for vértice v' de 0 a |V| - 1:
                if regiao[v'] == True and distances[v'] > max_distance:
                    max_distance = distances[v']
                    max_distance_vertice = v'
            if max_distance < min_max_distances[regiao]:
                min_max_distances[regiao] = max_distance
                min_max_distances_vertices[regiao] = v
                min_max_distances_parents[regiao] = parents
                min_max_distances_length[regiao] = distances

plantaVirtual = Planta() // com o número de regiões

for i de 0 a |planta -> CEPs|-1:
    for j de 0 a |planta -> CEPs|:
        if i != j:
            v_saida = min_max_distances_vertices[i],
            v_entrada = min_max_distances_vertices[j]
            newSegmento(vSaida = i,
                        vEntrada = j,
                        tamanho = min_max_distances_length[i][v_entrada])

v0 = min_max_distances_vertices[0]

for v de 0 a |planta -> CEPs| - 1:
    parents = vetores de tamanho |V|
    for i de 0 a |V| - 1:
        parents[i] = -1

MST(parents, v0´, plantaVirtual)

result = lista de arestas vazia

// Começamos do 1 para não fazer iterações com v0, pois começamos dele.
for i de 1 a |planta -> CEPs| - 1:
    virtual_parent = parents[i]
    // Pego o pai da aresta virtual
    real_parent = min_max_distances_vertices[virtual_parent]
    // vetor de pais da real, no grafo não direcionado original
    current_real_parents = min_max_distances_parents[virtual_parent]
    // Vertice com ínidice original no grafo
    real_start = min_max_distances_vertices[i]

    while real_start != real_parent:
        Adiciona (current_real_parents[real_start], real_start) ao result
        real_start = current_real_parents[real_start]

return result
```