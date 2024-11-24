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

min_max_distances, min_max_distances_vertices = vetor de tamanho |planta -> CEPs|
for i de 0 a |planta -> CEPs| - 1:
    min_max_distances[i] = infinito
    min_max_distances_vertices[i] = -1

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

v0 = min_max_distances_vertices[0]

parents = vetor de tamanho |V|
    for i de 0 a |V| - 1:
        parents[i] = -1

MST(parents, v0)

result = lista de arestas vazia

for i de 1 a |planta -> CEPs| - 1:
    v1 = min_max_distances_vertices[i]
    
    while v1 != v0:
        Adiciona (parents[v1], v1) a result
        v1 = parents[v1]

return result
```