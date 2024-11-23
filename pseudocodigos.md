# Item 1

```
plantaCompletaND = Planta()
plantasND = vetor de tamanho |planta -> CEPs|

for i de 0 a |planta -> CEPs| - 1:
    plantaND = Planta()
    plantasND[i] = plantaND

for vértice v de 0 a |V| - 1:
    edges = (planta -> listaAdj)[v]
    for aresta e em edges:
        adiciona e a plantasND[e -> CEP] -> listaAdj
        adiciona e a plantaCompletaND -> listaAdj
        se e -> dupla == False:
            e' = cópia de e
            e' -> vSaida = e -> vEntrada
            e' -> vEntrada = e -> vSaida
            adiciona e' a plantasND[e -> CEP] -> listaAdj
            adiciona e' a plantaCompletaND -> listaAdj

train_stations = vetor de tamanho |planta -> CEPs|
j = 0

for plantaND em plantasND:
    max_distances = vetor de tamanho |V|
    for i de 0 a |V| - 1:
        max_distances = infinito
    for vértice v de 0 a |V| - 1:
        distances = vetor de tamanho |V|
        Dijkstra(v, distances) no subgrafo plantaND
        max_distance = 0
        for i de 0 a |V| - 1:
            if distances[i] > max_distance:
                max_distance = distances[i]
        max_distances[v] = max_distance
    min_max_distance = infinito
    min_max_distance_vertex = -1
    for i de 0 a |V| - 1:
        if max_distances[i] < min_max_distance:
            min_max_distance = max_distances[i]
            min_max_distance_vertex = i
    train_stations[j++] = min_max_distance_vertex

parents = vetor de tamanho |V|
MST(parents) no grafo plantaCompletaND


```