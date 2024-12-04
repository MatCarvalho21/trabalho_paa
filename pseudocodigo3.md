# Questão 03

## Parâmetros
- Planta da Cidade (Planta);
- MST Metrô;
- Ciclo Ônibus;
- Vértice de origem (int);
- Vértice de destino (int);
- Valor Máximo (int);

## Retorno

- Vetor: um vetor contendo um par de vetores (vetor de segmentos, vetor de meios de transporte), total gasto e tempo necessário.

## Passo a Passo

Vamos receber a planta da cidade, mas vamos operar em um grafo virtual e para contruir ele vamos precisar adicionar arestas.

Primeiramente, para todo vértice conectado, vamos adicionar uma indo e uma voltando entre eles. Elas vão ser referentes ao trajeto andando (ele independe do segmento ser mão dupla ou não). Como peso, ela vai ter o tempo gasto percorrendo a pé (5km/h).

Logo em seguida, vou adicionar mais arestas ao grafo virtual, dessa vez referente ao metrô. Vou conectar diretamente as estações e os seus pesos vão ser o tempo para ir de uma estação para a outra.

Por fim, vou adicionar ainda mais arestas, agora entre todos os vértices que compõe a rota de ônibus. A ideia é criar um grafo completo ligando todas esses vértices entre si e o peso de cada uma dessas arestas, digamos A(i,j) é o tempo gasto de ônibus para ir da aresta i para a aresta j seguindo a rota e o sentido da linha de ônibus. Lembrando que i e j são arestas da linha de ônibus.

Com o grafo construido, teremos que criar um Dijkstra de certa maneira modificado, mas que vai conseguir identificar exatamente qual o melhor caminho.

A grande questão desse Dijkstra é que ele vai ir tendo o tempo como peso e quando chegar em uma aresta que seja ônibus ou metrô, ele vai ter que adicionar ao peso dessa aresta o tempo_acumulado (mod X) que é o tempo de espera para pegar esse determinado meio de transporte. Outra modificação é ir olhando sempre para o valor restante, vai existir verificações se é possível entrar no ônibus ou no metrô, caso não seja, essas arestas que falamos vão ter peso infinito e vão ser descosideradas pelo Dijkstra. De resto, é um Dijkstra comum que retorna uma lista de segmentos, o tempo e o valor gastos.

Com essa lista de segmentos, estrutura de dados que teremos que adaptar dos problemas anteriores, podemos encontrar a lista de meios de transporte usados.


```python
def dijkstra_metro(mst, origem):
    num_vertices = mst.listaAdj.size()
    distancias = [float('inf')] * num_vertices
    visitados = [False] * num_vertices
    anteriores = [None] * num_vertices

    distancias[origem] = 0

    while True:
        menor_distancia = float('inf')
        vertice_atual = -1

        for i in range(num_vertices):
            if not visitados[i] and distancias[i] < menor_distancia:
                menor_distancia = distancias[i]
                vertice_atual = i

        if menor_distancia == float('inf'):
            break

        adj_node = mst.listaAdj[vertice_atual].head()

        while adj_node is not None:
            vizinho = adj_node.vertex
            peso_aresta = adj_node.weight

            if not visitados[vizinho]:
                nova_distancia = distancias[vertice_atual] + peso_aresta
                if nova_distancia < distancias[vizinho]:
                    distancias[vizinho] = nova_distancia
                    anteriores[vizinho] = vertice_atual

            adj_node = adj_node.next()

        visitados[vertice_atual] = True

    return distancias, anteriores
```

```python
def constroi_arestas_metro(mst, lista_estacoes):
    lista_arestas = list()

    for estacao in lista_estacoes:
        distancias, anteriores = dijkstra(mst, estacao)
        for estacao2 in lista_estacoes:
            if estacao = estacao2:
                continue
            lista_arestas.append((estacao, estacao2, distancias[estacao2]))

    return lista_arestas
```

```python
def calcula_pesos_ciclo(grafo, ciclo, start):
    n = len(ciclo)
    distancias = [float("inf")] * n

    for i in range(n):
        if ciclo[i] == start:
            startIndex = i
            break

    distancias[startIndex] = 0
    end = startIndex - 1
    if end < 0:
        end = n - 1

    while start != end:
        listaAdj = grafo.listaAdj[start]

        nextIndex = (startIndex + 1) % n
        nextVertex = ciclo[nextIndex]
        distancias[nextIndex] = distancias[startIndex]

        for i in range(len(listaAdj)):
            if listaAdj[i].destino == nextVertex:
                peso = listaAdj[i].peso
                distancias[nextIndex] += peso
                break
        
        start = nextVertex
        startIndex = nextIndex

    return peso
```

```python
def constroi_arestas_onibus(grafo, ciclo):
    ordem_vertices = ciclo[:-1]

    for v1 in ordem_vertices:
        pesos = calcula_pesos_ciclo(grafo, ordem_vertices, v1)
        for v2 in ordem_vertices:
            if v1 != v2:
                peso = pesos[v2]
                grafo.adiciona_aresta(v1, v2, peso)

    return grafo
```



```python
velocidade = 5

def constroi_grafo_virtual(grafo, dinheiro, subway, bus):
    plantaVirtual = newPlanta(grafo.numVertices)

    for each_vertice in grafo.listAdj:
        for each_aresta in each_vertice:
            plantaVirtual.addSegmento(each_aresta.v1,
                                        each_aresta.v2,
                                        each_aresta.tamanho,
                                        each_aresta.tamanho/(1000*velocidade),
                                        "andando")
    
    for # adicionar as conexões da MST

    for each_segmento in bus.segmentos
        plantaVirtual.addSegmento(each_segmento.v1,
                                        each_segmento.v2,
                                        each_aresta.tamanho,
                                        each_aresta.tamanho/(1000*velLim*transito),
                                        "onibus")

    return plantaVirutal

def dijkstra_adapitado(plantaVirtual, origem, destino, dinheiro):
    dinheiro_atual = dinheiro

    if dinheiro > passagem_onibus:
        onibus = true
    else:
        onibus = false
    
    if dinheiro > passagem_metro:
        metro = true
    else:
        metro = false

    vertice_final = -1
    lista_de_vertices = []
    lista_de_parents = []
    lista_de_visitados = []
    lista_de_custo = []
    while vertice_final != destino:
        

```

``` 
    t = limite de tempo

    Heap tempHeap = newHeap()

    vetor<int> = times(num_vertices, MAX_INT)
    vetor<int> = parents(num_vertices, -1)

    times[vertice_inicial] = 0
    parents[vertice_inicial] = vertice_inicial
    temp_heap.add((0, vertice_inicial))

    while !tempHeap.empty(): 
        (dist, w) = tempHeap.topElement()
        tempHeap.removeTopElement()
        if w = vertice_inicial:
            break
        vector<Segmentos*> edges = w.listaAdj
        // each_aresta (w, x)
        for each_aresta in edges:
            newTime = times[w] + each_aresta.tempo
            if newTime < t and newTime < times[x]:
                times[x] = newTime
                parents[x] = w
                newDist = dist + each_aresta.size()
                heap.add(newDist, x)

    if parent[num_vertices] = -1
        return []
    
    vector<int> iPath; 
            
```

```
distancia_taxi = 0
dinheiro_restante = X
tempo_total = 0

taxa_fixa = 10
lim_metros = 2000
taxa_variavel = 0.002

calcula_pço_taxi(distancia_taxi)
    int dist_variavel = distancia_taxi - lim_metros;
    if dist_variavel <= 0:
        return 0 // pois já foi discontada na hora de descer para o nível 2
    else:
        return taxa_fixa + taxa_variavel*dist_variavel

se eu to no nível 2, ou seja, no grafo dos táxis 
na hora de consultar o custo para se mover para uma nova aresta vai ser dado por essa função (calcula_pço_taxi)
toda vez que eu percorrer uma aresta no grafo de baixo distancia_taxi deve ser atualizado
quando eu sair do nível 2, distancia_taxi deve ser zerado
quando ao que o dijstra vai olhar, vai ser o tempo que é um atributo de Segmento e vai ser somado somente no final do dijkstra, quando tivermos a sequência de segmentos
para ir do grafo2 custa a taxa fixa (quer dizer que entrei no taxi)
sair do grafo 2 não custa nada, significa que desci do táxi
no grafo de cima ainda vou ter a variável de custo, ela só vai ser subtraida quando entrar no ônibus ou no metrô (caso não seja suficiente, ignora aquele meio de transporte), ent tenho que verificar qual tipo de aresta estou percorrendo (a questão do tempo não importa, visto que ele tá administrando)
a ideia é retornar um vetor de segmentos, o tempo gasto e quanto custou 

def dijkstra_custo(grafo, lim_dinheiro)
    nivel_anterior = -1
    nivel_atual = 1
    distancia_taxi = 0
    vector<int> lista_segmentos;

    Heap tempHeap = newHeap()
    vetor<int> = cost(num_vertices, MAX_INT)
    vetor<int> = parents(num_vertices, -1)

    cost[vertice_inicial] = 0
    parents[vertice_inicial] = vertice_inicial
    temp_heap.add((0, vertice_inicial)) // 0 é o tempo gasto para chegar no vértice inicial

    while !tempHeap.empty():
        (tempo, vertice_atual) = tempHeap.topElement()
        tempHeap.removeTopElement()
        if vertice_atual = vertice_inicial:
            break
        nivel_atual = vertice_atual.nivel
        vector<Segmentos*> edges = vertice_atual.listaAdj

        // each_aresta (vertice_atual, cada_adjacente)
        for each_aresta in edges:
            custo_aux, dist_taxi = calcula_custo(vertice_atual, cada_adjacente, distancia_taxi)
            newCost = cost[vertice_atual] + custo_aux
            if newCost < lim_dinheiro and newCost < cost[cada_adjacente]:
                cost[cada_adjacente] = newCost
                parents[cada_adjacente] = vertice_atual
                newTime = tempo + each_aresta.custo() // nesse caso o custo é o tempo
                heap.add(newTime, cada_adjacente)

    // lista vazia
    if parents[vertice_inicial] = -1:
        return []

    iPath = []  # Lista vazia para armazenar o caminho

    vertice_atual = vertice_inicial

    # Reconstrói o caminho a partir do vetor de pais
    while vertice_atual != parents[vertice_atual]:
        iPath.append(vertice_atual)  # Adiciona o vértice ao caminho
        vertice_atual = parents[vertice_atual]  # Move para o próximo vértice no caminho

    path = []
    for i in ipath.size() a 1:
        adiciona ipath[i] a path
    
    return path 


    

def calcula_custo(vertice_atual, vertice_adjacente, dist_taxi):
    // entrei no metro cindo de outro meio de transporte
    if vertice_atual.MT != "metro" and vertice_adjacente.MT == "metro":
        return passagem_metro, dist_taxi
    
    // entrei no onibus vindo de outro meio de transporte
    if vertice_atual.MT != "onibus" and vertice_adjacente.MT == "onibus":
        return passagem_onibus, dist_taxi

    // vou do nível 1 para o nível 2
    if vertice_atual.MT != "taxi" and vertice_adjacente.MT = "taxi":
        return taxa_fixa_taxi, dist_taxi

    // percorrendo arestas no nível 2
    if vertice_atual.MT == "taxi" and vertice_adjacente.MT = "taxi":
        // vai retornar o custo e a nova dist_taxi
        return calcula_custo_taxi(vertice_atual, vertice_adjacente, dist_taxi)   

    return 0, dist_taxi  

def calcula_custo_taxi(vertice_atual, vertice_adjacente, dist_taxi):
    segmento = encontra_segmento(vertice_atual, vertice_adjacente)
    distancia = dist_taxi + segmento.tamanho
    custo = 0

    int dist_variavel = distancia - lim_metros;
    if dist_variavel <= 0:
        custo = 0
    else:
        custo =  taxa_variavel*dist_variavel

    return custo, distancia
```

