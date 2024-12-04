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
def dijkstra(mst, origem):
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
    if end == -1:
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