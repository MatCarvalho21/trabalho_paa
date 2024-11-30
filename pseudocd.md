# Questão 02:

## Cilco Entre as Regiões:
OK
Construir um grafo virtual com os nossos pesos, Peso = N + (Nº imóveis turístico e comercial) - (Nº imóveis residencial e industrial).

OK
Durante a construção, vamos salvar os vértices da borda do grafo. 

A fazer
Encontrar o vértice ótimo em cada região, minimiza a distância dele para os vértices da borda.  (Dijkstra de todos os vértices da região, fazendo uma média)

A fazer
Vou construir o grafo das regiões, conectando cada par de vértice ótimo. Formando um grafo completo e direcionado. 

OK
### Nearest Neighbor Heuristic - Vídeo:

A fazer
### 2-Opt ou 3-Opt:

```python

def calculaPeso(segmento)
    vetorImoveis = segmento.imoveis()
    comerciais = 0
    industriais = 0
    turisticos = 0
    residenciais = 0
    for i in vetorImoveis.size:
        swicth (vetorImoceis[i]):
            case 0:
                comerciais += 1
                break
            case 1: 
                industriais += 1
                break
            case 2:
                turisticos += 1
                break
            default:
                residenciais += 1
                break
    
    return (turisticos + comerciais) - (residenciais + industriais)
                

def grafo_virtual(planta, limiar):

    verticesBorda = set()
    numVertices = planta.listaAdj.size()
    plantaVirtual = Planta(numVertices)
    for i in numVertices:
        auxSet = set()
        listaAux = planta.listaAdj[i]
        tempNode = listAux.head()
        while tempNode != None:
            auxSet.add(tempNode.CEP)
            novoPeso = calculaPeso(tempNode)
            tempNode2 = newSegmento(tempNode.vSaida, 
                                    tempNode.vEntrada, 
                                    tempNode.limVel, 
                                    limiar + novoPeso, 
                                    tempNode.CEP, 
                                    tempNode.rua, 
                                    tempNode.dupla)
            adicionaSegmentoAPlanta(plantaVirtual, tempNode2)
            tempNode = tempNode.next()

        if auxSet.size > 1:
            verticesBorda.add(i)

    retorna plantaVirtual, verticesBorda
```

```python
def DijkstraRegional(planta, origem, cepRegiao):
    num_vertices = planta.listaAdj.size
    distancias = [float('inf')] * num_vertices
    visitados = [False] * num_vertices
    anteriores = [None] * num_vertices

    distancias[origem] = 0

    while True:
        menor_distancia = float('inf')
        vertice = -1

        for i in range(num_vertices):
            if not visitados[i] and distancias[i] < menor_distancia:
                menor_distancia = distancias[i]
                vertice = i
        
        if min_distancia == float('inf'):
            break

        adj_node = grafo.listaAdj[vertice_atual].head
        while adj_node != None:
            cepVizinho = adj_node.cep
            
            if cepVizinho != cepRegiao:
                adj_node = adj_node.next
                continue

            vertice_vizinho = adj_node.vertex
            peso_aresta = adj_node.weight

            if not visitados[vertice_vizinho]:
                nova_distancia = distancias[vertice_atual] + peso_aresta
                if nova_distancia < distancias[vertice_vizinho]:
                    distancias[vertice_vizinho] = nova_distancia
                    anteriores[vertice_vizinho] = vertice_atual

            adj_node = adj_node.next

        visitados[vertice_atual] = True

    return distancias, anteriores
```



```python
# vou supor que já existe a lista de vértices ideais
# dijkstra top vai receber uma plantaVirtual, uma plantaOrigianl, dois vértices e adicionar na planata virtual a conexão 
# entre os dois vértices, sendo o peso dessa conexão a menor distância entre os dois vértices no grafo/planta original
for each_vertice in verticesIdeais:
    for each_Vertice2 in verticesIdeais:
        dijkstraTop(plantaRegioes, plantaVirtual, each_vertice, each_vertice2):
        
def NearestNeighbor(plantaRegioes, verticeAtual = 0)
    num_vertices = plantaRegioes.listaAdj.size()

    ciclo = list()

    visitados = [False] * num_vertices
    visitados[verticeAtual] = True
    # O(V + E)
    while True:
        listaAdjAtual = plantaRegioes.listaAdj[verticeAtual]

        nextVertice = -1
        menorPeso = float("inf")

        current = listaAdjAtual.head()

        while current != None:
            if (not visitados[current.vSaida]) and (current.peso < menorPeso):
                menorPeso = current.peso
                nextVertice = current.vSaida
            
            current = current.next

        if nextVertice == -1:
            break
        
        ciclo.append(nextVertice)
        verticeAtual = nextVertice

    ciclo.append(ciclo[0])

    return ciclo
```








