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
        distance = dijkstraTop(plantaRegioes, each_vertice, each_vertice2):
        plantaVirtual.addVertex(each_vertice, each_vertice2, distance)
        
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




```python
def two_opt_directed(graph, initial_cycle):
    """
    graph: matriz de adjacência do grafo direcionado (custos das arestas)
    initial_cycle: lista de vértices representando o ciclo inicial
    Retorna: ciclo otimizado e o custo associado
    """
    n = len(initial_cycle)
    best_cycle = initial_cycle[:]
    best_cost = calculate_cost_directed(graph, best_cycle)
    improved = True

    while improved:
        improved = False
        for i in range(n - 1):
            for j in range(i + 2, n):  # Garantir que não há sobreposição
                # Troca duas arestas, mantendo a direção
                new_cycle = best_cycle[:i+1] + best_cycle[i+1:j+1][::-1] + best_cycle[j+1:]
                new_cost = calculate_cost_directed(graph, new_cycle)
                
                if new_cost < best_cost:
                    best_cycle = new_cycle
                    best_cost = new_cost
                    improved = True

    return best_cycle, best_cost


def calculate_cost_directed(graph, cycle):
    """
    Calcula o custo total de um ciclo no grafo direcionado.
    """
    cost = 0
    for i in range(len(cycle) - 1):
        cost += graph[cycle[i]][cycle[i+1]]
    cost += graph[cycle[-1]][cycle[0]]  # Retorno ao vértice inicial
    return cost
```

---

# Questão 02

## Ciclo Entre as Regiões

Este código tem como objetivo calcular o ciclo ótimo entre regiões de um grafo, considerando pesos calculados com base nos tipos de imóveis e otimizando o percurso utilizando heurísticas como Nearest Neighbor e 2-Opt.

---

## Passos Principais

1. **Construção do Grafo Virtual**:
   - Cada vértice recebe um peso calculado com base no número de imóveis turísticos, comerciais, residenciais e industriais.
   - Vértices da borda são identificados e armazenados para processamento futuro.

2. **Cálculo do Vértice Ótimo por Região**:
   - Para cada região, encontrar o vértice que minimiza a distância média até os vértices da borda, utilizando Dijkstra.

3. **Construção do Grafo das Regiões**:
   - Conectar os vértices ótimos de cada região, formando um grafo completo e direcionado, com pesos correspondentes às menores distâncias.

4. **Heurísticas para Otimização do Ciclo**:
   - Implementação de Nearest Neighbor para encontrar um ciclo inicial.
   - Aplicação da técnica 2-Opt para otimizar o ciclo obtido.

---

## Funções e Pseudocódigo

```python
def calcula_peso(segmento):
    """
    Calcula o peso de um segmento com base no tipo de imóveis.

    Parameters
    ----------
    segmento : Segmento
        Objeto que contém informações sobre os imóveis associados ao segmento.

    Returns
    -------
    int
        Peso do segmento calculado como (turísticos + comerciais) - (residenciais + industriais).
    """
    vetor_imoveis = segmento.imoveis()
    comerciais = 0
    industriais = 0
    turisticos = 0
    residenciais = 0

    for imovel in vetor_imoveis:
        if imovel == 0:
            comerciais += 1
        elif imovel == 1:
            industriais += 1
        elif imovel == 2:
            turisticos += 1
        else:
            residenciais += 1

    return (turisticos + comerciais) - (residenciais + industriais)

def construir_grafo_virtual(planta, limiar):
    """
    Constrói um grafo virtual com pesos ajustados e identifica vértices de borda.

    Parameters
    ----------
    planta : Planta
        O grafo original da planta.
    limiar : int
        Limiar base usado para ajustar os pesos dos segmentos.

    Returns
    -------
    Planta
        O grafo virtual criado.
    set
        Conjunto de vértices de borda.
    """
    vertices_borda = set()
    num_vertices = planta.listaAdj.size()
    planta_virtual = Planta(num_vertices)

    for i in range(num_vertices):
        aux_set = set()
        lista_aux = planta.listaAdj[i]
        temp_node = lista_aux.head()

        while temp_node is not None:
            aux_set.add(temp_node.CEP)
            novo_peso = calcula_peso(temp_node)
            temp_segmento = newSegmento(
                temp_node.vSaida,
                temp_node.vEntrada,
                temp_node.limVel,
                limiar + novo_peso,
                temp_node.CEP,
                temp_node.rua,
                temp_node.dupla
            )
            planta_virtual.adiciona_segmento(temp_segmento)
            temp_node = temp_node.next()

        if len(aux_set) > 1:
            vertices_borda.add(i)

    return planta_virtual, vertices_borda

def dijkstra_regional(planta, origem, cep_regiao):
    """
    Aplica o algoritmo de Dijkstra para calcular distâncias dentro de uma região específica.

    Parameters
    ----------
    planta : Planta
        O grafo representando a planta da região.
    origem : int
        Vértice inicial para calcular as distâncias.
    cep_regiao : int
        Identificador da região para restringir os cálculos.

    Returns
    -------
    list
        Distâncias mínimas do vértice de origem a todos os outros vértices na região.
    list
        Lista de predecessores para reconstrução do caminho mínimo.
    """
    num_vertices = planta.listaAdj.size()
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

        adj_node = planta.listaAdj[vertice_atual].head()

        while adj_node is not None:
            if adj_node.cep != cep_regiao:
                adj_node = adj_node.next()
                continue

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

def encontrar_vertices_otimos(planta, vertices_borda, cep_regiao):
    """
    Encontra os vértices ótimos para cada região, minimizando a média das distâncias às bordas.

    Parameters
    ----------
    planta : Planta
        O grafo representando a planta da região.
    vertices_borda : set
        Conjunto de vértices de borda.
    cep_regiao : int
        Identificador da região.

    Returns
    -------
    list
        Lista de vértices ótimos.
    """
    num_vertices = planta.listaAdj.size()
    vertices_otimos = []
    menor_media_distancias = float('inf')

    for vertice in range(num_vertices):
        distancias, _ = dijkstra_regional(planta, vertice, cep_regiao)

        distancias_borda = [distancias[borda] for borda in vertices_borda if distancias[borda] != float('inf')]

        if not distancias_borda:
            continue 

        media_distancias = sum(distancias_borda) / len(distancias_borda)

        if media_distancias < menor_media_distancias:
            menor_media_distancias = media_distancias
            vertice_otimo = vertice

    vertices_otimos.append(vertice_otimo)
    return vertices_otimos

def construir_grafo_regioes(planta_regioes, vertices_ideais):
    """
    Constrói um grafo virtual conectando vértices ideais de diferentes regiões.

    Parameters
    ----------
    planta_regioes : Planta
        O grafo original das regiões.
    vertices_ideais : list
        Lista de vértices ótimos identificados.

    Returns
    -------
    Planta
        Grafo virtual completo conectando os vértices ideais.
    """
    planta_virtual = Planta()

    for vertice1 in vertices_ideais:
        for vertice2 in vertices_ideais:
            if vertice1 != vertice2:
                distancia = dijkstra_top(planta_regioes, vertice1, vertice2)
                planta_virtual.adiciona_vertice(vertice1, vertice2, distancia)

    return planta_virtual

def nearest_neighbor(planta_regioes, vertice_inicial=0):
    """
    Aplica a heurística do vizinho mais próximo para encontrar um ciclo no grafo.

    Parameters
    ----------
    planta_regioes : Planta
        O grafo representando as regiões.
    vertice_inicial : int, optional
        Vértice de início do ciclo. O padrão é 0.

    Returns
    -------
    list
        Ciclo encontrado que passa por todos os vértices e retorna ao inicial.
    """
    num_vertices = planta_regioes.listaAdj.size()
    ciclo = []
    visitados = [False] * num_vertices

    vertice_atual = vertice_inicial
    visitados[vertice_atual] = True

    while True:
        lista_adj_atual = planta_regioes.listaAdj[vertice_atual]
        menor_peso = float("inf")
        proximo_vertice = -1

        current = lista_adj_atual.head()
        while current is not None:
            if not visitados[current.vSaida] and current.peso < menor_peso:
                menor_peso = current.peso
                proximo_vertice = current.vSaida
            current = current.next()

        if proximo_vertice == -1:
            break

        ciclo.append(proximo_vertice)
        vertice_atual = proximo_vertice
        visitados[vertice_atual] = True

    ciclo.append(ciclo[0])
    return ciclo

def two_opt_directed(grafo, ciclo_inicial):
    """
    Otimiza um ciclo direcionado usando a técnica Two-Opt.

    Parameters
    ----------
    grafo : list
        Matriz de adjacência representando os custos do grafo direcionado.
    ciclo_inicial : list
        Lista representando o ciclo inicial.

    Returns
    -------
    list
        Ciclo otimizado.
    int
        Custo total do ciclo otimizado.
    """
    n = len(ciclo_inicial)
    melhor_ciclo = ciclo_inicial[:]
    melhor_custo = calcular_custo_direcionado(grafo, melhor_ciclo)
    melhorado = True

    while melhorado:
        melhorado = False
        for i in range(n - 1):
            for j in range(i + 2, n): 
                novo_ciclo = (
                    melhor_ciclo[:i+1] + 
                    melhor_ciclo[i+1:j+1][::-1] + 
                    melhor_ciclo[j+1:]
                )
                novo_custo = calcular_custo_direcionado(grafo, novo_ciclo)

                if novo_custo < melhor_custo:
                    melhor_ciclo = novo_ciclo
                    melhor_custo = novo_custo
                    melhorado = True

    return melhor_ciclo, melhor_custo


def calcular_custo_direcionado(grafo, ciclo):
        """
    Calcula o custo total de um ciclo em um grafo direcionado.

    Parameters
    ----------
    grafo : list
        Matriz de adjacência representando os custos do grafo.
    ciclo : list
        Lista de vértices representando o ciclo.

    Returns
    -------
    int
        Custo total do ciclo.
    """
    custo = 0
    for i in range(len(ciclo) - 1):
        custo += grafo[ciclo[i]][ciclo[i+1]]
    custo += grafo[ciclo[-1]][ciclo[0]] 
    return custo

```

