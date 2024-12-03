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

### Etapa 1 - Construção do Grafo Virtual

Esta função é $O(I)$, sendo $I$ o número de imóveis do segmento. Sua ideia é
calcular o peso de um segmento baseado em seus imóveis, onde adicionamos
peso caso não queiramos passar por ele (residencial e industrial), e subtraímos
peso caso queiramos passar por ele (comercial e ponto turístico).

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
```

Esta função é $O((V+E) * I)$, sendo $I$ o número máximo de imóveis em um segmento,
$V$ o número de vértices e $E$ o número de segmentos na planta. Sua ideia é
reconstruir o grafo com os novos pesos baseados no número de imóveis do segmento,
além de salvar os vértices que estão na borda entre duas regiões em uma lista.

```python
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
```

Esta função é $O(V^2)$, em resumo, aplicaremos o algoritmo de Dijkstra, $O(V^2)$, a partir
de uma origem a todos os vértices em uma região específica.

```python
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
```

Esta função é, em resumo, $O(V^3)$, pois aplicará o Dijkstra Regional $O(V^2)$,
a todos os vértices da região, resultando em $O(V^3)$ no pior caso em que uma
região possui todos os vértices, neste caso, buscamos achar o vértice que
minimiza a média das distâncias entre ele e os vértices da borda da região.

```python
def encontrar_vertice_otimo(planta, vertices_borda, cep_regiao):
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
    vertice_otimo = -1
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

    return vertice_otimo
```

Esta função é, na teoria, $O(R * V^3)$, pois encontrará o vértice ótimo em $O(V^3)$
para cada região, mas note que, se o número de regiôes se aproximar de $V$, então,
o tempo para encontrar o vértice ótimo será $O(V^2)$.
Neste caso, mesmo que na teoria possa existir o algoritmo em $O(V^4)$, o mesmo
nunca acontecerá, pois, quanto mais regiões, teremos menos números de execuções
de Dijkstra Regional, deste modo, $1 \le R < V$, e, Dijkstra
é inversamente proporcional à $R$.

```python
def acha_vertices_regionais(planta, vertices_borda):
    vertices_otimos = list()

    ceps = planta.CEPs

    for cep in ceps:
        vertice_otimo = encontrar_vertice_otimo(planta, vertices_borda, cep)
        vertices_otimos.append(vertice_otimo)

    return vertices_otimos
```

Esta função é $O(V^3)$, já que, no pior caso, o número de regiões se aproxima de
$V$, e, para cada nova região, devemos aplicar um Dijkstra, em O(V^2), para seu
vértice ótimo.

```python
def construir_grafo_regioes(planta_regioes, vertices_otimos):
    """
    Constrói um grafo virtual conectando vértices ideais de diferentes regiões.

    Parameters
    ----------
    planta_regioes : Planta
        O grafo original das regiões.
    vertices_otimos : list
        Lista de vértices ótimos identificados.

    Returns
    -------
    Planta
        Grafo virtual completo conectando os vértices ideais.
    """
    planta_virtual = Planta()

    for vertice1 in vertices_otimos:
        distancias = dijkstra_normal(planta_regioes, vertice1)

        for vertice2 in vertices_otimos:
            if vertice1 != vertice2:
                planta_virtual.adiciona_vertice(vertice1, vertice2, distancias[vertice2])

    return planta_virtual
```

```python
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
```

```python

def gerarMatrizAdjacencia(planta_regioes):
    """
    Cria uma matriz de adjascência a partir de uma lista de adjascência

    Parameters
    ----------
    planta_regioes : list
        Lista de adjacência representando os custos do grafo direcionado das regiões.

    Returns
    -------
    list
        Matriz de adjascência
    """
    num_vertices = planta_regioes.listaAdj.size()

    matriz = [[0] * num_vertices] * num_vertices

    for vertice_atual in range(num_vertices):
        adj_node = planta.listaAdj[vertice_atual].head()

        while adj_node is not None:
            vizinho = adj_node.vertex
            peso_aresta = adj_node.weight

            matriz[vertice_atual][vizinho] = peso_aresta

            adj_node = adj_node.next()

    return matriz
```

```python

def two_opt_directed(grafo, ciclo_inicial):
    """
    Otimiza um ciclo direcionado usando a técnica Two-Opt.

    Parameters
    ----------
    grafo : list
        Lista de adjacência representando os custos do grafo direcionado das regiões.
    ciclo_inicial : list
        Lista representando o ciclo inicial.

    Returns
    -------
    list
        Ciclo otimizado.
    int
        Custo total do ciclo otimizado.
    """
    n = len(ciclo_inicial) - 1
    matriz_adj = gerarMatrizAdjacencia(grafo)

    melhor_ciclo = ciclo_inicial[:-1]
    melhor_custo = min(calcular_custo_direcionado(matriz_adj, melhor_ciclo))
    melhorado = True

    if n < 4:
        return melhor_ciclo, melhor_custo

    while melhorado:
        melhorado = False
        for i in range(n - 4):
            for j in range(i + 2, n - 2):
                novo_ciclo = melhor_ciclo[:]

                temp = novo_ciclo[j]
                novo_ciclo[j] = novo_ciclo[i + 1]
                novo_ciclo[i + i] = temp

                novo_custo_ida, novo_custo_volta = calcular_custo_direcionado(grafo, novo_ciclo)
                volta = False
                if novo_custo_volta < novo_custo_ida:
                    volta = True
                
                if min(novo_custo_ida, novo_custo_volta) < melhor_custo:
                    melhor_custo = min(novo_custo_ida, novo_custo_volta)
                    melhor_ciclo = novo_ciclo[:]
                    melhorado = True
                    if volta:
                        melhor_ciclo = novo_ciclo[::-1]

    return melhor_ciclo, melhor_custo
```

```python
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
    tuple[int, int]
        Custo total do ciclo na ordem de ida e ordem de volta.
    """
    custo_ida = 0
    custo_volta = 0
    n_vertices = len(ciclo)

    for i in range(n_vertices - 2):
        custo_ida += grafo[ciclo[i]][ciclo[i+1]]

        j = n_vertices -1 - i
        custo_volta += grafo[ciclo[j]][ciclo[j-1]]

    custo_ida += grafo[ciclo[-1]][ciclo[0]]
    custo_volta += grafo[ciclo[0]][ciclo[-1]]

    return custo_ida, custo_volta
```

---
# Questão 03

Pessoa vai inserir Origem, Destino e Dinheiro

vamos rodas o dijkstra normal e encontrar o menor caminho a pé (grafo normal) (sequência de segmentos e tempo gasto)

criar grafo virtual sendo o peso dos segmentos o tempo = comprimento/(vel_max * transito) (sequência de segmentos e tempo gasto)

vamos rodar dijkstra pro táxi nesse novo grafo virtual 

encontrar a estação de metrô e o ponto do ônibus mais próximo da origem (dijsktra multi-source)
encontrar a estação de metrô e o ponto de ônibus mais próximo do destino a partir da estação de origem (dijkstra multi-source)

dijsktra estação de metrô inicial e estação de metrô final (sequência de segmentos e tempo gasto)
dijskstra estação de ônibus inicial e estação de metrô final (sequência de segmentos e tempo gasto)



Grafo virtual com linha de metrô (ligação A e B) e o ciclo do ônibus (KNN arestas paralelas)



Temos que retornar preço, tempo gasto, pair(sequência de segmentos e métodos de transporte).

Funções Auxiliares:

Remodelar o mapa

Dijkstra da Origem até o metrô mais próximo: Caminho andando e de táxi
Dijkstra da Origem até o ônibus mais próximo: Caminho andando e de táxi
Dijkstra Multi-source das estações de metrô até o destino: Caminho andando e de táxi
Dijkstra Multi-source das paradas de ônibus até o destino: Caminho andando e de táxi

Executamos dijkstra até o destino andando e de táxi 

A - A - A
T - T - T (A)
A - M - A
A - M - T
A - O - A
A - O - T
T - M - A
T - M - T
T - O - A
T - O - T
A/T - O - M - A/T


```python

def ex03(origem, destino, valor = inf):
    Dijkstra_Origem_Final() # me retorna a rota, tempo e o custo gasto pra ir andando e de táxi até o final
    Dijkstra_MS_Origem_Metro()
    Dijkstra_MS_Origem_Onibus()
    Dijkstra_MS_Metro_Destino()
    Dijkstra_MS_Onibus_Destino()
    Dijkstra_Metro_Metro()
    Dijkstra_Onibus_Onibus()
```











<!-- 
---
# Rascunho
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

--- -->
