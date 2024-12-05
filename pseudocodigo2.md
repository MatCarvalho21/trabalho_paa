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
    set_aux_ceps = [set()] * num_vertices

    for i in range(num_vertices):
        lista_aux = planta.listaAdj[i]
        temp_node = lista_aux.head()

        while temp_node is not None:
            set_aux_ceps[temp_node.vSaida].add(temp_node.CEP)
            set_aux_ceps[temp_node.vEntrada].add(temp_node.CEP)

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

    for i in range(num_vertices):
        if len(set_aux_ceps[i]) > 1:
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

        if menor_distancia == float('inf') or vertice_atual == -1:
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
        if ververtice_otimo != -1:
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
    list
        Lista dos resultados de pais a partir da execução do Dijkstra para o vértice i,
        usada para reconstruir o caminho completo entre o vértice i e j das regiões.
    """
    numVertices = planta_regioes.num_vertices
    planta_virtual = Planta(numVertices)
    lista_anteriores = [None] * len(vertices_otimos)

    for vertice1 in vertices_otimos:
        distancias, anteriores = dijkstra_normal(planta_regioes, vertice1)
        lista_anteriores[vertice1] = anteriores
        for vertice2 in vertices_otimos:
            if vertice1 != vertice2:
                planta_virtual.adiciona_vertice(vertice1, vertice2, distancias[vertice2])

    return planta_virtual, lista_anteriores
```

Esta função é $O(V+E)$, já que, visa escolher um vértice inicial, percorrer suas arestas,
escolher a do vértice de menor valor, após isto, repetir o mesmo processo para o vértice
desta aresta, e fazer a escolha se considerar o vértice já adicionado ao ciclo, deste modo,
percorreremos cada vértice uma vez, e, para cada vértice percorreremos suas
arestas, resultando em uma complexidade $O(V+E)$ pois não repetiremos vértices nem arestas.

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
    ciclo = [vertice_inicial]
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

Esta função é $O(V^2)$, visto que, inicializaremos a matriz em $O(V^2)$,
assim, percorreremos todos os vértices uma vez e suas arestas uma vez, resultando
em complexidade $O(V+E)$ para passarmos os pesos das arestas para a matriz de
adjascência, resultando em uma complexidade final de $O(V^2)$, visto que,
para um grafo denso, $O(V+E) \approx O(V^2)$.

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

Esta função é $O(V)$, visto que, percorreremos o ciclo uma vez, e, como ele
contém todos os vértices, a operação se dará em $O(V)$, mas, como receberemos
um grafo em forma de matriz de adjascência, o acesso a aresta de $i$ para $j$
se dará em $O(1)$, resultando numa complexidade final de $O(V)$.

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

Esta função é $O(V^3)$, visto que, percorre todo o ciclo inicial em $O(V)$, e,
percorre seus sucessores do ciclo em $O(V)$ e, calcula o novo custo do ciclo
modificado, em $O(V)$, resultando em $O(V*(V*(V)))=O(V^3)$. Note que a operação
para inverter o ciclo também é dada em $O(V)$, mas não altera a complexidade,
visto que teríamos apenas $O(V*(V*(V + V)))=O(V^3)$.

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
        for i in range(n - 2):
            for j in range(i + 2, n):
                novo_ciclo = melhor_ciclo[:]

                temp = novo_ciclo[j]
                novo_ciclo[j] = novo_ciclo[i + 1]
                novo_ciclo[i + i] = temp
                novo_ciclo.append(novo_ciclo[0])
                novo_custo_ida, novo_custo_volta = calcular_custo_direcionado(grafo, novo_ciclo)
                novo_ciclo = novo_ciclo[:-1]

                volta = False
                if novo_custo_volta < novo_custo_ida:
                    volta = True
                
                if min(novo_custo_ida, novo_custo_volta) < melhor_custo:
                    melhor_custo = min(novo_custo_ida, novo_custo_volta)
                    melhor_ciclo = novo_ciclo[:]
                    melhorado = True
                    if volta:
                        melhor_ciclo = novo_ciclo[::-1]

    melhor_ciclo.append(melhor_ciclo[0])
    return melhor_ciclo, melhor_custo
```

Note que, esta função possui a maior complexidade das funções implementadas anteriormente,
sendo esta $O(R*V^3)$, visto que, no loop final, 

```python
limiar = 10
def bus(planta):
    planta_virtual, vertices_borda = construir_grafo_virtual(planta, limiar)

    vertices_regionais = acha_vertices_regionais(planta_virtual, vertices_borda)

    planta_regioes, lista_predecessores = construir_grafo_regioes(planta_virtual, vertices_regionais)

    ciclo_inicial = nearest_neighbor(planta_regioes, vertices_regionais[0])

    ciclo_novo, custo_ciclo = two_opt_directed(planta_regioes, ciclo_inicial)

    result = list()

    if len(ciclo_novo) < 3:
        return result

    for i in range(len(ciclo_novo) - 1):
        vertice_atual = ciclo_novo[i]
        vertice_proximo = ciclo_novo[i + 1]

        predecessores = lista_predecessores[vertice_atual]
        path = list()

        while vertice_proximo != -1:
            path.append(vertice_proximo)
            vertice_proximo = predecessores[vertice_proximo]

        for j in range(len(path) - 1, -1, -1):
            if path[j] != resultado[-1]:
                result.append(path[j])

    return result
```
