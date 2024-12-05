## Questão 03

### Passo a Passo 

A ideia é criar grafos sobrepostos, com base no grafo original da cidade. Esse grafo contaria com arestas paralelas (cada uma representando em meio de transporte) além de uma projeção vertical espelhada que representaria o percurso de taxi. 

Inicialmente, precisamos criar esse grafo virutal, para isso temos uma série de funções: `dijkstraMetro`, `achaArestasMetro`, `calculaDistTempoCiclo`, `achaArestasOnibus` e `constroiPlantaBusca`. O funcionamento de todas elas e as suas complexidades serão explicados abaixo. Além disso, o código dessas funções conta com uma série de comentários e descrições que serão muito úteis.

Posteriormente, vamso ter que defato colocar o algoritmo em prática e resolver o problema apresentado na tarefa 3. Para isso vamos usar a função `dijkstra_custo` que conta com duas funções auxiliares `calcula_custo` e `calcula_custo_taxi`. Como dito anteriormente, todas serão devidamente explicadas abaixo e contam com comentários ao longo de suas implementações. 

### Pseudocódigos

##### dijkstraMetro()

A função `dijkstraMetro` implementa o algoritmo de Dijkstra para encontrar as menores distâncias entre um vértice de origem e todos os outros vértices em um grafo de metrô. Ela inicializa os vetores de distâncias e predecessores, e utiliza uma fila de prioridade (min-heap) para processar os vértices de acordo com a menor distância. Para cada vértice, o algoritmo verifica seus vizinhos e atualiza a distância se um caminho mais curto for encontrado. Ao final, a função retorna as menores distâncias e os predecessores de cada vértice, permitindo reconstruir o caminho mais curto. A complexidade do algoritmo é \(O((V + E) \log V)\), onde \(V\) é o número de vértices e \(E\) o número de arestas, o que o torna eficiente para grafos esparsos como são as cidades.

```
Função dijkstraMetro(linha_metro, origem):
    Parâmetros: linha_metro (grafo representando o sistema de metrô), origem (índice do vértice de origem)
    Retorno: distâncias (vetor com a distância mínima de origem para todos os vértices),
            parents (vetor com os predecessores de cada vértice)

    Inicializar:
        numVertices = tamanho da lista de adjacências de linha_metro
        distâncias = vetor de tamanho numVertices, inicializado com INT_MAX (representando infinito)
        distâncias[origem] = 0  // Distância para a origem é 0
        parents = vetor de tamanho numVertices, inicializado com -1 (sem predecessor)
        
        filaPrioridade = fila de prioridade (min-heap) para gerenciar os vértices a serem explorados
        filaPrioridade.push((0, origem))  // Coloca a origem na fila de prioridade com distância 0

    Enquanto filaPrioridade não estiver vazia:
        topo = filaPrioridade.top()  // Pega o vértice com a menor distância
        distAtual = topo.first  // Distância do vértice atual
        verticeAtual = topo.second  // Índice do vértice atual

        filaPrioridade.pop()  // Remove o vértice da fila

        Se distAtual > distâncias[verticeAtual]:
            Continuar para o próximo topo (vértice) na fila

        // Explorar todos os vizinhos do vértice atual
        Para cada segmento em linha_metro.listaAdj[verticeAtual]:
            vizinho = segmento.vEntrada  // Obter o vértice de saída do segmento
            peso = segmento.tamanho  // Peso da aresta (distância entre os vértices)

            Se distâncias[verticeAtual] + peso < distâncias[vizinho]:
                distâncias[vizinho] = distâncias[verticeAtual] + peso  // Atualizar a distância do vizinho
                parents[vizinho] = verticeAtual  // Atualizar o predecessor do vizinho
                filaPrioridade.push((distâncias[vizinho], vizinho))  // Coloca o vizinho na fila com a nova distância

    Retornar (distâncias, parents)  // Retorna o vetor de distâncias e o vetor de predecessores
```

##### achaArestasMetro()

A função `achaArestasMetro` encontra as arestas entre as estações de metrô, utilizando o algoritmo de Dijkstra para calcular as distâncias mínimas entre as estações. Para cada estação na lista `estacoesMetro`, a função chama `dijkstraMetro` para obter as distâncias mínimas a partir dessa estação. Em seguida, a função percorre todas as outras estações e, se a estação não for a mesma, calcula o peso da aresta como a distância dividida por 1000 (para converter de metros para quilômetros). As arestas e seus respectivos pesos são armazenados em um vetor, que é retornado ao final. A complexidade do algoritmo é \(O(V^2 \log V + V^2 E)\), onde \(V\) é o número de vértices e \(E\) o número de arestas.


```
Função achaArestasMetro(mstMetro, estacoesMetro):
    Parâmetros: mstMetro (grafo representando o sistema de metrô), estacoesMetro (lista de vértices de estações de metrô)
    Retorno: arestasMetro (vetor de arestas entre as estações de metrô com os respectivos pesos)

    Inicializar:
        arestasMetro = lista vazia para armazenar as arestas

    Para cada i de 0 até o tamanho de estacoesMetro - 1:
        Chamar a função dijkstraMetro passando mstMetro e estacoesMetro[i] como parâmetros
        resultado = dijkstraMetro(mstMetro, estacoesMetro[i])
        distancias = resultado.first  // Vetor de distâncias de origem estacoesMetro[i] a todos os outros vértices
        predecessores = resultado.second  // Vetor de predecessores de cada vértice
        
        Para cada j de 0 até o tamanho de estacoesMetro - 1:
            Se i for igual a j: Continuar (não adicionar a aresta para a mesma estação)
            
            // Calcular o peso da aresta entre estacoesMetro[i] e estacoesMetro[j]
            peso = distancias[estacoesMetro[j]] / 1000.0  // Convertendo para quilômetros
            
            // Adicionar a aresta e seu peso ao vetor arestasMetro
            arestasMetro.push_back({{estacoesMetro[i], estacoesMetro[j]}, peso})

    Retornar arestasMetro  // Retorna o vetor com todas as arestas e seus respectivos pesos
```

##### calculaDistTempoCiclo()

A função `calculaDistTempoCiclo` calcula as distâncias e tempos necessários para percorrer um ciclo de transporte dentro de uma planta. A função recebe a planta, um vetor de vértices representando o ciclo e um vértice inicial `start`. A partir desse vértice inicial, a função percorre os segmentos da planta, calculando a distância total em quilômetros e o tempo necessário para cada segmento do ciclo. As distâncias e tempos são armazenados em um vetor de pares e retornados ao final. A complexidade dessa função é \(O(n \cdot m)\), onde \(n\) é o número de vértices no ciclo e \(m\) o número de segmentos adjacentes.


```
Função calculaDistTempoCiclo(planta, ciclo, start):
    Parâmetros: 
        planta (grafo representando a planta com segmentos de transporte),
        ciclo (vetor de vértices que formam o ciclo de transporte),
        start (vértice de início no ciclo)
    Retorno: 
        distanciasTempos (vetor com distâncias e tempos para cada vértice no ciclo)

    Inicializar:
        n = tamanho do vetor ciclo  // Número de vértices no ciclo
        distanciasTempos = lista de tamanho n, inicializada com valores (0, 0)  // Distâncias e tempos
        startIndex = -1

    Para i de 0 até n - 1:
        Se ciclo[i] for igual a start:
            startIndex = i  // Encontrar o índice do vértice de início
            Parar o loop

    distanciasTempos[startIndex] = (0, 0)  // Inicializar a distância e tempo do vértice de início

    endIndex = startIndex - 1  // O vértice final será o anterior ao índice de início
    Se endIndex < 0:
        endIndex = n - 1  // Caso o índice final seja negativo, ajustar para o último vértice do ciclo

    Enquanto startIndex não for igual a endIndex:
        Obter os segmentos adjacentes ao vértice ciclo[startIndex] da planta
        nextIndex = (startIndex + 1) % n  // Calcular o índice do próximo vértice no ciclo (circular)

        distanciasTempos[nextIndex] = distanciasTempos[startIndex]  // Copiar as distâncias e tempos atuais para o próximo índice

        Para cada segmento em segmentos:
            Se segmento.vEntrada for igual a ciclo[nextIndex]:
                distanciaKM = segmento.tamanho / 1000.0  // Converter tamanho do segmento para quilômetros
                distanciasTempos[nextIndex].first += distanciaKM  // Atualizar a distância
                distanciasTempos[nextIndex].second += (distanciaKM / segmento.limVel)  // Atualizar o tempo com base na velocidade do segmento
                Parar o loop

    Retornar distanciasTempos  // Retornar o vetor com as distâncias e tempos para cada vértice no ciclo
```

##### achaArestasOnibus()

A função `achaArestasOnibus` calcula as arestas que conectam as estações de ônibus em um ciclo de transporte dentro de uma planta. Ela recebe a planta e um vetor de inteiros representando o ciclo de ônibus. Para cada estação do ciclo (exceto a última), a função chama `calculaDistTempoCiclo` para calcular as distâncias e tempos de viagem entre as estações, e armazena essas informações como arestas. Cada aresta é representada por um par de pares, onde o primeiro par contém os vértices conectados e o segundo contém a distância e o tempo de viagem entre eles. A complexidade dessa função é \(O(n^2)\), onde \(n\) é o número de vértices no ciclo de ônibus.


```
Função achaArestasOnibus(planta, cicloBus):
    Parâmetros: 
        planta (grafo representando a planta com segmentos de transporte),
        cicloBus (vetor de vértices que formam o ciclo do ônibus)
    Retorno:
        arestasOnibus (vetor de pares que representam as arestas e suas distâncias/tempos)

    Inicializar:
        cicloTemp = cicloBus
        Remover o último elemento de cicloTemp  // Porque o ciclo é fechado, o último vértice não precisa ser considerado aqui

        arestasOnibus = lista vazia  // Para armazenar as arestas do ciclo de ônibus

    Para cada i de 0 até tamanho de cicloTemp - 1:
        distanciasTempos = calculaDistTempoCiclo(planta, cicloTemp, cicloTemp[i])  // Calcular as distâncias e tempos para o ciclo a partir do vértice cicloTemp[i]

        Para cada j de 0 até tamanho de cicloTemp - 1:
            Se i for igual a j: 
                Continuar para o próximo j (evitar adicionar aresta de um vértice para ele mesmo)

            // Adicionar aresta entre cicloTemp[i] e cicloTemp[j] com a distância e tempo calculados
            arestasOnibus.push_back({{cicloTemp[i], cicloTemp[j]}, {distanciasTempos[j].first, distanciasTempos[j].second}})

    Retornar arestasOnibus  // Retornar as arestas do ciclo de ônibus com distâncias e tempos
```

##### constroiPlantaBusca()

A função `constroiPlantaBusca` constrói um grafo expandido a partir de uma planta de transporte, adicionando segmentos de diferentes tipos de transporte, como "andar", "taxi", "metro" e "onibus". A função recebe como parâmetros a planta original, um ciclo de ônibus e um grafo de metrô. 

Primeiramente, a função cria uma nova instância de `PlantaBusca`, que possui o dobro de vértices da planta original (uma duplicação para cada tipo de transporte). Para cada segmento da planta original, ela cria dois tipos de segmentos: um para o transporte a pé ("andar") e outro para o transporte de táxi ("taxi"). Cada segmento de táxi e a pé é adicionado ao grafo expandido, com o cálculo da distância em quilômetros e o tempo necessário para percorrê-lo. Além disso, são adicionados segmentos de conexão entre os vértices duplicados para permitir a troca entre os diferentes meios de transporte.

Em seguida, são calculadas as arestas do metrô e do ônibus. Para o metrô, são adicionadas arestas entre as estações, considerando a distância e o tempo de viagem, enquanto para o ônibus, são calculadas as distâncias e tempos de cada ciclo de ônibus, criando arestas entre as estações de ônibus correspondentes. Todos esses segmentos são adicionados ao grafo expandido.

O grafo retornado é um grafo dirigido que permite representar a troca entre diferentes meios de transporte, incluindo andar a pé, táxi, metrô e ônibus. Cada aresta possui informações sobre a distância e o tempo de viagem, além do tipo de transporte associado.

A complexidade dessa função é \(O(n + m)\), onde \(n\) é o número de vértices na planta e \(m\) é o número de segmentos (arestas) presentes.

```
Função constroiPlantaBusca(planta, cicloBus, mstMetro, estacoesMetro):
    Parâmetros:
        planta (grafo representando os segmentos de transporte),
        cicloBus (vetor de vértices que formam o ciclo do ônibus),
        mstMetro (grafo representando o metro),
        estacoesMetro (lista de estações de metrô)
    Retorno:
        plantaBusca (estrutura que contém todos os segmentos transformados em "busca")

    Inicializar:
        nVertices = número de vértices em planta (tamanho de listaAdj)
        plantaBusca = novo grafo (com 2*nVertices vértices) // Representa uma planta com transporte a pé e de táxi

    Para cada i de 0 até nVertices - 1:
        segmentos = planta.listaAdj[i]
        
        Para cada segmento em segmentos:
            // Convertendo para distâncias em quilômetros
            segmentoKm = segmento.tamanho / 1000.0

            // Criar segmentos de "andar" (a pé)
            segmentoAndar = novoSegmentoBusca(i, segmento.vEntrada, segmentoKm, segmentoKm / VelocidadeAndar, "andar")
            plantaBusca.adicionaSegmento(segmentoAndar)

            // Criar segmentos de "taxi"
            segmentoTaxi = novoSegmentoBusca(i + nVertices, segmento.vEntrada + nVertices, segmentoKm, segmentoKm / segmento.limVel, "taxi")
            plantaBusca.adicionaSegmento(segmentoTaxi)

            // Criar conexões entre "andar" e "taxi" (ida e volta)
            segmentoConexaoIda = novoSegmentoBusca(i, i + nVertices, 0, 0, "andar")
            segmentoConexaoVolta = novoSegmentoBusca(i + nVertices, i, 0, 0, "taxi")
            segmentoConexaoIda.vertical = verdadeiro
            segmentoConexaoVolta.vertical = verdadeiro

            plantaBusca.adicionaSegmento(segmentoConexaoIda)
            plantaBusca.adicionaSegmento(segmentoConexaoVolta)

            Se segmento.dupla:
                Continuar para o próximo segmento
            Senão:
                // Criar o segmento reverso para "andar"
                segmentoAndar2 = novoSegmentoBusca(segmento.vEntrada, i, segmentoKm, segmentoKm / VelocidadeAndar, "andar")
                plantaBusca.adicionaSegmento(segmentoAndar2)

    // Adicionar segmentos de metrô
    arestasMetro = chama a função achaArestasMetro(mstMetro, estacoesMetro)
    Para cada arestaMetro em arestasMetro:
        aresta = arestaMetro.first
        distancia = arestaMetro.second

        segmentoMetro = novoSegmentoBusca(aresta.first, aresta.second, distancia, distancia / VelocidadeMetro, "metro")
        plantaBusca.adicionaSegmento(segmentoMetro)

    // Adicionar segmentos de ônibus
    arestasOnibus = chama a função achaArestasOnibus(planta, cicloBus)
    Para cada arestaOnibus em arestasOnibus:
        aresta = arestaOnibus.first
        distTempo = arestaOnibus.second

        segmentoOnibus = novoSegmentoBusca(aresta.first, aresta.second, distTempo.first, distTempo.second, "onibus")
        plantaBusca.adicionaSegmento(segmentoOnibus)

    Retornar plantaBusca  // Retorna a planta construída com todos os segmentos
```

##### calcula_custo_taxi()

A função `calcula_custo_taxi` possui complexidade constante, ou seja, \(O(1)\). Isso ocorre porque a função realiza uma série de operações simples, como soma e comparação de valores, independentemente do tamanho do grafo ou do número de segmentos. Ela calcula o custo do trajeto com base na distância acumulada e no segmento adjacente, sem iterar sobre estruturas de dados mais complexas, o que resulta em um tempo de execução fixo para qualquer entrada.


```
Função calcula_custo_taxi(origem, destino, dist_taxi, adjacente):
    Parâmetros:
        origem (índice do vértice de origem),
        destino (índice do vértice de destino),
        dist_taxi (distância acumulada de táxi até o ponto atual),
        adjacente (segmento do tipo SegmentoBusca representando o próximo trecho)

    Inicializar:
        segmento_tamanho = adjacente.distancia  // Tamanho do segmento atual
        nova_distancia = dist_taxi + segmento_tamanho  // Distância total até o destino
        custo = 0.0  // Inicializa o custo como 0.0

    Se nova_distancia > limite_km:
        // Se a nova distância excede o limite de km gratuitos
        custo = taxa_variavel * (nova_distancia - limite_km)  // Calcula o custo extra

    Retornar par(custo, nova_distancia)  // Retorna o custo extra e a nova distância acumulada
```

##### calcula_custo()

A função `calcula_custo` calcula o custo de transitar de um segmento para outro, considerando o tipo de transporte e as condições de cada segmento. Quando o tipo de transporte entre os segmentos é diferente, como ao mudar de ônibus para metrô, ela retorna o custo fixo associado a essa troca (exemplo: custo de passagem para metrô ou ônibus). Se o segmento adjacente for um táxi e o deslocamento for vertical, é cobrado um custo fixo para o táxi. Caso contrário, se o táxi for horizontal, o custo é calculado levando em conta o limite de km gratuito, aplicando uma taxa variável se necessário. Se não houver mudança de meio de transporte, o custo retornado é zero. \(O(1)\).

```
Função calcula_custo(atual, adjacente, distancia_taxi):
    Parâmetros:
        atual (segmento atual do tipo SegmentoBusca),
        adjacente (segmento adjacente do tipo SegmentoBusca),
        distancia_taxi (distância acumulada de táxi até o ponto atual)

    Se atual.meioTransporte != adjacente.meioTransporte:
        Se adjacente.meioTransporte == "metro":
            Retornar (passagem_metro, distancia_taxi)  // Custo fixo para mudar para o metrô
        
        Se adjacente.meioTransporte == "onibus":
            Retornar (passagem_onibus, distancia_taxi)  // Custo fixo para mudar para ônibus

    Se adjacente.meioTransporte == "taxi" E adjacente.vertical == Verdadeiro:
        Retornar (taxa_fixa, distancia_taxi)  // Custo fixo para mudar para táxi (no caso vertical)

    Se adjacente.meioTransporte == "taxi" E adjacente.vertical == Falso:
        // Se o meio de transporte for táxi e não for vertical
        Retornar calcula_custo_taxi(atual.vDestino, adjacente.vDestino, distancia_taxi, adjacente)  // Calcula o custo de táxi com base na distância

    Retornar (0.0, distancia_taxi)  // Se não houver mudança de meio de transporte, retorno 0 de custo
```

##### dijkstra_custo()

A função `dijkstra_custo` implementa o algoritmo de Dijkstra para encontrar o caminho de menor custo entre dois vértices em um grafo, levando em consideração um limite de custo (dinheiro) e o tempo de viagem acumulado ao longo do percurso. O grafo é representado por um objeto `PlantaBusca`, que é um conjunto de segmentos interconectados entre si. O objetivo é buscar o caminho que minimize o tempo de viagem, respeitando o custo acumulado de todos os segmentos percorridos até o destino, de forma que o custo total não ultrapasse o limite estabelecido. 

A função começa inicializando três mapas: `tempo_minimo`, `custo_acumulado` e `segmento_pai`. O primeiro mapa armazena o menor tempo encontrado até cada segmento, o segundo armazena o custo acumulado até esse ponto, e o último é usado para reconstruir o caminho mais curto após a execução do algoritmo. A fila de prioridade (`priority_queue`) é usada para processar os segmentos, começando pelo vértice inicial, onde o custo e o tempo iniciais são zero. Cada segmento é processado, e para cada um, calcula-se o custo e o tempo até os seus vizinhos (segmentos adjacentes), e se um caminho mais eficiente (com menor tempo e custo dentro do limite) for encontrado, ele é atualizado.

O algoritmo continua iterando até que todos os segmentos possíveis sejam processados ou até que o destino seja alcançado. Quando o destino é encontrado ou a fila de prioridade estiver vazia, a reconstrução do caminho é feita a partir dos predecessores (armazenados no mapa `segmento_pai`). O caminho final é reconstruído na ordem correta e retornado como um vetor de segmentos. A complexidade do algoritmo é dominada pela operação de extração de mínimo da fila de prioridade, o que resulta em uma complexidade de tempo de O((V + E) log V), onde V é o número de vértices e E é o número de arestas no grafo. Isso ocorre porque, para cada vértice, o algoritmo pode realizar até E operações de relaxamento, e cada operação de extração de mínimo da fila tem complexidade logarítmica em relação ao número de vértices.

```
Função dijkstra_custo(grafo, vertice_inicial, vertice_destino, lim_dinheiro):
    Parâmetros:
        grafo (PlantaBusca contendo segmentos e adjacências)
        vertice_inicial (índice do vértice de origem)
        vertice_destino (índice do vértice de destino)
        lim_dinheiro (limite de custo disponível para o percurso)

    Inicializar:
        tempo_minimo = mapa de tempos mínimos para cada segmento
        custo_acumulado = mapa de custos acumulados para cada segmento
        segmento_pai = mapa para armazenar o "pai" de cada segmento
        fila = fila de prioridade para armazenar os estados (segmento, custo acumulado, tempo acumulado)

    Para cada vértice no grafo:
        tempo_minimo[vértice] = infinito
        custo_acumulado[vértice] = infinito

    Para cada segmento adjacente ao vertice_inicial:
        Adicionar o segmento à fila com custo inicial zero

    Enquanto a fila não estiver vazia:
        Extrair o estado com o menor custo acumulado
        Se o custo acumulado > lim_dinheiro, continuar com o próximo estado
        Se o segmento atual for o destino, interromper

        Para cada segmento adjacente ao segmento atual:
            Calcular o custo e a nova distância de táxi
            Calcular o novo custo e tempo acumulado

            Se o novo custo <= lim_dinheiro e o novo tempo < tempo_minimo:
                Atualizar tempo_minimo e custo_acumulado
                Adicionar o novo estado à fila
                Definir o segmento atual como pai do segmento adjacente

    Reconstruir o caminho:
        Iniciar no segmento de destino
        Seguir os pais até o vertice_inicial

    Inverter o caminho para começar do vertice_inicial
    Retornar o caminho reconstruído
```

Em resumo, os algoritmos e funções descritos fornecem uma estrutura eficiente para calcular e otimizar rotas em sistemas de transporte multimodal, como metrô, ônibus e táxi. Através da combinação de algoritmos clássicos, como o de Dijkstra, e o uso de grafos expandidos para representar os diferentes meios de transporte, o sistema é capaz de identificar as rotas mais rápidas e com menor custo entre diferentes pontos. A integração desses componentes permite simular de forma precisa e eficaz os fluxos de transporte urbano, facilitando a escolha das melhores opções de viagem para os usuários.
