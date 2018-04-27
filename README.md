Para o controle de veículos no SUMO, pode-se usar o TraCI,
preferencialmente sua API para python. De modo a facilitar o trabalho,
há alguns módulos lidando com o grafo da rede e a busca de caminhos
nele.

As aplicações já feitas podem ser usadas como base para as
próximas. Algumas de suas classes, por exemplo, talvez possam ser
reutilizadas.  Para isso, recomenda-se que uma boa documentação delas
seja mantida nas próprias docstrings do python, cuja atualização é
mais fácil de fazer e difícil de esquecer, além de haver várias
ferramentas que facilitam a navegação por elas.

# Módulos de uso geral

A comunicação com o SUMO se dá pelo pacote TraCI,
disponível no diretório tools/traci da distribuição do SUMO.

Há um pacote python do próprio SUMO para lidar com suas redes, o
sumolib. Ele se encontra no diretório tools/sumolib da distribuição do
SUMO.

Às vezes é necessário guardar mais informações sobre as edges do que
há na classe, por exemplo os distritos a que pertencem ou as ocupações
em uma janela móvel de tempo. A maneira mais fácil de mantê-las é
adicionar campos aos objetos dinamicamente, como python permite. Por
exemplo:

    edge.districts = set()

Foi desenvolvido também um módulo para busca de caminho mínimo no
grafo definido por essa rede, com os algoritmos Dijkstra e A*. Esse
módulo é o search.py e se encontra no diretório vehcontrol do
repositório. Os custos de cada edge e a heurística usada pelo A* são
parâmetros: funções que, respectivamente, recebem uma edge retornando
seu custo, e recebem um par de edges retornando sua distância
heurística.

# Aplicações de controle

## Motoristas com/sem GPS

Uma modelagem do uso ou não de GPS pode ser encontrada no aplicativo gpsdrivers.py

O programa recebe um arquivo de rede, arquivos de distritos e arquivos
de rotas. O comportamento de cada veículo é determinado pelo seu tipo,
e tipos desconhecidos são ignorados. Mais detalhes sobre os tipos
conhecidos e outras opções do programa podem ser consultados
chamando-o com a opção '--help':

    ./gpsdrivers.py --help

As classes provavelmente reutilizáveis, com alguma adaptação, são:

- Os drivers, que provêem uma interface genérica para motoristas que
  os controlam de uma forma "event-based" (um método é chamado quando
  entra na rede, outro quando sai da rede...)

- Em particular, o RoutedDriver, que define motoristas que calculam
  sua rota ao entrar na rede, usando uma avaliação parametrizável do
  custo das edges. Isso permite usar a mesma classe para vários
  comportamentos, por exemplo: caminho mais curto; levar em
  consideração informações observadas; evitar ruas desconhecidas.

- Os edge evaluators, que definem o comportamento de RoutedDrivers e
  podem acumular informações sobre edges.

Para mais detalhes sobre cada classe, verificar as docstrings no
código ou com as ferramentas tradicionais do python ('help()',
pydoc...)
