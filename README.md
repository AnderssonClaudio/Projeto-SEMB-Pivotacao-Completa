# Projeto-SEMB-Pivotacao-Completa

* Algoritmo de Pivotação Completa implementado no PIC 16F18875

* O algoritmo a seguir, utiliza o método da pivotação completa para achar uma matriz N*N+1
* equivalente, a fim de facilitar a resolução de um Sistema Linear.
* O método consiste em achar o maior número em módulo da matriz, onde será o pivô. A partir
* do pivô, utilizando o método de gauss, é possível achar os
* multiplicadores de cada linha a ser zerada. O multiplicador é dado por -(elemento a ser
* zerado/pivô). A partir dos multiplicadores de cada linha é possível obter
* uma matriz equivalente da matriz original. Para toda a linha do pivô, os valores serão
* multiplicados pelo multiplicador da linha a ser zerada e serão
* somados a seus respectivos valores, gerando a matriz equivalente. Para cada iteração, a
* linha do pivô será extraída da matriz original, dando início a
* uma nova iteração, onde serão achados um novo pivô e será realizada novas operações a fim
* de achar uma nova matriz equivalente. O método termina quando restar
* somente uma linha da matriz original. Assim juntaM-se as outras linhas que foram extraídas
* formando enfim a matriz equivalente que poderá ser operada para
* resolver o sistema. Para facilitar a resolução do sistema, ainda é feita a organização da matriz
* equivalente a fim de obter uma matriz triangular superior.

* Este algoritmo foi desenvolvido como trabalho para a cadeira de Sistemas Embarcados,
* constituída no curso de Bacharelado em Engenharia de Computação.
