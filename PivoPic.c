/*
* @(#)PivoPic.c    1.00 26/11/2018
*
* Autores: Andersson Cláudio Alves Santana e Paulo Victor da Silva Mourão
* Instituto Federal de Educação, Ciência e Tecnologia do Ceará - IFCE
* Todos direitos reservados
*
* Copyright - Este trabalho é protegido pelas leis de direitos autorais contidas na
* constituição brasileira. É proibido qualquer tipo de venda deste material. As
* informações contidas neste trabalho podem ser utilizadas para fins de estudo,
* desde que previamente solicitada a autorização dos autores, desde que dentro
* dessas leis.

* Contato: anderssonclaudio0@gmail.com, pvictormourao@gmail.com
-----------------------------------------------------------------
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
-----------------------------------------------------------------
* Entrada: Matriz de N linhas e N colunas, no formato NxN+1.
* Saída: A matriz resultante, além das raízes do sistema linear, visualizadas no depurador do MPLAB X
 */
/*--------------------------------------------------------------
--  #1.
--  Data: Nov,07,2018
--  Autor: Andersson Cláudio e Paulo Victor
--  Motivo: Implementação dos Pragmas, includes e defines necessários para o funcionamento no pic.
-------------------------------------------------------------
--  #2.
--  Data: Nov,09,2018
--  Autor: Andersson Cláudio e Paulo Victor
--  Motivo:  Melhoria na função de organização da matriz triangular.
-------------------------------------------------------------
--  #3.
--  Data: Nov,12,2018
--  Autor: Andersson Cláudio
--  Motivo: Implementação da função BuracoVet
-------------------------------------------------------------
--  #4.
--  Data: Nov,16,2018
--  Autor: Andersson Cláudio
--  Motivo: Modificação dos requisitos de 800 floats. Só foi possível utilizar uma matriz 7x8 declarada manualmente.
-------------------------------------------------------------
--  #5.
--  Data: Nov,21,2018
--  Autor: Andersson Cláudio e Paulo Victor
--  Motivo:
        a) Implementação da função de substituição
        b) Mudança na declaração do tipo de algumas variáveis, mudança na declaração dos vetores principais para globais e reaproveitamento de vetores
-------------------------------------------------------------
--  #6.
--  Data: Nov,24,2018
--  Autor: Paulo Victor
--  Motivo: Melhoria na função de substituição

-------------------------------------------------------------
*/



//Por Andersson e Paulo Victor: #1.
//Configurações necessárias para o bom funcionamento do PIC 16F18875

//CONFIG1
#pragma config FEXTOSC = OFF    // External Oscillator mode selection bits (Oscillator not enabled)
#pragma config RSTOSC = HFINT32 // Power-up default value for COSC bits (HFINTOSC with OSCFRQ= 32 MHz and CDIV = 1:1)
#pragma config CLKOUTEN = OFF   // Clock Out Enable bit (CLKOUT function is disabled; i/o or oscillator function on OSC2)
#pragma config CSWEN = ON       // Clock Switch Enable bit (Writing to NOSC and NDIV is allowed)
#pragma config FCMEN = ON       // Fail-Safe Clock Monitor Enable bit (FSCM timer enabled)

// CONFIG2
#pragma config MCLRE = ON       // Master Clear Enable bit (MCLR pin is Master Clear function)
#pragma config PWRTE = OFF      // Power-up Timer Enable bit (PWRT disabled)
#pragma config LPBOREN = OFF    // Low-Power BOR enable bit (ULPBOR disabled)
#pragma config BOREN = ON       // Brown-out reset enable bits (Brown-out Reset Enabled, SBOREN bit is ignored)
#pragma config BORV = LO        // Brown-out Reset Voltage Selection (Brown-out Reset Voltage (VBOR) set to 1.9V on LF, and 2.45V on F Devices)
#pragma config ZCD = OFF        // Zero-cross detect disable (Zero-cross detect circuit is disabled at POR.)
#pragma config PPS1WAY = ON     // Peripheral Pin Select one-way control (The PPSLOCK bit can be cleared and set only once in software)
#pragma config STVREN = ON      // Stack Overflow/Underflow Reset Enable bit (Stack Overflow or Underflow will cause a reset)

// CONFIG3
#pragma config WDTCPS = WDTCPS_31// WDT Period Select bits (Divider ratio 1:65536; software control of WDTPS)
#pragma config WDTE = OFF       // WDT operating mode (WDT Disabled, SWDTEN is ignored)
#pragma config WDTCWS = WDTCWS_7// WDT Window Select bits (window always open (100%); software control; keyed access not required)
#pragma config WDTCCS = SC      // WDT input clock selector (Software Control)

// CONFIG4
#pragma config WRT = OFF        // UserNVM self-write protection bits (Write protection off)
#pragma config SCANE = available// Scanner Enable bit (Scanner module is available for use)
#pragma config LVP = ON         // Low Voltage Programming Enable bit (Low Voltage programming enabled. MCLR/Vpp pin function is MCLR.)

// CONFIG5
#pragma config CP = OFF         // UserNVM Program memory code protection bit (Program Memory code protection disabled)
#pragma config CPD = OFF        // DataNVM code protection bit (Data EEPROM code protection disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.

//#include <xc.h>
#include <stdio.h>
#include <stdlib.h>
#define _XTAL_FREQ 32000000
#define F_CPU 32000000/64//#define Baud_value(baud_rate) (((float)(F_CPU)/(float)baud_rate)-1)
#define Baud_value (((float)(F_CPU)/(float)baud_rate)-1)//calculus for UART serial tramission rate
//fim por Andersson e Paulo Victor: #1.

#define linhas 3 //Definição do tamanho de linhas da matriz. Poderá ser modificada até atingir as limitações da plataforma e do código, no caso 7 linhas
#define colunas 4 //Definição do tamanho de colunas da matriz. Poderá ser modificada até atingir as limitações da plataforma e do código, no caso 8 colunas

/*Utilizado somente quando é necessário visualizar a saída no simulador
void putch(unsigned char data) {
    while(!TRMT);//Waiting for previous data to transmit completely
    TXREG = data;//Writing data to Transmit Register and starts transmission
}
*/
//Por Andersson Cláudio e Paulo Victor: 5#.b)
//Declaração da estrutura de matriz do tipo float com linhas e colunas definidas pelas constantes de mesmo nome

typedef struct{
	float mat[linhas][colunas];
}mat;

/*Declaração da matriz original m2, a equivalente res e a matriz triangular tri. Também foi declarado como global dois vetores, um de float para ser utilizado como o vetor dos multiplicadores
e posteriormente o vetor de raízes. O outro vetor de inteiros é utilizado para organizar a matriz triangular, como proteção contra sobrescrita de colunas com a mesma quantidade de
zeros e por fim, para identificar cada raíz.*/

mat m2;
mat res;
mat tri;
float v[linhas];
int vetAux[colunas-1];
//Fim por Andersson Cláudio e Paulo Victor: #5.b)

//Por Andersson Cláudio: #4.
/*Função para preencher a matriz que é passada como parâmetro (mat* matriz) .*/

void preenchematriz(mat* matriz){

/* A matriz é preenchida manualmente já que existe uma limitação do PIC ao usar bibliotecas que possam gerar números randômicos.
A matriz 3x4, usada para exemplo, é uma matriz conhecida, com raízes previamente calculadas manualmente. A matriz 7x8 foi gerada por
um programa a parte utilizando números randômicos de 0 a 100.

Abaixo, declaração da matriz 3x4 usada para exemplo*/

    matriz->mat[0][0] = 1;
    matriz->mat[0][1] = 5;
    matriz->mat[0][2] = -2;
    matriz->mat[0][3] = 11;
    matriz->mat[1][0] = -2;
    matriz->mat[1][1] = 4;
    matriz->mat[1][2] = 1;
    matriz->mat[1][3] = 3;
    matriz->mat[2][0] = 3;
    matriz->mat[2][1] = -1;
    matriz->mat[2][2] = 4;
    matriz->mat[2][3] = 11;


// Matriz 7x8, valor máximo que foi possível implementar até o momento
/*
    matriz->mat[0][0] = 82;
    matriz->mat[0][1] = 69;
    matriz->mat[0][2] = 96;
    matriz->mat[0][3] = 19;
    matriz->mat[0][4] = 37;
    matriz->mat[0][5] = 22;
    matriz->mat[0][6] = 43;
    matriz->mat[0][7] = 91;

    matriz->mat[1][0] = 70;
    matriz->mat[1][1] = 70;
    matriz->mat[1][2] = 38;
    matriz->mat[1][3] = 47;
    matriz->mat[1][4] = 27;
    matriz->mat[1][5] = 18;
    matriz->mat[1][6] = 0;
    matriz->mat[1][7] = 60;

    matriz->mat[2][0] = 80;
    matriz->mat[2][1] = 12;
    matriz->mat[2][2] = 49;
    matriz->mat[2][3] = 34;
    matriz->mat[2][4] = 4;
    matriz->mat[2][5] = 94;
    matriz->mat[2][6] = 21;
    matriz->mat[2][7] = 45;

    matriz->mat[3][0] = 43;
    matriz->mat[3][1] = 11;
    matriz->mat[3][2] = 10;
    matriz->mat[3][3] = 93;
    matriz->mat[3][4] = 36;
    matriz->mat[3][5] = 81;
    matriz->mat[3][6] = 2;
    matriz->mat[3][7] = 99;

    matriz->mat[4][0] = 85;
    matriz->mat[4][1] = 15;
    matriz->mat[4][2] = 79;
    matriz->mat[4][3] = 98;
    matriz->mat[4][4] = 13;
    matriz->mat[4][5] = 3;
    matriz->mat[4][6] = 26;
    matriz->mat[4][7] = 36;

    matriz->mat[5][0] = 38;
    matriz->mat[5][1] = 96;
    matriz->mat[5][2] = 93;
    matriz->mat[5][3] = 52;
    matriz->mat[5][4] = 8;
    matriz->mat[5][5] = 10;
    matriz->mat[5][6] = 97;
    matriz->mat[5][7] = 40;

    matriz->mat[6][0] = 32;
    matriz->mat[6][1] = 18;
    matriz->mat[6][2] = 91;
    matriz->mat[6][3] = 86;
    matriz->mat[6][4] = 44;
    matriz->mat[6][5] = 76;
    matriz->mat[6][6] = 98;
    matriz->mat[6][7] = 53;
 */
 }
//Fim por Andersson Cláudio: 4#.

//Função que imprime os valores da matriz(somente quando usa UART ou com simulador)

void imprimeMatriz(mat* matriz, int linha, int coluna){

    int i, j;

	for(i=0; i<linha; i++){
		for(j=0; j<coluna; j++){
			printf("%.f|", matriz->mat[i][j]);
		}
		printf("\n");
	}
}

/*Função para encontrar pivô e gerar uma matriz com os cálculos já efetuados.
Ela varre a matriz comparando cada valor com a variável maior. Se for maior que o valor da variável, ela faz a troca pelo novo valor e armazena as posições i e j nas variáveis postAux_i e posAux_j respectivamente.
No final da varredura, ela vai ter a linha do pivô para efetuar os cálculos.

*/
void MetodoPivo(mat* matriz,mat* res, int linha, int coluna, float * v ){

	int i = 0; //contador
	int j = 0; //contador
	int k = 0; //contador
	int l = 0; //contador
	int cont = 0; //contador que determina o tanto de iterações do laço While (cont < linha)
    int posAux_i, posAux_j; //posição da linha e coluna em que se encontra o pivô
	float maior; //variável que armazena o pivô

    /*
    Esta primeira parte varre a matriz comparando cada valor com a variável maior. Se for maior que o valor da variável,
    ocorre a troca pelo novo valor e armazena as posições i e j nas variáveis postAux_i e posAux_j respectivamente.
    No final da varredura, ela vai ter a linha do pivô para efetuar os cálculos.
    */
	while(cont<linha){ //Percorre a matriz pelas linhas
    maior = 0;
		for(i=0 ; i<linha ; i++){ // varre a matriz nas linhas para encontrar o maior
			for(j =0; j<coluna-1 ; j++){// varre a matriz nas colunas para encontrar o maior
				if(matriz->mat[i][j]<0){
                    // verificação para saber qual número é o maior se for menor que zero, ocorre a multiplicação por -1(Valor em módulo).
					if(maior<(matriz->mat[i][j]*(-1))){
						maior = (matriz->mat[i][j])*(-1);
						posAux_i = i;
						posAux_j = j;
					}
				}else{
					if(maior < matriz->mat[i][j]){
						maior = matriz->mat[i][j];
						posAux_i = i;
						posAux_j = j;
					}
				}
			}
		}

        /*

        Cálculo dos multiplicadores que são armazenados em um vetor v[], declarado como global.
        O multiplicador é obtido pelo valor negativo da divisão dos elementos de posição i e coluna posAux_j pelo pivô

        */
		for(j=0;j<linha;j++){
			if(j!=posAux_i){
        			v[j] = (matriz->mat[j][posAux_j]/matriz->mat[posAux_i][posAux_j])*(-1);
				//printf("%lf\n\n", v[j]);
			}
       	}
            // Faz as operações na matriz para calcular a matriz resultante de acordo com o método
	    for( k = 0 ; k < coluna; k++ ){
            for( l = 0 ; l < linha ; l++ ){
                // As operações são feitas por colunas, ou seja, cada linha de uma coluna é feita a operação e depois muda para outra coluna.
            	if( l != posAux_i ){
                    matriz->mat[l][k] = ( v[l] * matriz->mat[posAux_i][k] ) + matriz->mat[l][k];
				}
			}
		}
		//matriz equivalente que recebe os valores da matriz original e a original é zerada na linha do pivô para não ocorrer do mesmo pivô sempre ser o selecionado
		for(j =0 ; j<coluna; j++){
			res->mat [cont][j] = matriz->mat[posAux_i][j];
            matriz->mat[posAux_i][j] = 0 ;
		}
		cont++;
	}
}

/*
//Por Andersson Cláudio: #3.
Função utilizada para encontrar um espaço na esquerda ou direita de um índice que já estava ocupado no vetor.
Pode ocorrer casos onde colunas diferentes tenha a mesma quantidade de zeros e como utilizamos a quantidade como índice do vetor,
pode ocorrer sobrescrita e futuramente quando tentar copiar os valores para a matriz final triangular, pode ter algum valor absurdo no
vetor e ocasionar um erro no código.

Esta função recebe um vetAux que foi declarado global, contZero que vai indicar a posição no vetor e a quantidade de colunas.

O retorno é utilizado na função organiza para verificar em que lado do vetor tem posição livre para armazenar a quantidade de zeros.
    0 - Esquerda
    1 - Direita

*/

int BuracoVet(int * vetAux, int contZero, int coluna){
    int i, j; //contadores

    for(i=0; i<coluna-1; i++){
//Verifica se o valor do vetor é -1 e se o valor da posição do vetor é menor que a posição de índice contZero(quantidade de zeros).
        if (vetAux[i]==-1 && vetAux[i]<vetAux[contZero]){
//Move todos os valores para a esquerda em caso positivo
            for(j=i; j<contZero-1; j++){
                vetAux[j]=vetAux[j+1];
            }
            return 0; //esquerda
        }
//Caso seja maior e o valor adjacente seja -1. Ele vai mover todos os valores para a direita.
        if (vetAux[i]==-1 && vetAux[i]>vetAux[contZero]){
            for(j=i; j>contZero+1; j--){
                vetAux[i+1]=vetAux[i];
            }
            return 1; //direita
        }
    }
}
//Fim por Andersson Cláudio: #3



/*
//Por Andersson e Paulo Victor: #2

Função para organizar a matriz equivalente em uma triangular superior.
Recebe a matriz equivalente res, uma matriz vazia para ser povoada, a quantidade de linhas e colunas e um vetAux que irá ser reutilizado.

*/

void organiza(mat* res, mat* resultado, int linha, int coluna, int * vetAux){

    int i; //Contador
    int j; //Contador
    int l=-1; //Contador especial iniciado com -1 para não contar na primeira iteração.
    int contZero; //Contador de zeros.
    int teste, flag; // auxiliares
     //30
    /*

    O vetor vetAux é preenchido com -1 para sempre que for feito a tentativa de escrever a posição da coluna nele, ele só pode fazer isso se tiver esse valor.
    Uma forma de proteger sobrescrita. Como utilizamos a quantidade de zeros como índice do vetor, pode ocorrer casos onde colunas diferentes tenha a mesma quantidade de zeros
    e futuramente quando tentar copiar os valores para a matriz final triangular, pode ter algum valor absurdo no vetor e ocasionar um erro no resultado ou o programa pode parar de responder.

    */
    for(i=0; i<coluna-1; i++){
        vetAux[i]=-1;
    }
    //Um for para percorrer as colunas e verificar quantos zeros possui a matriz.
    /*

    Ao fim dessa contagem de zeros, o contZero vai servir como índice do vetor vetAux para armazenar a posição da coluna. A que tem 0 Zeros fica na posição vetAux[0],
    a coluna que tem 3 zeros fica na posição vetAux[3].

    */
    for(j=0; j<coluna-1; j++){
        contZero = 0;
        for (i=0; i<linha; i++){
                teste = res->mat[i][j];
            if(teste == 0){
                contZero++;
            }
        }

        if (vetAux[contZero]==-1){
            vetAux[contZero] = j;
        }else{
            BuracoVet(vetAux, contZero, coluna);
            flag = BuracoVet(vetAux, contZero, coluna); //chama função para procurar espaço no vetor. De acordo com o retorno, ele coloca o valor na esquerda ou direita do índice que ja foi utilizado
            if (flag == 0){
                vetAux[contZero-1] = j;
            }
            if (flag == 1){
                vetAux[contZero+1] = j;
            }
        }
    }

//A maior quantidade de zeros em uma coluna fica mais à esquerda de uma matriz. Então o vetor salvo com as posições anteriormente, é percorrido do fim para o início para povoar a matriz resultado.
    for(j=coluna-1; j>=0; j--){

        for(i=0; i<linha; i++){
            //caso especial onde a última coluna não entra no vetor de posições, pois a última coluna não possui zeros.
            if(j == coluna-1){
                resultado->mat[i][j] = res->mat[i][j];
            }else{
                teste = vetAux[j];
                resultado->mat[i][l] = res->mat[i][teste];
            }
        }
        l++;
    }
}
//Fim por Andersson Cláudio e Paulo Victor #2

//Por Andersson Cláudio e Paulo Victor #5.a)
/*
Função que faz o cálculo das raízes.
Recebe a matriz resultado, quantidade de linhas, colunas, um vetor de float v e o vetor de inteiros vetAux.

Essa função vai percorrer linha a linha calculando as raízes de acordo com o método de substituição retroativa.

//Por Paulo Victor #6.
*O vetAux é reutilizado aqui para indicar as posições das raízes. Assim foi necessária uma adaptação na função
*/
void substituicao(mat* resultado, int linha, int coluna, float * v, int * vetAux){

	float s; //s é a soma dos valores de uma linha em que já se sabe as raizes
	int i, j; //contadores
    short cont=0; //contador utilizado para saber qual linha utilizar
    float vet[linhas]; //vetor que irá guardar as raízes

	v[linha-1] = resultado->mat[linha-1][linha] / resultado->mat[linha-1][linha-1];
    vet[vetAux[cont]] = v[linha-1];
    cont++;
	for (i = linha-2; i >= 0; i--){
		s = 0;
    	for (j = i+1; j < linha; j++){
      		s += resultado->mat[i][j] * v[j];
    	}
    	v[i] = (resultado->mat[i][linha] - s)/ resultado->mat[i][i];
        vet[vetAux[cont]] = v[i];
        cont++;
    }

  	for (i = 0; i < linha; i++){
    	printf("\nx%d = %f\n", i+1, vet[i]);
  	}

}
//Fim por Andersson Cláudio e Paulo Victor #5.a)
//Fim por Paulo Victor #6

int main(void) {

//Na função Main só estão as chamadas de função e funções de print para o eventual uso de um simulador

	preenchematriz(&m2); //Preenche a matriz com números aleatórios
	printf("\n\n\n----------matriz original---------\n\n\n");
	imprimeMatriz(&m2, linhas, colunas); //imprime a matriz
	MetodoPivo(&m2,&res, linhas, colunas, v); //encontra o pivô e faz o método da pivotação
	printf("\n\n\n----------matriz equivalente---------\n\n\n");
	imprimeMatriz(&res, linhas, colunas);
	organiza(&res, &tri, linhas,colunas, vetAux); //organiza a matriz equivalente em uma triangularizada
	printf("\n\n\n----------matriz triangular-------------\n\n\n");
	imprimeMatriz(&tri, linhas, colunas);
    substituicao(&tri, linhas, colunas, v, vetAux); //implementa o método da substituição retroativa para encontrar as raízes do sistema



}


