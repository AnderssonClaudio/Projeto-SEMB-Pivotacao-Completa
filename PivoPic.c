/*
* @(#)PivoPic.c    1.00 26/11/2018
*
* Autores: Andersson Cl�udio Alves Santana e Paulo Victor da Silva Mour�o
* Instituto Federal de Educa��o, Ci�ncia e Tecnologia do Cear� - IFCE
* Todos direitos reservados
*
* Copyright - Este trabalho � protegido pelas leis de direitos autorais contidas na
* constitui��o brasileira. � proibido qualquer tipo de venda deste material. As
* informa��es contidas neste trabalho podem ser utilizadas para fins de estudo,
* desde que previamente solicitada a autoriza��o dos autores, desde que dentro
* dessas leis.

* Contato: anderssonclaudio0@gmail.com, pvictormourao@gmail.com
-----------------------------------------------------------------
* Algoritmo de Pivota��o Completa implementado no PIC 16F18875

* O algoritmo a seguir, utiliza o m�todo da pivota��o completa para achar uma matriz N*N+1
* equivalente, a fim de facilitar a resolu��o de um Sistema Linear.
* O m�todo consiste em achar o maior n�mero em m�dulo da matriz, onde ser� o piv�. A partir
* do piv�, utilizando o m�todo de gauss, � poss�vel achar os
* multiplicadores de cada linha a ser zerada. O multiplicador � dado por -(elemento a ser
* zerado/piv�). A partir dos multiplicadores de cada linha � poss�vel obter
* uma matriz equivalente da matriz original. Para toda a linha do piv�, os valores ser�o
* multiplicados pelo multiplicador da linha a ser zerada e ser�o
* somados a seus respectivos valores, gerando a matriz equivalente. Para cada itera��o, a
* linha do piv� ser� extra�da da matriz original, dando in�cio a
* uma nova itera��o, onde ser�o achados um novo piv� e ser� realizada novas opera��es a fim
* de achar uma nova matriz equivalente. O m�todo termina quando restar
* somente uma linha da matriz original. Assim juntaM-se as outras linhas que foram extra�das
* formando enfim a matriz equivalente que poder� ser operada para
* resolver o sistema. Para facilitar a resolu��o do sistema, ainda � feita a organiza��o da matriz
* equivalente a fim de obter uma matriz triangular superior.

* Este algoritmo foi desenvolvido como trabalho para a cadeira de Sistemas Embarcados,
* constitu�da no curso de Bacharelado em Engenharia de Computa��o.
-----------------------------------------------------------------
* Entrada: Matriz de N linhas e N colunas, no formato NxN+1.
* Sa�da: A matriz resultante, al�m das ra�zes do sistema linear, visualizadas no depurador do MPLAB X
 */
/*--------------------------------------------------------------
--  #1.
--  Data: Nov,07,2018
--  Autor: Andersson Cl�udio e Paulo Victor
--  Motivo: Implementa��o dos Pragmas, includes e defines necess�rios para o funcionamento no pic.
-------------------------------------------------------------
--  #2.
--  Data: Nov,09,2018
--  Autor: Andersson Cl�udio e Paulo Victor
--  Motivo:  Melhoria na fun��o de organiza��o da matriz triangular.
-------------------------------------------------------------
--  #3.
--  Data: Nov,12,2018
--  Autor: Andersson Cl�udio
--  Motivo: Implementa��o da fun��o BuracoVet
-------------------------------------------------------------
--  #4.
--  Data: Nov,16,2018
--  Autor: Andersson Cl�udio
--  Motivo: Modifica��o dos requisitos de 800 floats. S� foi poss�vel utilizar uma matriz 7x8 declarada manualmente.
-------------------------------------------------------------
--  #5.
--  Data: Nov,21,2018
--  Autor: Andersson Cl�udio e Paulo Victor
--  Motivo:
        a) Implementa��o da fun��o de substitui��o
        b) Mudan�a na declara��o do tipo de algumas vari�veis, mudan�a na declara��o dos vetores principais para globais e reaproveitamento de vetores
-------------------------------------------------------------
--  #6.
--  Data: Nov,24,2018
--  Autor: Paulo Victor
--  Motivo: Melhoria na fun��o de substitui��o

-------------------------------------------------------------
*/



//Por Andersson e Paulo Victor: #1.
//Configura��es necess�rias para o bom funcionamento do PIC 16F18875

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

#define linhas 3 //Defini��o do tamanho de linhas da matriz. Poder� ser modificada at� atingir as limita��es da plataforma e do c�digo, no caso 7 linhas
#define colunas 4 //Defini��o do tamanho de colunas da matriz. Poder� ser modificada at� atingir as limita��es da plataforma e do c�digo, no caso 8 colunas

/*Utilizado somente quando � necess�rio visualizar a sa�da no simulador
void putch(unsigned char data) {
    while(!TRMT);//Waiting for previous data to transmit completely
    TXREG = data;//Writing data to Transmit Register and starts transmission
}
*/
//Por Andersson Cl�udio e Paulo Victor: 5#.b)
//Declara��o da estrutura de matriz do tipo float com linhas e colunas definidas pelas constantes de mesmo nome

typedef struct{
	float mat[linhas][colunas];
}mat;

/*Declara��o da matriz original m2, a equivalente res e a matriz triangular tri. Tamb�m foi declarado como global dois vetores, um de float para ser utilizado como o vetor dos multiplicadores
e posteriormente o vetor de ra�zes. O outro vetor de inteiros � utilizado para organizar a matriz triangular, como prote��o contra sobrescrita de colunas com a mesma quantidade de
zeros e por fim, para identificar cada ra�z.*/

mat m2;
mat res;
mat tri;
float v[linhas];
int vetAux[colunas-1];
//Fim por Andersson Cl�udio e Paulo Victor: #5.b)

//Por Andersson Cl�udio: #4.
/*Fun��o para preencher a matriz que � passada como par�metro (mat* matriz) .*/

void preenchematriz(mat* matriz){

/* A matriz � preenchida manualmente j� que existe uma limita��o do PIC ao usar bibliotecas que possam gerar n�meros rand�micos.
A matriz 3x4, usada para exemplo, � uma matriz conhecida, com ra�zes previamente calculadas manualmente. A matriz 7x8 foi gerada por
um programa a parte utilizando n�meros rand�micos de 0 a 100.

Abaixo, declara��o da matriz 3x4 usada para exemplo*/

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


// Matriz 7x8, valor m�ximo que foi poss�vel implementar at� o momento
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
//Fim por Andersson Cl�udio: 4#.

//Fun��o que imprime os valores da matriz(somente quando usa UART ou com simulador)

void imprimeMatriz(mat* matriz, int linha, int coluna){

    int i, j;

	for(i=0; i<linha; i++){
		for(j=0; j<coluna; j++){
			printf("%.f|", matriz->mat[i][j]);
		}
		printf("\n");
	}
}

/*Fun��o para encontrar piv� e gerar uma matriz com os c�lculos j� efetuados.
Ela varre a matriz comparando cada valor com a vari�vel maior. Se for maior que o valor da vari�vel, ela faz a troca pelo novo valor e armazena as posi��es i e j nas vari�veis postAux_i e posAux_j respectivamente.
No final da varredura, ela vai ter a linha do piv� para efetuar os c�lculos.

*/
void MetodoPivo(mat* matriz,mat* res, int linha, int coluna, float * v ){

	int i = 0; //contador
	int j = 0; //contador
	int k = 0; //contador
	int l = 0; //contador
	int cont = 0; //contador que determina o tanto de itera��es do la�o While (cont < linha)
    int posAux_i, posAux_j; //posi��o da linha e coluna em que se encontra o piv�
	float maior; //vari�vel que armazena o piv�

    /*
    Esta primeira parte varre a matriz comparando cada valor com a vari�vel maior. Se for maior que o valor da vari�vel,
    ocorre a troca pelo novo valor e armazena as posi��es i e j nas vari�veis postAux_i e posAux_j respectivamente.
    No final da varredura, ela vai ter a linha do piv� para efetuar os c�lculos.
    */
	while(cont<linha){ //Percorre a matriz pelas linhas
    maior = 0;
		for(i=0 ; i<linha ; i++){ // varre a matriz nas linhas para encontrar o maior
			for(j =0; j<coluna-1 ; j++){// varre a matriz nas colunas para encontrar o maior
				if(matriz->mat[i][j]<0){
                    // verifica��o para saber qual n�mero � o maior se for menor que zero, ocorre a multiplica��o por -1(Valor em m�dulo).
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

        C�lculo dos multiplicadores que s�o armazenados em um vetor v[], declarado como global.
        O multiplicador � obtido pelo valor negativo da divis�o dos elementos de posi��o i e coluna posAux_j pelo piv�

        */
		for(j=0;j<linha;j++){
			if(j!=posAux_i){
        			v[j] = (matriz->mat[j][posAux_j]/matriz->mat[posAux_i][posAux_j])*(-1);
				//printf("%lf\n\n", v[j]);
			}
       	}
            // Faz as opera��es na matriz para calcular a matriz resultante de acordo com o m�todo
	    for( k = 0 ; k < coluna; k++ ){
            for( l = 0 ; l < linha ; l++ ){
                // As opera��es s�o feitas por colunas, ou seja, cada linha de uma coluna � feita a opera��o e depois muda para outra coluna.
            	if( l != posAux_i ){
                    matriz->mat[l][k] = ( v[l] * matriz->mat[posAux_i][k] ) + matriz->mat[l][k];
				}
			}
		}
		//matriz equivalente que recebe os valores da matriz original e a original � zerada na linha do piv� para n�o ocorrer do mesmo piv� sempre ser o selecionado
		for(j =0 ; j<coluna; j++){
			res->mat [cont][j] = matriz->mat[posAux_i][j];
            matriz->mat[posAux_i][j] = 0 ;
		}
		cont++;
	}
}

/*
//Por Andersson Cl�udio: #3.
Fun��o utilizada para encontrar um espa�o na esquerda ou direita de um �ndice que j� estava ocupado no vetor.
Pode ocorrer casos onde colunas diferentes tenha a mesma quantidade de zeros e como utilizamos a quantidade como �ndice do vetor,
pode ocorrer sobrescrita e futuramente quando tentar copiar os valores para a matriz final triangular, pode ter algum valor absurdo no
vetor e ocasionar um erro no c�digo.

Esta fun��o recebe um vetAux que foi declarado global, contZero que vai indicar a posi��o no vetor e a quantidade de colunas.

O retorno � utilizado na fun��o organiza para verificar em que lado do vetor tem posi��o livre para armazenar a quantidade de zeros.
    0 - Esquerda
    1 - Direita

*/

int BuracoVet(int * vetAux, int contZero, int coluna){
    int i, j; //contadores

    for(i=0; i<coluna-1; i++){
//Verifica se o valor do vetor � -1 e se o valor da posi��o do vetor � menor que a posi��o de �ndice contZero(quantidade de zeros).
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
//Fim por Andersson Cl�udio: #3



/*
//Por Andersson e Paulo Victor: #2

Fun��o para organizar a matriz equivalente em uma triangular superior.
Recebe a matriz equivalente res, uma matriz vazia para ser povoada, a quantidade de linhas e colunas e um vetAux que ir� ser reutilizado.

*/

void organiza(mat* res, mat* resultado, int linha, int coluna, int * vetAux){

    int i; //Contador
    int j; //Contador
    int l=-1; //Contador especial iniciado com -1 para n�o contar na primeira itera��o.
    int contZero; //Contador de zeros.
    int teste, flag; // auxiliares
     //30
    /*

    O vetor vetAux � preenchido com -1 para sempre que for feito a tentativa de escrever a posi��o da coluna nele, ele s� pode fazer isso se tiver esse valor.
    Uma forma de proteger sobrescrita. Como utilizamos a quantidade de zeros como �ndice do vetor, pode ocorrer casos onde colunas diferentes tenha a mesma quantidade de zeros
    e futuramente quando tentar copiar os valores para a matriz final triangular, pode ter algum valor absurdo no vetor e ocasionar um erro no resultado ou o programa pode parar de responder.

    */
    for(i=0; i<coluna-1; i++){
        vetAux[i]=-1;
    }
    //Um for para percorrer as colunas e verificar quantos zeros possui a matriz.
    /*

    Ao fim dessa contagem de zeros, o contZero vai servir como �ndice do vetor vetAux para armazenar a posi��o da coluna. A que tem 0 Zeros fica na posi��o vetAux[0],
    a coluna que tem 3 zeros fica na posi��o vetAux[3].

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
            flag = BuracoVet(vetAux, contZero, coluna); //chama fun��o para procurar espa�o no vetor. De acordo com o retorno, ele coloca o valor na esquerda ou direita do �ndice que ja foi utilizado
            if (flag == 0){
                vetAux[contZero-1] = j;
            }
            if (flag == 1){
                vetAux[contZero+1] = j;
            }
        }
    }

//A maior quantidade de zeros em uma coluna fica mais � esquerda de uma matriz. Ent�o o vetor salvo com as posi��es anteriormente, � percorrido do fim para o in�cio para povoar a matriz resultado.
    for(j=coluna-1; j>=0; j--){

        for(i=0; i<linha; i++){
            //caso especial onde a �ltima coluna n�o entra no vetor de posi��es, pois a �ltima coluna n�o possui zeros.
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
//Fim por Andersson Cl�udio e Paulo Victor #2

//Por Andersson Cl�udio e Paulo Victor #5.a)
/*
Fun��o que faz o c�lculo das ra�zes.
Recebe a matriz resultado, quantidade de linhas, colunas, um vetor de float v e o vetor de inteiros vetAux.

Essa fun��o vai percorrer linha a linha calculando as ra�zes de acordo com o m�todo de substitui��o retroativa.

//Por Paulo Victor #6.
*O vetAux � reutilizado aqui para indicar as posi��es das ra�zes. Assim foi necess�ria uma adapta��o na fun��o
*/
void substituicao(mat* resultado, int linha, int coluna, float * v, int * vetAux){

	float s; //s � a soma dos valores de uma linha em que j� se sabe as raizes
	int i, j; //contadores
    short cont=0; //contador utilizado para saber qual linha utilizar
    float vet[linhas]; //vetor que ir� guardar as ra�zes

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
//Fim por Andersson Cl�udio e Paulo Victor #5.a)
//Fim por Paulo Victor #6

int main(void) {

//Na fun��o Main s� est�o as chamadas de fun��o e fun��es de print para o eventual uso de um simulador

	preenchematriz(&m2); //Preenche a matriz com n�meros aleat�rios
	printf("\n\n\n----------matriz original---------\n\n\n");
	imprimeMatriz(&m2, linhas, colunas); //imprime a matriz
	MetodoPivo(&m2,&res, linhas, colunas, v); //encontra o piv� e faz o m�todo da pivota��o
	printf("\n\n\n----------matriz equivalente---------\n\n\n");
	imprimeMatriz(&res, linhas, colunas);
	organiza(&res, &tri, linhas,colunas, vetAux); //organiza a matriz equivalente em uma triangularizada
	printf("\n\n\n----------matriz triangular-------------\n\n\n");
	imprimeMatriz(&tri, linhas, colunas);
    substituicao(&tri, linhas, colunas, v, vetAux); //implementa o m�todo da substitui��o retroativa para encontrar as ra�zes do sistema



}


