/*--------------------------------------------------------------------

******************************************************************************
* @file control.cpp
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI
* @version V0.0.7
* @created 22/09/2014
* @Modified 23/09/2014
* @e-mail isaac25silva@yahoo.com.br
* @brief control
****************************************************************************
****************************************************************************

Arquivo fonte contendo o programa que com aprendizado por reforço 
utilizando o algoritmo Q-learning com heurística.

Os estados estão discretizados pelas posições do Acelerômetro.

/--------------------------------------------------------------------*/
#include <stdio.h>
#include <termio.h>
#include <unistd.h>
#include <dynamixel.h>
#include <sys/resource.h>
#include <sys/types.h>
#include <dirent.h>
#include <stdlib.h>
#include <termios.h>
#include <string.h>
#include "blackboard.h"
#include <iomanip>
#include <time.h>
#include <assert.h>
#include <float.h>
#include <limits.h>
#include <math.h>
#include <ctype.h>
#include <iostream>
#include <math.h>
#include "ql.h"
#include <iterator>
#include <fstream>
#include <vector>

#ifdef __cplusplus__
  #include <cstdlib>
#else
  #include <stdlib.h>
#endif

//void PrintCommStatus(int CommStatus);
//void PrintErrorCode(void);

using namespace std;

void saveQ(double qvalues[61][2], unsigned int passos, unsigned int Episodio);
void saveEpisodios(unsigned int Episodio, unsigned int passos);
unsigned int openFiletoGetQvalueVector(unsigned int &Episodio);

char *extraiDado (char *buffer, char *temp); //luiz
float filtro_Z(); //luiz
float filtro_X(); //luiz
int reforcoRobo(float sensor); //luiz
bool empe(float sensor); //luiz
int estadoROBO(float sensorZ,float sensorX); //luiz
float caso_para_Robo(float valor); //luiz
float robo_para_caso(float valor); //luiz
void similaridade(float AnguloQuadril, int estado); //luiz
void servo_funcionando();

///////// transfer //////////////////////////
int CASOS_USADOS=0;
void le_arquivo_casos();
char *extraiDado (char *buffer, char *temp);
#define NUM_CASOS 400        
double	CASOS_tetha1[NUM_CASOS];
double CASOS_tetha2[NUM_CASOS];
int CASOS_acao[NUM_CASOS];
double CASOS_reward_acao[NUM_CASOS];


////////////////////////////////////////////////////////////////////////////
////////////global variables////////////////////////////////////////////////
int inclina = 55;
/*unsigned int StandupPos[22] = {803, 512, //01 , 02, (cabeça)
                382, 417+20, 353+20, //03, 04, 05, (braço direito)
                472+170, 600-20, 689-20, //06, 07, 08, (braço esquerdo) 472
                512, 512+20+5, 512-5, 591-10-5, 512, 801+5+8+20-23, //09, 10, 11, 12, 13, 14 (perna direita)
               512, 512-25-5, 512+20, 492-5, 512, 224-5-8+23}; //15, 16, 17, 18, 19, 20 (perna esquerda)*/

//unsigned int StandupPos[22] = {742,292,612,552,582,512,502,572,507,592,379,592,507,402,462,342,459,442,424,762};
 unsigned int StandupPos[22] =  {842,212+170,457,622,022,987,517,532,517,592,535,482,522,442,542,517,602,512,1024,792};

//----Vetor que discretiza a posição fixa do acelerometro refente aos estados--------------------
float posAccel[31] = {-0.84, -0.845, -0.85, -0.855, -0.86, -0.865, -0.87, -0.875, -0.88, -0.89,
		   -0.90, -0.91, -0.92, -0.93, -0.94, -0.95, -0.96, -0.97, -0.98, -0.99,
		   -1.00, -1.01, -1.02, -1.03, -1.04, -1.05, -1.06, -1.07, -1.08, -1.09, -1.10};

int main(int argc, char* argv[])
{

    int * baudnum = new int; //alocado dinâmicamente
    *baudnum = DEFAULT_BAUDNUM;  //velocidade de transmissao da serial em 1Mbps
    int * deviceIndex = new int; //alocado dinâmicamente
    *deviceIndex = 0; 		//endereça USB
    unsigned int tensaomedia = 0;
    unsigned int passos = 0;

    using_shared_memory();

    char string[50]; //String usada para definir prioridade do programa

    //system("echo fei 123456| sudo -S chmod 777 /dev/ttyUSB*");//libera acesso a USB

    sprintf(string,"echo 123456 | sudo -S renice -20 -p %d", getpid()); // prioridade maxima na execução
    system(string);//prioridade
    printf( "Darwin Robot running...\n\n" );
   

// ---- Open USBDynamixel -----------------------------------------------{
bool servoComunica = false;
bool servoConectado = false;
	while(*deviceIndex<3)// laço que percorre o servo 0, 1 e 2.
	{
		if( dxl_initialize(*deviceIndex, *baudnum) == 0 )
		{
			printf( "Failed to open servo%d!\n", *deviceIndex );
			if(*deviceIndex==2)  // Não encontrou nenhum
			{
				if(servoComunica)
				    printf("Conectou-se a uma placa mas não conseguiu se comunicar com o servo\n");
				else
				    printf("Não encontrou porra nenhuma placa do servo conectada a porta USB\n");
			        return 0;
			}
			*deviceIndex = *deviceIndex + 1;      // Não conecta na placa do servo e tenta a proxima porta.
		}
		else
		{
			servoComunica = true;
			printf( "Succeed to open Servo%d!\n", *deviceIndex );
			servoConectado = dxl_read_byte( 10, 3 ) == 10;
			usleep(1000);
			servoConectado = dxl_read_byte( 10, 3 ) == 10;//Tenta novamente caso falhe a comunicação
			usleep(1000);
			servoConectado = dxl_read_byte( 10, 3 ) == 10;//Tenta novamente caso falhe a comunicação
    			if(servoConectado)
			{
       			 	printf("Servo%d okay - Connected and communicated!\n", *deviceIndex);
			 	break;
			}
    			else
    			{
				printf("Servo wrong or not communicated!\n");
				if(*deviceIndex==2)
				{
				    printf("Conectou-se a uma placa mas não conseguiu se comunicar com o servo\n");
				    return 0;
				}
				*deviceIndex = *deviceIndex + 1;
			}
		}
	}
	delete deviceIndex; delete baudnum; //desalocando da memória
//-----------------------------------------------------------------------------}

dxl_write_word(2, P_GOAL_POSITION_L, StandupPos[1]); //Abaixa o braco

for(int x=3; x<=8; x++)
    dxl_write_word( x, 34, 200); // Inicia os braços com baixo torque

//dxl_write_word(10, MOVING_SPEED, 30);
//dxl_write_word(16, MOVING_SPEED, 30);
robo_ereto();
     espera_mov();



//while(1){}

	//printf( "Press Enter key to continue!(press ESC and Enter to quit)\n" );
	//	if(getchar() == 0x1b)
	//		return 0;

while(1) //inicia o loop do programa main-------------------------------------
{

	//std::cout<<IMU_ACCEL_X<<std::endl;
	//std::cout<<IMU_ACCEL_Y<<std::endl;
	//std::cout<<IMU_ACCEL_Z<<std::endl;

unsigned int ServoBalanco1 = 11; //seleciona o servo da coxa para o treinamento
unsigned int ServoBalanco2 = 12; //seleciona o servo da coxa para o treinamento

//unsigned int ServoBalanco1 = 14; //seleciona o servo do calcanhar para o treinamento
//unsigned int ServoBalanco2 = 20; //seleciona o servo do calcanhar para o treinamento

	dxl_write_word(ServoBalanco1, MOVING_SPEED, 30);
	dxl_write_word(ServoBalanco2, MOVING_SPEED, 30);

	//dxl_write_word(10, P_GOAL_POSITION_L, dxl_read_word( 10, P_PRESENT_POSITION_L) + 80);
	//dxl_write_word(16, P_GOAL_POSITION_L, dxl_read_word( 16, P_PRESENT_POSITION_L) - 80);
     //espera_mov();

sleep(2);//aguarda 2 segundos a estabilização do robô

int estado, acao, estadoNovo, posiinit, cont=0, estadoServo;
bool atGoal = 0;
int reforco;
double best_new_qval, qval;
double  imu_med=0, imu_medx=0, imu_medz=0;
unsigned int passoObjetivo=0;
unsigned int Episodio=0;
int escolhe;

//dxl_read_word( ID, P_PRESENT_POSITION_L)

  /* initialize random seed: */
  srand (time(NULL));

//--------------------Inicie o programa abrindo o arquivo ou aleatorio---------------------------------
//printf("Abrindo arquivo.....\n");
passos = openFiletoGetQvalueVector(Episodio);//abre o arquivo salvo e passa os valores para Qvalue
//esta funcao inicializa os valores de Q com valores aleatorio
//init_qvalues();
//passos=0;
//-----------------------------------------------------------------------------------------------------
sleep(1);

//-----------------------------------------------------------------------------------------------

//--Ter uma posição fixa das posições do motor evita que o motor vá para posições indesejadas---
int posEstadoServo10[162]; //Vetor que guarda a posição fixa do servo
int posEstadoServo16[162]; //Vetor que guarda a posição fixa do servo
//-------------------------------------------------------------------

//-------Gera os valores dos servos correspondente aos estados---------
for(int x=0;x<161;x++)
{
	posEstadoServo10[x]= StandupPos[ServoBalanco1-1]-80+x;
	posEstadoServo16[x]= StandupPos[ServoBalanco2-1]+80-x;
	
}
//--------------------------------------------------------------------
/*
for(int x=0;x<61;x++)
{
	//Valores das heurísticas----------------------
	if(x<30)
	{
	    heuristic[x][0]=0;
	    heuristic[x][1]=10;
	}
	else
	{
	    heuristic[x][0]=10;
	    heuristic[x][1]=0;
	}
	//---------------------------------------------
}*/

int contador=0;

if (system("CLS")) system("clear");
// Carrega os CASOS para comparar ////////
printf("Carregando casos....espere.....\n\n");
le_arquivo_casos(); //carrega casos
printf("Casos Carregados !!!..............\n");
printf("**********************************\n\n");
sleep(2);
////////////////////PERFUMARIA /////////////////////////////////////////
if (system("CLS")) system("clear");
printf("Iniciando Aprendizado....(3)\n");
sleep(1);//aguarda X segundos a estabilização do robô
if (system("CLS")) system("clear");
printf("Iniciando Aprendizado....(2)\n");
sleep(1);//aguarda X segundos a estabilização do robô
if (system("CLS")) system("clear");
printf("Iniciando Aprendizado....(1)\n");
sleep(1);//aguarda X segundos a estabilização do robô
if (system("CLS")) system("clear");
printf("Iniciando Aprendizado....(0)\n");
sleep(1);//aguarda X segundos a estabilização do robô
if (system("CLS")) system("clear");
//////////////////////////////////////////////////////////////////////////
////
////
////
//================================================================================================
//---------------------------------Inicia o Aprendizado-------------------------------------------
do
{

	dxl_write_word(ServoBalanco1, MOVING_SPEED, 30); //define baixa velocidade ao servo
	dxl_write_word(ServoBalanco2, MOVING_SPEED, 30); //define baixa velocidade
	dxl_write_word(ServoBalanco1, MOVING_SPEED, 30); //define baixa velocidade
	dxl_write_word(ServoBalanco2, MOVING_SPEED, 30); //define baixa velocidade
     	
	//inicializa o robo em um estado aleatório aleatoria-------------------
     	estadoServo = (rand() % 161); // Randomicamente cai em um estado aleatorio

     
     //---------------------------------------------------------------------

     	dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]); //Manda o servo para estado
     	dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]); //Manda o servo para estado
     	dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]); //Manda o servo para estado
     	dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]); //Manda o servo para estado
     	espera_mov();

	//-----Calcula a média para filtrar ruído da IMU---------------------------------

	imu_med=filtro_Z();
	imu_medx=filtro_X();

	//-------------------------------------------------------------------------------

	std::cout<<"IMUZ = "<<imu_med<<"| IMUX = "<<imu_medx<<std::endl;


	//--- Verifica em que estado o agente se encontra neste momento-------------------

	estado=estadoROBO(imu_med,imu_medx);
	
	//-------------------------------------------------------------------------------
		

     	std::cout<<"\e[0;34m";
     	printf("Passos %d | Estado Escolhido %d | Estado do Servo = %d \n",passos, estado, estadoServo);
     	std::cout<<"\e[0m";
     	atGoal = 0;

     //----------------------------------------------------------------------


     	estadoNovo = estado;

	dxl_write_word(ServoBalanco1, MOVING_SPEED, 600);
	dxl_write_word(ServoBalanco2, MOVING_SPEED, 600);
     do
     {

	servo_funcionando();

        //vária de 0 a 1024 corresponde de 0 a 300 graus
        // cada valor do encoder = 0,29 graus.
        int decoderM=dxl_read_word( ServoBalanco1, P_PRESENT_POSITION_L); //peguei so um motor
	float anguloM=0;
	anguloM=decoderM*0,29;
        anguloM=robo_para_caso(anguloM);
	similaridade(anguloM,estadoNovo); //procura na base de casos


	acao = choose_best_transfer_action(estado); //Escolhe a acao
	
		
	if(acao == 0){if(estadoServo<160)estadoServo+=2;}
	else {if(estadoServo>0)estadoServo-=2;}
	

	dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]);
	dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]);
	dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]);
	dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]);

	if(passos%50==0){saveQ(qvalues, passos, Episodio);}

	espera_mov();
	sleep(1); //eu adicionei

	passos++;
	passoObjetivo++;

	//-----Calcula a média para filtrar ruído da IMU---------------------------------
	imu_med=filtro_Z();
	imu_medx=filtro_X();
	//-------------------------------------------------------------------------------

	//--- Verifica em que estado o agente se encontra neste momento-------------------

        estadoNovo= estadoROBO(imu_med,imu_medx);
	
	//-------------------------------------------------------------------------------

	//---Recebe reforco-----------------------------------------------------------------
	 reforco=0;
	 reforco=reforcoRobo(imu_med);

	if(reforco == 1000)
	{
	  atGoal = 1; saveQ(qvalues, passos, Episodio);saveEpisodios(Episodio, passoObjetivo);
	  passoObjetivo=0;Episodio++;printf("Salvando Dados !!!");
	}
	//-----------------------------------------------------------------------------------
	//---Atualiza valor do Q-------------------------------------------------------------
	best_new_qval = best_transfer_qvalue(estadoNovo);

	qval = qvalues[estado][acao];
	
	qvalues[estado][acao] = (1 - ALPHA)*qval + ALPHA*(reforco + GAMMA*best_new_qval);
	//-----------------------------------------------------------------------------------
 	
	std::cout<<"\e[0;31m"; 
	printf("acao escolhida %d | Reforco Ganho %d\n", acao,reforco);	
	std::cout<<"\e[0;34m";
     	printf("Passos %d Estado Escolhido %d | Estado do Novo = %d\n", passos,estado, estadoNovo);
     	std::cout<<"\e[0m";

	estado = estadoNovo;

         

     }while( atGoal == 0 );
    	printf("%d %d \n", cont, passos);
	cont++;

    
}while(1);

//printf("Alcancou %d objetivos\n", MAX_REACHED_GOALS);
//--------------------------------------Fim do Aprendizado--------------------------------------
//==============================================================================================
//espera_mov();
    sleep(2);
    }//fim do while loop do main

    // Close device
    dxl_terminate();
    printf( "Press Enter key to terminate...\n" );
    getchar();
    return 0;
}

//==============================================================================================
//--------------------Salva a tabel Q no arquivo------------------------------------------------
void saveQ(double qvalues[61][2], unsigned int passos, unsigned int Episodio)
{
    std::string separator = " "; // Use blank as default separator between single features
    std::fstream File;

    File.open("/home/fei/RoboFEI-HT/Qvalue.dat", std::ios::out);
    if (File.good() && File.is_open())
    {
        for (int i = 0; i < 61; ++i)
        {
            for (int j = 0; j < 2; ++j)
                    File << qvalues[i][j] << separator;
        }
	File << Episodio << separator;
	File << passos;
        File << std::endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}
//==============================================================================================
//--------------------Salva o numero de passos por Episodio-------------------------------------
void saveEpisodios(unsigned int Episodio, unsigned int passos)
{
    std::fstream File;

    File.open("/home/fei/RoboFEI-HT/Episodio.dat", std::ios::app | std::ios::out);
    if (File.good() && File.is_open())
    {
        File << Episodio <<" "<< passos;
        File << std::endl;
        File.flush();
        File.close();
    }
    else
	printf("Erro ao Salvar o arquivo\n");
}
//===============================================================================================
//-------------Abre o arquivo que contem --------------------------------------------------------
unsigned int openFiletoGetQvalueVector(unsigned int &Episodio)
{
    int passos=0;
    const char *fileName1;
    std::string fileName = "/home/fei/RoboFEI-HT/Qvalue.dat";
    fileName1 = fileName.c_str();

	std::ifstream File(fileName1);
	std::istream_iterator<float> start(File), end;


	std::vector<float> vectorTemp(start, end);
	int x=0;
        for (int i = 0; i < 122; i+=2)
        {
                qvalues[x][0] = vectorTemp[i];
                qvalues[x][1] = vectorTemp[i+1];
		x++;
        }	
	Episodio = vectorTemp[122];
	passos = vectorTemp[123];

	return passos;
}
//=================================================================================================================
//****************** CASOS ****************************************
//////// Manipulacao dos arquivos com casos

void le_arquivo_casos()
{
	
	int MAX_LEN =300;
	char linha[MAX_LEN];
        //char * linha = malloc(MAX_LEN+1);
	//char nome[300];
	char temp[300];
	char *p;
	int n_casos=0;
	
	FILE *arq;
	//arq=fopen("/home/fei/RoboFEI-HT/casos.txt", "r"); //
	arq=fopen("/home/fei/RoboFEI-HT/casos.txt", "r");
	fgets (linha, MAX_LEN, arq);
	///cout<<"saiu1"<<endl;
	
	while ( ! feof(arq) )
	//while(n_casos<100)
    	{
                //cout<<"-------"<<endl;
		p = linha;
		p = extraiDado (p, temp); //palavra numero:
		cout<<"numero "<<p<<endl;
		p = extraiDado (p, temp); //pega o P:
                cout<<"P: "<<p<<endl;
		//printf("%s ",temp);
		//p = extraiDado (p, temp); //pega P:
		//cout<<"P "<<p<<endl;
		p = extraiDado (p, temp); //pega 01		
		CASOS_tetha1[n_casos]=atof(temp);
		cout<<"Valor1: "<<p<<endl;	
	
		p = extraiDado (p, temp); //pega 02	
		CASOS_tetha2[n_casos]=atof(temp);
		cout<<"Valor2: "<<p<<endl;	
		
		p = extraiDado (p, temp); //pega A:
                cout<<"A: "<<p<<endl;

		p = extraiDado (p, temp); //pega a acao
		CASOS_acao[n_casos]=atoi(temp);
		cout<<"Acao: "<<p<<endl;
		
		p = extraiDado (p, temp);//pega R:
                cout<<"Pegou R: "<<p<<endl;

		p = extraiDado (p, temp);// pega o valor de R_AC
		CASOS_reward_acao[n_casos]=atof(temp);
		cout<<"Valor R: "<<p<<endl;
		
		//printf("l %d, Valor 1 %f, Valor 2 %f \n",n_casos, CASOS_tetha1[n_casos],CASOS_tetha2[n_casos]=atof(temp));
		
		fgets (linha, 255, arq);
		
		n_casos++;
		

	}
	//printf("Descarregando Arquivo\n")

	fclose(arq);

	CASOS_USADOS=n_casos;
	cout<<"Casos carregado----------"<<endl;
//Teste para ver se estava carregando corretamente os caso mesmo.	
	
	/*FILE *run;
	run = fopen("copia_casos_arquivados.txt", "w");
	
	for (int i = 0; i < CASOS_USADOS; ++i) 
	{
	fprintf(run,"caso: %d P: %f %f A: %d R_AC: %f \n", i,CASOS_tetha1[i],CASOS_tetha2[i],CASOS_acao[i],CASOS_reward_acao[i]) ;
	}

	fclose(run);*/
}

char *extraiDado (char *buffer, char *temp)
{
    int i=0;
	
    do {
	temp[i] = *buffer;
	buffer++; i++;
    } 
    while (*buffer != ' '); temp[i] = '\0';
		
    return ++buffer;
}

//FUNCAO RETORNA SE ESTA DE PE OU NAO //
bool empe(float sensor)
{
  if(sensor < -1.10){return true;}
	else return false;
}
//// FILTROS////////////////////////////
float filtro_Z()
{
	#define media_z 25
	float imu_somaz = 0;
	float temp_media = 0;
	
	for(int x=0; x<media_z;x++)
	{
		temp_media += IMU_ACCEL_Z;
		
		usleep(4000);
	}
	imu_somaz  = temp_media / media_z;

	return imu_somaz;

}

float filtro_X()
{
	#define media_x 25
	float imu_somax = 0;
	float temp_media = 0;
	for(int x=0; x<media_x;x++)
	{
		temp_media += IMU_ACCEL_X;
		usleep(4000);
	}
	imu_somax  = temp_media / media_x;

	return imu_somax;
}

////////REFORCO/////
int reforcoRobo(float sensor)
{
        int comparacao;
   	int reforco=0;
        int valor=-1.1;

        comparacao=(sensor/valor)*100;

        if(comparacao>95){reforco = +1000;}
        else {reforco = -1;}
 
	return reforco;
}
///////////////////////ESTADO //////////////////////////////////////////////////////////////////////////

int estadoROBO(float sensorZ,float sensorX) 
{
  int estado=0;
 

//--- Verifica em que estado o agente se encontra neste momento-------------------
	if(sensorZ>=posAccel[0])
	{
	     if(sensorX>=0) //verifica se o robô está pendendo para frente ou para traz
	         estado = 0;
	     else
	         estado = 60;
	}
	else
	{
	     if(sensorZ<posAccel[30])
		estado = 30;
	     else
	     {
	         if(sensorX>=0)//verifica se o robô está pendendo para frente ou para traz
		 {
		    for(int x=1; x<30;x++)
		    {
		      if(sensorZ<posAccel[x] && sensorZ>=posAccel[x+1])
			   estado = x;
		      printf("Estado[%d]= %f | Estado[%d]=%f\n", x, posAccel[x],x+1, posAccel[x+1] );
		    }
		 }
		 else
		 {
		    for(int x=1; x<30;x++)
		    {
		      if(sensorZ<posAccel[x] && sensorZ>=posAccel[x+1])
			   estado = 60-x;
		      printf("Estado[%d]= %f | Estado[%d]=%f\n", 60-x, posAccel[x],60-x+1, posAccel[x+1] );
		    }
		 }
	     }
	}



	return estado;

}

 void similaridade(float AnguloQuadril, int estado)
{

	double fatorsimilaridade = 0.5;
	double valorSimilaridade = 0;
	int valorAcao = 0;

	//CASOS_tetha2[i] = valor quadril
   

  for (int i = 0; i < CASOS_USADOS; ++i)
	{
		//distancia local generica.
		valorSimilaridade = CASOS_tetha2[i]- AnguloQuadril;

                 if(valorSimilaridade < fatorsimilaridade)
			{valorAcao=CASOS_acao[i];}

	}

	if(valorAcao==1)heuristic[estado][0]=1000;
	else if(valorAcao==2)heuristic[estado][1]=1000;
	//else if(valorAcao==2)heuristic[estado][0]=1;  //arrumar

}

//************ Algoritmo de Caso para Robo *****************************
float caso_para_Robo(float valor)
{
   float saida;
   saida=(150*valor)/90;
   return saida;
}
//************ Algoritmo de Robo para Caso *********************************

float robo_para_caso(float valor)
{
   float saida;
   saida=(90*valor)/150;
   return saida;
}

void servo_funcionando() //verifica se esta funcionando
{
	if(dxl_read_word( 11, 34 ) == 0) //verifica se o servo zerou o torque desligando o servo
	{
		dxl_write_word(2, MOVING_SPEED, 80);
		dxl_write_word(2, P_GOAL_POSITION_L, 0); //Levanta o braco pedindo ajuda
		cout<<"Houve falha no Servo 11 - O servo desligou o torque"<<endl;
		exit(0);
	}
	if(dxl_read_word( 12, 34 ) == 0) //verifica se o servo zerou o torque desligando o servo
	{
		dxl_write_word(2, MOVING_SPEED, 80);
		dxl_write_word(2, P_GOAL_POSITION_L, 0); //Levanta o braco pedindo ajuda
		cout<<"Houve falha no Servo 12 - O servo desligou o torque"<<endl;
		exit(0);
	}
}






