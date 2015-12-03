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

//void PrintCommStatus(int CommStatus);
//void PrintErrorCode(void);

using namespace std;

void saveQ(double qvalues[61][2], unsigned int passos, unsigned int Episodio);

void saveEpisodios(unsigned int Episodio, unsigned int passos);

unsigned int openFiletoGetQvalueVector(unsigned int &Episodio);

void servo_funcionando();

//////global variables////////////////////////////////////////////////
int inclina = 55;
unsigned int StandupPos[22] =  {842,212+170,457,622,022,987,517,532,517,592,535,482,522,442,542,517,602,512,1024,792};

//----Vetor que discretiza a posição fixa do acelerometro refente aos estados--------------------
float posAccel[31] = {-0.84, -0.845, -0.85, -0.855, -0.86, -0.865, -0.87, -0.875, -0.88, -0.89,
		   -0.90, -0.91, -0.92, -0.93, -0.94, -0.95, -0.96, -0.97, -0.98, -0.99,
		   -1.00, -1.01, -1.02, -1.03, -1.04, -1.05, -1.06, -1.07, -1.08, -1.09, -1.10};


using namespace std;

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
				    printf("Não encontrou nenhuma placa do servo conectada a porta USB\n");
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
double reforco;
double best_new_qval, qval;
double  imu_med=0, imu_medx=0;
unsigned int passoObjetivo=0;
unsigned int Episodio=0;
int escolhe;

  /* initialize random seed: */
  srand (time(NULL));

//--------------------Inicie o programa abrindo o arquivo ou aleatorio---------------------------------
passos = openFiletoGetQvalueVector(Episodio);//abre o arquivo salvo e passa os valores para Qvalue
//esta funcao inicializa os valores de Q com valores aleatorio
//init_qvalues();
//passos=0;
//-----------------------------------------------------------------------------------------------------


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
	//printf("Servo10Pos[%d] = %d | ", x, posEstadoServo10[x]);
	//printf("Servo16Pos[%d] = %d\n", x, posEstadoServo16[x]);
}
//--------------------------------------------------------------------

for(int x=0;x<61;x++)
{
	//Valores das heurísticas----------------------
	if(x<30)
	{
	    heuristic[x][0]=0;
	    heuristic[x][1]=0;
	}
	else
	{
	    heuristic[x][0]=0;
	    heuristic[x][1]=0;
	}
	//---------------------------------------------
}

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

     //escolhe = (rand() % 2);
    // printf("escolhe = %d",escolhe);
     //if(escolhe)
	//estadoServo = 170;
     //else
	//estadoServo = 0;
     //---------------------------------------------------------------------

     dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]); //Manda o servo para estado
     dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]); //Manda o servo para estado
     dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]); //Manda o servo para estado
     dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]); //Manda o servo para estado
     espera_mov();

	//-----Calcula a média para filtrar ruído da IMU---------------------------------
	float imu_soma = 0;
	float imu_somax = 0;
	for(int x=0; x<25;x++)
	{
		imu_soma += IMU_ACCEL_Z;
		imu_somax += IMU_ACCEL_X;
		usleep(4000);
	}
	imu_med  = imu_soma / 25;
	imu_medx  = imu_somax / 25;
	//-------------------------------------------------------------------------------

	std::cout<<"IMUZ = "<<imu_med<<"| IMUX = "<<imu_medx<<std::endl;

	//--- Verifica em que estado o agente se encontra neste momento-------------------
	if(imu_med>=posAccel[0])
	{
	     if(imu_medx>=0) //verifica se o robô está pendendo para frente ou para traz
	         estado = 0;
	     else
	         estado = 60;
	}
	else
	{
	     if(imu_med<posAccel[30])
		estado = 30;
	     else
	     {
	         if(imu_medx>=0)//verifica se o robô está pendendo para frente ou para traz
		 {
		    for(int x=1; x<30;x++)
		    {
		      if(imu_med<posAccel[x] && imu_med>=posAccel[x+1])
			   estado = x;
		      //printf("Estado[%d]= %f | Estado[%d]=%f\n", x, posAccel[x],x+1, posAccel[x+1] );
		    }
		 }
		 else
		 {
		    for(int x=1; x<30;x++)
		    {
		      if(imu_med<posAccel[x] && imu_med>=posAccel[x+1])
			   estado = 60-x;
		      //printf("Estado[%d]= %f | Estado[%d]=%f\n", 60-x, posAccel[x],60-x+1, posAccel[x+1] );
		    }
		 }
	     }
	}
	//-------------------------------------------------------------------------------
		

     std::cout<<"\e[0;34m";
     printf("Estado Escolhido %d | Estado do Servo = %d\n", estado, estadoServo);
     std::cout<<"\e[0m";
     atGoal = 0;

     //----------------------------------------------------------------------

//while(1){}
     estadoNovo = estado;

	dxl_write_word(ServoBalanco1, MOVING_SPEED, 600);
	dxl_write_word(ServoBalanco2, MOVING_SPEED, 600);
     do
     {
        servo_funcionando();
	//std::cout<<"estado = "<<estado<<std::endl;
	//std::cout<<"Stand = "<<StandupPos[9]<<std::endl;
	//std::cout<<"dxl = "<<dxl_read_word( 10, P_PRESENT_POSITION_L)<<std::endl;
     	//acao = best_qvalue_action(estado);
	acao = choose_best_action(estado);
	//printf("acao = %d\n",acao);
	//std::cout<<"estado = "<<estado<<std::endl;
	//if(estado < 0)
		//estado = 1;
	//if(estado > 170)
		//estado =169;
	if(acao==0)
	    printf("estado escolhido = %d | *Qacao0 = %2.4f | Qacao1 = %2.4f | IMU = %1.3f\n", estadoNovo, qvalues[estado][0], 			qvalues[estado][1], imu_med);
	else
	    printf("estado escolhido = %d | Qacao0 = %2.4f | *Qacao1 = %2.4f | IMU = %1.3f\n", estadoNovo, qvalues[estado][0], 			qvalues[estado][1], imu_med);
	//std::cout<<"estado escolhido = "<<estadoNovo<<" | Qacao0 = "<<qvalues[estado][0];
	//std::cout<<"Qacao1 = "<<qvalues[estado][1]<<std::endl;
	//executou a acao		

	if(acao == 0)
	{
		if(estadoServo<160)
			estadoServo+=2;
	}
	else
	{
		if(estadoServo>0)
			estadoServo-=2;
	}
	

	dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]);
	dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]);
	dxl_write_word(ServoBalanco1, P_GOAL_POSITION_L, posEstadoServo10[estadoServo]);
	dxl_write_word(ServoBalanco2, P_GOAL_POSITION_L, posEstadoServo16[estadoServo]);

	if(passos%50==0)
	    saveQ(qvalues, passos, Episodio);
	espera_mov();

	passos++;
	passoObjetivo++;


	//descobriu o novo estado

	//if(estadoNovo < 0)
		//estadoNovo = 0;
	//if(estadoNovo > 160)
		//estadoNovo =160;

	//-----Calcula a média para filtrar ruído da IMU---------------------------------
	imu_soma = 0;
	imu_somax = 0;
	for(int x=0; x<25;x++)
	{
		imu_soma += IMU_ACCEL_Z;
		imu_somax += IMU_ACCEL_X;
		usleep(4000);
	}
	imu_med  = imu_soma / 25;
	imu_medx  = imu_somax / 25;
	//-------------------------------------------------------------------------------

	//--- Verifica em que estado o agente se encontra neste momento-------------------
	if(imu_med>=posAccel[0])
	{
	     if(imu_medx>=0)
	         estadoNovo = 0;
	     else
	         estadoNovo = 60;
	}
	else
	{
	     if(imu_med<posAccel[30])
		estadoNovo = 30;
	     else
	     {
	         if(imu_medx>=0)
		 {
		    for(int x=1; x<30;x++)
		    {
		      if(imu_med<posAccel[x] && imu_med>=posAccel[x+1])
			   estadoNovo = x;
		      //printf("Estado[%d]= %f | Estado[%d]=%f\n", x, posAccel[x],x+1, posAccel[x+1] );
		    }
		 }
		 else
		 {
		    for(int x=1; x<30;x++)
		    {
		      if(imu_med<posAccel[x] && imu_med>=posAccel[x+1])
			   estadoNovo = 60-x;
		      //printf("Estado[%d]= %f | Estado[%d]=%f\n", 60-x, posAccel[x],60-x+1, posAccel[x+1] );
		    }
		 }
	     }
	}
	//-------------------------------------------------------------------------------

	//---Recebe reforco-----------------------------------------------------------------
	if( imu_med < -1.025)
	{

		//-----Calcula a média novamente para verificar se é um falso positivo
		float imu_soma = 0;
		for(int x=0; x<32;x++)
		{
			imu_soma += IMU_ACCEL_Z;
			usleep(5000);
		}
		imu_med  = imu_soma / 32;
		//-------------------------------------------------------


		//---Entra para recebe reforco positivo----------------------
		if( imu_med < -1.020 )
		{
			printf("Objetivo alcancado estado %d| passos = %d\n", estadoNovo, passoObjetivo);
			reforco = 1000;
			atGoal = 1;
			saveQ(qvalues, passos, Episodio);
			saveEpisodios(Episodio, passoObjetivo);
			passoObjetivo=0;
			Episodio++;
		}
		else
			reforco = -1;
	}
	else
		reforco = -1;
	//-----------------------------------------------------------------------------------

	//---Atualiza valor do Q-------------------------------------------------------------
	best_new_qval = best_qvalue(estadoNovo);

	qval = qvalues[estado][acao];
	
	qvalues[estado][acao] = (1 - ALPHA)*qval + ALPHA*(reforco + GAMMA*best_new_qval);
	//-----------------------------------------------------------------------------------
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

    File.open("Qvalue.dat", std::ios::out);
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

    File.open("Episodio.dat", std::ios::app | std::ios::out);
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
    std::string fileName = "Qvalue.dat";
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

