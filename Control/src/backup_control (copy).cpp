/*--------------------------------------------------------------------

******************************************************************************
* @file control.cpp
* @author Isaac Jesus da Silva - ROBOFEI-HT - FEI
* @version V0.0.7
* @created 20/01/2014
* @Modified 16/07/2014
* @e-mail isaac25silva@yahoo.com.br
* @brief control
****************************************************************************
****************************************************************************

Arquivo fonte contendo o programa que controla os servos do corpo do robô

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

//void PrintCommStatus(int CommStatus);
//void PrintErrorCode(void);

//////global variables////////////////////////////////////////////////
int inclina = 55;
unsigned int StandupPos[22] = {803, 512, //01 , 02, (cabeça)
                382, 417, 353+20, //03, 04, 05, (braço direito)
                472+170, 600, 689-20, //06, 07, 08, (braço esquerdo) 472
                512, 512+20, 512-5, 591-10, 512, 801+5-3, //09, 10, 11, 12, 13, 14 (perna direita)
               512, 512-25, 512+20, 492, 512, 224-5+3}; //15, 16, 17, 18, 19, 20 (perna esquerda)

/*
unsigned int StandupPos[22] = {803, 512, //01 , 02, (cabeça)
                545, 417, 335, //03, 04, 05, (braço direito)
                472, 600, 682, //06, 07, 08, (braço esquerdo)
                512, 512, 512, 591, 512, 791+5+5, //09, 10, 11, 12, 13, 14 (perna direita)
               512, 512, 535-10, 492, 512, 203+5+11+5}; //15, 16, 17, 18, 19, 20 (perna esquerda)
*/

int main(int argc, char* argv[])
{

    int * baudnum = new int; //alocado dinâmicamente
    *baudnum = DEFAULT_BAUDNUM;  //velocidade de transmissao da serial em 1Mbps
    int * deviceIndex = new int; //alocado dinâmicamente
    *deviceIndex = 0; 		//endereça USB
    unsigned int tensaomedia = 0;

    using_shared_memory();

    char string[50]; //String usada para definir prioridade do programa

    //system("echo fei 123456| sudo -S chmod 777 /dev/ttyUSB*");//libera acesso a USB

    sprintf(string,"echo fei 123456 | sudo -S renice -20 -p %d", getpid()); // prioridade maxima do codigo
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
			servoConectado = dxl_read_byte( 20, 3 ) == 20;
			usleep(1000);
			servoConectado = dxl_read_byte( 20, 3 ) == 20;//Tenta novamente caso falhe a comunicação
			usleep(1000);
			servoConectado = dxl_read_byte( 20, 3 ) == 20;//Tenta novamente caso falhe a comunicação
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

dxl_write_word(10, MOVING_SPEED, 30);
dxl_write_word(16, MOVING_SPEED, 30);
robo_ereto();
     espera_mov();

	//printf( "Press Enter key to continue!(press ESC and Enter to quit)\n" );
	//	if(getchar() == 0x1b)
	//		return 0;

	while(1)
	{


//std::cout<<IMU_ACCEL_X<<std::endl;
//std::cout<<IMU_ACCEL_Y<<std::endl;
//std::cout<<IMU_ACCEL_Z<<std::endl;

dxl_write_word(10, MOVING_SPEED, 30);
dxl_write_word(16, MOVING_SPEED, 30);

		//dxl_write_word(10, P_GOAL_POSITION_L, dxl_read_word( 10, P_PRESENT_POSITION_L) + 80);
		//dxl_write_word(16, P_GOAL_POSITION_L, dxl_read_word( 16, P_PRESENT_POSITION_L) - 80);
     //espera_mov();

//sleep(3);

int estado, acao, estadoNovo, posiinit, cont=0, reforco, passos=0;
bool atGoal = 0;
double best_new_qval, qval;

//esta funcao inicializa os valores de Q com valores aleatorio
init_qvalues();

do
{
     //inicializa o robo em uma posicao aleatoria
     posiinit = (rand() % 161);
     printf("posi %d \n", posiinit);
     dxl_write_word(10, P_GOAL_POSITION_L, StandupPos[9]-(posiinit-80));
     dxl_write_word(16, P_GOAL_POSITION_L, StandupPos[15]+(posiinit-80));
     espera_mov();
     atGoal = 0;
     passos = 0;

     do
     {

     	estado = ((StandupPos[9] - dxl_read_word( 10, P_PRESENT_POSITION_L)))+80;
	//std::cout<<"estado = "<<estado<<std::endl;
	//std::cout<<"Stand = "<<StandupPos[9]<<std::endl;
	//std::cout<<"dxl = "<<dxl_read_word( 10, P_PRESENT_POSITION_L)<<std::endl;
     	acao = best_qvalue_action(estado);
	printf("acao = %d\n",acao);
	std::cout<<"estado = "<<estado<<std::endl;
	if(estado < 0)
		estado = 1;
	if(estado > 160)
		estado =159;

	std::cout<<"estado escolhido = "<<estado<<std::endl;
	//executou a acao		

	if(acao == 0 && estado > 0 )
	{

		dxl_write_word(10, P_GOAL_POSITION_L, dxl_read_word( 10, P_PRESENT_POSITION_L) - 1);
		dxl_write_word(16, P_GOAL_POSITION_L, dxl_read_word( 16, P_PRESENT_POSITION_L) + 1);
	}
	else
	{
		if(estado < 160)
		{
		dxl_write_word(10, P_GOAL_POSITION_L, dxl_read_word( 10, P_PRESENT_POSITION_L) + 1);
		dxl_write_word(16, P_GOAL_POSITION_L, dxl_read_word( 16, P_PRESENT_POSITION_L) - 1);
		}
	}
	espera_mov();

	passos++;


	//descobriu o novo estado
     	estadoNovo = ((StandupPos[9] - dxl_read_word( 10, P_PRESENT_POSITION_L)))+80;
	if(estadoNovo < 0)
		estadoNovo = 0;
	if(estadoNovo > 160)
		estadoNovo =160;

	//-----Calcula a média para filtrar ruído da IMU--------
	float imu_soma = 0, imu_med;
	for(int x=0; x<20;x++)
	{
		imu_soma += IMU_ACCEL_Z;
		usleep(5000);
	}
	imu_med  = imu_soma / 20;
	//------------------------------------------------------


	//---Recebe reforco-----------------------
	if( imu_med < -1 )
	{
		reforco = 100;
		atGoal = 1;
	}
	else
		reforco = -1;
	//----------------------------------------


	//---Atualiza valor do Q-------------------------------------------------------------
	best_new_qval = best_qvalue(estadoNovo);

	qval = qvalues[estado][acao];
	
	qvalues[estado][acao] = (1 - ALPHA)*qval + ALPHA*(reforco + GAMMA*best_new_qval);
	//-----------------------------------------------------------------------------------
 

     }while( atGoal == 0 );
    printf("%d %d \n", cont, passos);

     cont++;
}while(cont < MAX_REACHED_GOALS);

//espera_mov();
sleep(2);


    }
	// Close device
	dxl_terminate();
	printf( "Press Enter key to terminate...\n" );
	getchar();
	return 0;
}

//=================================================================================================================
