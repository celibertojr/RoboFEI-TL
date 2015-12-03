#include <cstdlib>
#include "ql.h"
#include <stdio.h>
#include <time.h>


void init_qvalues()
{
  int r,c,a;

  for (r=0;r<= 60;r++)
	for (a=0;a<=1;a++)
	  {
	    qvalues[r][a] = randValue(rl_rand);
	    //printf("Qvalue[%d][%d] = %f \n",r, a, qvalues[r][a]);
	  }
}


double best_qvalue(int state)
{
	double best_val = -1000000.0;
	int a;
	
        for (a=0;a<=1;a++)
	{
	      if (qvalues[state][a] > best_val)
	      {
		  best_val = qvalues[state][a];
	      }
	}
	return (best_val);
}



int best_qvalue_action(int state)
{
	double best_val = -100000.0;
	int act,a;

	for (a=0;a<=1;a++)
	{
	    if ((qvalues[state][a]+heuristic[state][a]) > best_val)
	    {
		    best_val = qvalues[state][a]+heuristic[state][a];
		    act = a;
	    }
	}
	return (act);
}

int choose_best_action(int state)
{
  int act;

	//printf("R = %f\n",rvalue);
  if (randValue(rl_rand) < EXPLORATION_THRESHOLD)
    act = disac(rl_rand);
  else 
    act = best_qvalue_action(state);
  return (act);
} 

//// TRANFERENCIA de aprendizado /////////////////////////////


double best_transfer_qvalue(int state)
{
	double best_val = -1000000.0;
	int a;
	
    for (a=0;a<=1;a++)
	{
	      if (qvalues[state][a] > best_val)
	      {
		  best_val = qvalues[state][a];
	      }
	}
	return (best_val);
}



int best_qvalue_transfer_action(int state)
{
	double best_val = -100000.0;
	int act,a;

	for (a=0;a<=1;a++)
	  {
	    if ((qvalues[state][a]+heuristic[state][a]) > best_val)
	      {
		best_val = qvalues[state][a]+heuristic[state][a];
		act = a;
	      }
	  }
	return (act);
}


int choose_best_transfer_action(int state)
{
  double rvalue;
  double curr_val,best_val = -1000000.0;
   
  int a,act;

  rvalue = (rand() % 1000)/(double)1000;
	//printf("R = %f\n",rvalue);
  if (rvalue < EXPLORATION_THRESHOLD)
    act = rand() % 2;
  else 
    act = best_qvalue_transfer_action(state);
  return (act);
} 


