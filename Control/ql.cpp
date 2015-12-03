#include <cstdlib>
#include "ql.h"

void init_qvalues()
{
  int r,c,a;
  
  for (r=0;r<= 80;r++)
	for (a=0;a<=1;a++)
	  {
	    qvalues[r][a] = (rand() % 100)/100;
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
	    if (qvalues[state][a] > best_val)
	      {
		best_val = qvalues[state][a];
		act = a;
	      }
	  }
	return (act);
}

int choose_best_action(int state)
{
  double rvalue;
  double curr_val,best_val = -1000000.0;
   
  int a,act;

  rvalue = (rand() % 100)/100;

  if (rvalue > EXPLORATION_THRESHOLD)
    act = rand() % 2)
  else 
    act = best_qvalue_action(state);
  return (act);
} 
