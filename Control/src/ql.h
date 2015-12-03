
#include <random>

#define ITERATIONS 10000000
#define MAX_REACHED_GOALS 10000

#define TRIALS 3

#ifndef RAND_MAX
#define RAND_MAX 2147483000
#endif

#define MBIG 1000000000
#define MSEED 161803398
#define MZ 0
#define FAC (1.0/ MBIG)

#define GAMMA 0.99 /* fator de desconto = 90% */
#define ALPHA 0.2 /* taxa de aprendizado = 10% */
#define EXPLORATION_THRESHOLD 0.2 /* porcentagem de exploração = 05% */

std::mt19937 rl_rand(time(0));
std::uniform_real_distribution<double> randValue(0.00, 1.00);
std::uniform_int_distribution<> disac(0, 1);


double qvalues[121][2];

double heuristic[121][2];

void init_qvalues();

double best_qvalue(int state);

int best_qvalue_action(int state);

int choose_best_action(int state);

//transfer
double best_transfer_qvalue(int state);
int best_qvalue_transfer_action(int state);
int choose_best_transfer_action(int state);

