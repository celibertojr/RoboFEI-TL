#include <dynamixel.h>

void robo_ereto()
{
    for(int x=3; x<=8; x++)
       dxl_write_word( x, 34, 1023); // Seta o torque dos braços no máximo

    for(int i = 3;i<21;i++)
    {
       dxl_write_word(i, MOVING_SPEED, 60);
       dxl_write_word(i, P_GOAL_POSITION_L, StandupPos[i-1]);
    }
    inverse_kinematic_right_leg(0, 0, 0, 100);
    inverse_kinematic_left_leg(0, 0, 0, 100);

    for(int x=3; x<=8; x++)
       dxl_write_word( x, 34, 200); // Seta os braços com baixo torque
}
//================================================================================
