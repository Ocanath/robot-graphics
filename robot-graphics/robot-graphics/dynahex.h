#ifndef DYNAHEX_H
#define DYNAHEX_H
#include "kinematics.h"

/*
* Definition of the hexleg in C style
*/

#define NUM_FRAMES_HEXLEG 4	//0,1,2,3
#define NUM_JOINTS_HEXLEG 3
#define NUM_LEGS	6
typedef struct dynahexleg_t
{
	joint chain[NUM_FRAMES_HEXLEG];
	vect3 ef_0;
	vect3 ef_b;
	vect3 ef_knee;	//some example anchor points associated with each leg
}dynahexleg_t;

typedef struct dynahex_t
{
	//mat4 hw_b;
	dynahexleg_t leg[NUM_LEGS];	
}dynahex_t;

void init_dh_kinematics(dynahex_t* h);
void forward_kinematics_dynahexleg(dynahex_t* h);

#endif // !DYNAHEX_H