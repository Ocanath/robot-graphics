#ifndef FINGER_KINEMATICS_H
#define FINGER_KINEMATICS_H
#include "stdint.h"
#include "kinematics.h"

enum {REPEL, STOP, ATTRACT, NEUTRAL};	//concept for flag setting of conflict avoidance subroutine


typedef struct kinematic_finger_t
{
	joint chain[3];
	vect3 ef_pos_b;
}kinematic_finger_t;

typedef struct kinematic_hand_t
{
	kinematic_finger_t finger[5];
}kinematic_hand_t;

void finger_kinematics(kinematic_hand_t * kh);
void init_finger_kinematics(kinematic_hand_t * kh);
float get_4bar_driven_angle(float q1);

//void htmatrix_vect3_mult(mat4_t * m, vect3_t * v, vect3_t * ret);
void htmatrix_vect3_mult(mat4* m, vect3* v, vect3* ret);

/*
Procedure:
1. after new positions are loaded, set all the fk q values
2. 
*/

#endif
