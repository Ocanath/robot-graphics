#include "kinematics.h"
#include <stdio.h>
#include <stdlib.h>
#include <math.h>

///*
//load a const q, dh table
//*/
//void init_chain(kinematic_chain * chain, const dh_entry * table, const float * q_init, int num_frames)
//{
//	chain->num_frames = num_frames;
//	int i;
//	for (i = 1; i <= chain->num_frames; i++)
//	{
//		chain->j[i].dh_params = table[i];
//		chain->j[i].q = q_init[i];
//		chain->j[i].dq = 0;
//		chain->j[i].ddq = 0;
//	}
//	chain->hb_0 = mat4_I();
//	chain->hw_b = mat4_I();
//	init_forward_kinematics(chain);
//	forward_kinematics(chain);
//}

/*
	C++ PC ONLY function to pre-allocate memory for a kinematic chain.
	THIS IS NOT TO BE USED ON AN EMBEDDED SYSTEM. probably won't compile anyway
	to quote stackoverflow: 
		
		"The need of using malloc on such systems typically originates
		from confused PC programmers who have picked up embedded programming
		without studying it first."
*/
//void chain_pre_allocate(kinematic_chain * chain, int num_frames)
//{
//	chain->num_frames = num_frames;
//	chain->j = new joint[num_frames];
//}
//void chain_de_allocate(kinematic_chain * chain)
//{
//	delete chain->j;
//}

/*
load the kinematic chain Si, h0_i, him1_i variables based on the dh_table.
NOTE: only done ONCE, as the forward kinematics function only loads the elements which change with q

*/
void init_forward_kinematics(kinematic_chain * chain)
{
	chain->j[0].h0_i = mat4_I();
	chain->j[0].him1_i = mat4_I();

	int i;
	for (i = 1; i < chain->num_frames; i++)
	{
		//[row][column]
		chain->j[i].him1_i.m[0][0] = cos(chain->j[i].dh_params.theta);	chain->j[i].him1_i.m[0][1] = -1.0f*sin(chain->j[i].dh_params.theta)*cos(chain->j[i].dh_params.alpha);	chain->j[i].him1_i.m[0][2] = sin(chain->j[i].dh_params.theta)*sin(chain->j[i].dh_params.alpha); 		chain->j[i].him1_i.m[0][3] = chain->j[i].dh_params.a*cos(chain->j[i].dh_params.theta);
		chain->j[i].him1_i.m[1][0] = sin(chain->j[i].dh_params.theta);	chain->j[i].him1_i.m[1][1] = cos(chain->j[i].dh_params.theta)*cos(chain->j[i].dh_params.alpha);			chain->j[i].him1_i.m[1][2] = -1.0f*cos(chain->j[i].dh_params.theta)*sin(chain->j[i].dh_params.alpha);	chain->j[i].him1_i.m[1][3] = chain->j[i].dh_params.a*sin(chain->j[i].dh_params.theta);
		chain->j[i].him1_i.m[2][0] = 0;									chain->j[i].him1_i.m[2][1] = sin(chain->j[i].dh_params.alpha);											chain->j[i].him1_i.m[2][2] = cos(chain->j[i].dh_params.alpha);											chain->j[i].him1_i.m[2][3] = chain->j[i].dh_params.d;
		chain->j[i].him1_i.m[3][0] = 0;									chain->j[i].him1_i.m[3][1] = 0;																			chain->j[i].him1_i.m[3][2] = 0;																			chain->j[i].him1_i.m[3][3] = 1.0f;
	}
	//copy_HT(&(HTbase[1]), &(HTadj[1]));
	chain->j[1].h0_i = chain->j[1].him1_i;
	for (i = 2; i < chain->num_frames; i++)
		chain->j[i].h0_i = mat4_mult(chain->j[i-1].h0_i, chain->j[i].him1_i);
}

/*
calculate the forward kinematics of chain, pass by reference-pointer
*/
void forward_kinematics(kinematic_chain * chain)
{
	int i;
	for (i = 1; i < chain->num_frames; i++)
	{
		chain->j[i].dh_params.theta = chain->j[i].q;
		//update all variable entries (only rotational joints for this framework, although uncommenting the d line below will allow for FK for prismatic joints. 
		chain->j[i].him1_i.m[0][0] = cos(chain->j[i].dh_params.theta);		chain->j[i].him1_i.m[0][1] = -sin(chain->j[i].dh_params.theta)*cos(chain->j[i].dh_params.alpha);			chain->j[i].him1_i.m[0][2] = sin(chain->j[i].dh_params.theta)*sin(chain->j[i].dh_params.alpha); 		chain->j[i].him1_i.m[0][3] = chain->j[i].dh_params.a*cos(chain->j[i].dh_params.theta);
		chain->j[i].him1_i.m[1][0] = sin(chain->j[i].dh_params.theta);		chain->j[i].him1_i.m[1][1] = cos(chain->j[i].dh_params.theta)*cos(chain->j[i].dh_params.alpha);				chain->j[i].him1_i.m[1][2] = -cos(chain->j[i].dh_params.theta)*sin(chain->j[i].dh_params.alpha);		chain->j[i].him1_i.m[1][3] = chain->j[i].dh_params.a*sin(chain->j[i].dh_params.theta);
		//chain->j[i].him1_i.m[2][3] = chain->j[i].dh_params.d;
	}
	chain->j[1].h0_i = chain->j[1].him1_i;
	for (i = 2; i < chain->num_frames; i++)
		chain->j[i].h0_i = mat4_mult(chain->j[i - 1].h0_i, chain->j[i].him1_i);
}

void calc_J_point(kinematic_chain * chain, vect3 point)
{
	int i, v_idx;
	vect3 z;
	vect3 d;
	for (i = 1; i < chain->num_frames; i++)
	{
		//h0_i[0] = I
		for (v_idx = 0; v_idx < 3; v_idx++)
			z.v[v_idx] = chain->j[i - 1].h0_i.m[v_idx][2];					//extract the unit vector corresponding to the axis of rotation of frame i-1 (i.e., the axis of rotation of q)
		for (v_idx = 0; v_idx < 3; v_idx++)	
			d.v[v_idx] = point.v[v_idx] - chain->j[i - 1].h0_i.m[v_idx][3];	//extract the difference between the target point in frame 0 and the origin of frame i-1
		vect3 res = cross(z, d);
			
		chain->j[i].Si.v[0] = z.v[0];		//Si = [w, v]^T; 
		chain->j[i].Si.v[1] = z.v[1];		//z_im1*q = w
		chain->j[i].Si.v[2] = z.v[2];

		chain->j[i].Si.v[3] = res.v[0];	//(z_im1 x (p - o_im1))*q = v
		chain->j[i].Si.v[4] = res.v[1];
		chain->j[i].Si.v[5] = res.v[2];
	}

}

vect6 calc_w_v(kinematic_chain * chain, vect3 * w, vect3 * v)
{
	vect6 ret;
	int i;
	for (i = 0; i < 6; i++)
		ret.v[i] = 0;			//initialize the sum
	for (i = 1; i < chain->num_frames; i++)
		ret = vect6_add(ret, vect6_scale(chain->j[i].Si, chain->j[i].dq));
	w->v[0] = ret.v[0];
	w->v[1] = ret.v[1];
	w->v[2] = ret.v[2];
	v->v[0] = ret.v[3];
	v->v[1] = ret.v[4];
	v->v[2] = ret.v[5];
	return ret;
}

void calc_tau(kinematic_chain * chain, vect6 f, float * tau)
{
	int i;
	for (i = 1; i < chain->num_frames; i++)
		tau[i] = vect6_dot(chain->j[i].Si, f);
}

/*
helper function for extracting the origin of a ht matrix
*/
vect3 h_origin(mat4 h)
{
	vect3 ret;
	int i;
	for (i = 0; i < 3; i++)
		ret.v[i] = h.m[i][3];
	return ret;
}
/*
helper function for extracting the origin of a ht matrix 
in the form of a 4 vector
*/
vect4 h_origin_vect4(mat4 h)
{
	vect4 ret;
	int i;
	for (i = 0; i < 3; i++)
		ret.v[i] = h.m[i][3];
	ret.v[3] = 1;
	return ret;
}

/*
* TODO TEST THIS FUNCTION
* Transforms a NORMALIZED quaternion to a rotation matrix
*/
mat4 quat_to_mat4(vect4 quat, vect3 origin)
{
	mat4 m;
	float q1 = quat.v[0];
	float q2 = quat.v[1];
	float q3 = quat.v[2];
	float q4 = quat.v[3];

	float qq1 = q1*q1;
	float qq2 = q2*q2;
	float qq3 = q3*q3;
	float qq4 = q4*q4;

	float q2q3 = q2*q3;
	float q1q2 = q1*q2;
	float q1q3 = q1*q3;
	float q1q4 = q1*q4;
	float q2q4 = q2*q4;
	float q3q4 = q3*q4;

	m.m[0][0] = qq1 + qq2 - qq3 - qq4;
	m.m[0][1] = 2.0f*(q2q3 - q1q4);
	m.m[0][2] = 2.0f*(q2q4 + q1q3);
	m.m[1][0] = 2.0f*(q2q3 + q1q4);
	m.m[1][1] = qq1 - qq2 + qq3 - qq4;
	m.m[1][2] = 2.0f*(q3q4 - q1q2);
	m.m[2][0] = 2.0f*(q2q4 - q1q3);
	m.m[2][1] = 2.0f*(q3q4 + q1q2);
	m.m[2][2] = qq1 - qq2 - qq3 + qq4;

	for (int r = 0; r < 3; r++)
		m.m[r][3] = origin.v[r];
	for (int c = 0; c < 3; c++)
		m.m[3][c] = 0;
	m.m[3][3] = 1.0f;
	return m;
}