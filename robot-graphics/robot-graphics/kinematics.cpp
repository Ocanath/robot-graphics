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
	Copies the contents of one mat4 to the other. could use memcpy interchangably
*/
void copy_mat4(mat4 * dest, mat4 * src)
{
	for (int r = 0; r < 4; r++)
	{
		for (int c = 0; c < 4; c++)
		{
			dest->m[r][c] = src->m[r][c];
		}
	}
}

/*
*	Initialize links with urdf-style inputs (xyz and roll pitch yaw)
*/
void init_forward_kinematics_urdf(joint* j, vect3 * xyz, vect3 * rpy, int num_joints)
{
	for(int i = 1; i <= num_joints; i++)
	{
		mat4 Hlink = Hz(rpy[i].v[2]);
		Hlink = mat4_mult(Hlink, Hy(rpy[i].v[1]));
		Hlink = mat4_mult(Hlink, Hx(rpy[i].v[0]));	//obtained Hrpy
		for(int r = 0; r < 3; r++)
			Hlink.m[r][3] = xyz[i].v[r];
		copy_mat4(&j[i].h_link, &Hlink);	//load result into joint matrix
		copy_mat4(&j[i].him1_i, &j[i].h_link);	//initial q=0 loading for him1_i
		j[i - 1].child = &j[i];	//load child reference in case we want to traverse this as a singly linked list
	}	
	copy_mat4(&j[0].h_link, &j[0].hb_i);
	j[0].q = 0;
	j[num_joints].child = NULL;
	for (int i = 1; i <= num_joints; i++)
		mat4_mult_pbr(&j[i - 1].hb_i, &j[i].him1_i, &j[i].hb_i);
}

/*
load the kinematic chain Si, h0_i, him1_i variables based on the dh_table.
NOTE: only done ONCE, as the forward kinematics function only loads the elements which change with q

*/
/*
	load the kinematic chain Si, variables based on the dh_table.
	TODO: make this a linked list
*/
void init_forward_kinematics_dh(joint* j, const dh_entry* dh, int num_joints)
{
	for (int i = 1; i <= num_joints; i++)
	{
		float sin_alpha = sin_fast(dh[i].alpha);
		float cos_alpha = cos_fast(dh[i].alpha);
		/*Precomputed Hd*Ha*Halpha*/
		j[i].h_link = {
			{
				{1,		0,				0,				dh[i].a},
				{0,		cos_alpha,		-sin_alpha,		0},
				{0,		sin_alpha,		cos_alpha,		dh[i].d},
				{0,		0,				0,				1}
			}
		};
		copy_mat4(&j[i].him1_i, &j[i].h_link);	//htheta of 0 is the identity
		j[i - 1].child = &j[i];
	}
	copy_mat4(&j[0].h_link, &j[0].hb_i);
	j[0].q = 0;
	j[num_joints].child = NULL;	//initialize the last node in the linked list to NULL
	for (int i = 1; i <= num_joints; i++)
		mat4_mult_pbr(&j[i - 1].hb_i, &j[i].him1_i, &j[i].hb_i);
}
/*
Do forward kinematics on a singly linked list of joints!
Inputs:
		j: the root of the chain
Outputs:
		updates him1_i and hb_i of every element in the list

obviously singly linked list has same limitation as array. On
an embedded system, dynamic memory and recursion are not allowed 
(both of which would be quite nice here; i.e. we can implement this as a dfs
type recursive run through the tree) so we'll have to operate on the tree
in a rigid structure (i.e. hard-coding what would normally be recursive)
*/
void forward_kinematics(joint* j, int num_joints)
{
	//int i;
	//for (i = 1; i <= num_joints; i++)
	//{
	//	float sth = sin_fast(j[i].q);
	//	float cth = cos_fast(j[i].q);
	//	mat4 h_theta = { { {cth, -sth, 0,  0 }, { sth,cth, 0, 0}, {0, 0, 1, 0 }, {0, 0, 0, 1} } };
	//	mat4_mult_pbr(&h_theta, &j[i].h_link, &j[i].him1_i);
	//}
	//for (i = 1; i <= num_joints; i++)
	//	mat4_mult_pbr(&j[i - 1].hb_i, &j[i].him1_i, &j[i].hb_i);		//j[i].h0_i = mat4_mult(j[i - 1].h0_i, j[i].him1_i);


	joint* chain_start = j;
	while (j->child != NULL)
	{
		j = j->child;		//assume the passed pointer is the base of the chain, so start with the first child
		float sth = sin_fast(j->q);
		float cth = cos_fast(j->q);
		mat4 h_theta = { { {cth, -sth, 0,  0 }, { sth,cth, 0, 0}, {0, 0, 1, 0 }, {0, 0, 0, 1} } };
		mat4_mult_pbr(&h_theta, &j->h_link, &j->him1_i);
	}
	
	joint* parent = chain_start;	//since we only track the child node in the joint, we'll track the parent and only go down the list
	j = chain_start;
	while (j->child != NULL)
	{
		j = j->child;	//j is now one ahead of parent
		mat4_mult_pbr(&parent->hb_i, &j->him1_i, &j->hb_i);
		parent = j;
	}
}

/*
	Calculates the robot jacobian matrix satisfying the relationship v = J*qdot, where qdot is a vector of generalized joint velocities,
	and v is the linear velocity of the argument 'point' (expressed in chain frame 0, and rigidly attached to the end effector frame).

INPUTS:
	j : pointer to a list of joints, sized 'num_joints'. This function loads the Si vector in this list, which is eqal to the column vector of the jacobian matrix for that joint
	num_joints : number of joints in the joint list
	point: point, rigidly attached to the end effector (highest numerical index of the joint matrix) frame, expressed in frame 0

OUTPUTS:
	Si vectors in the joint list j.
NOTE:
	This jacobian matrix is following the jacobian matrix convention defined in Featherstone's texts, which
	loads the 'linear' (cross product) component in the latter 3 elements of the 6 vector. Therefore the resulting
	velocity vector of the jacobian multiply will be:
	{
		w0,
		w1,
		w2,
		v0,
		v1,
		v2
	};
	where v is the linear and w is the rotational component.
*******************************************************************************************************************
	Note also that the torque component for a given end effector force, for a given joint is given as follows:

		1. tau_i = dot(Si  f)

	Where f is the (6 vector) force and torque expressed in the 0 frame, Si is the column vector of
	the jacobian, and tau_i is the torque of that joint. This is equivalent to:

		2. [tau] = J^T * f

	Where [tau] is a vector of torques (size n/number of joints), J^T is the transpose of the jacobian matrix defined above, and
	f is the same generalized force/torque 6 vector expressed in the 0 frame.
*/
void calc_J_point(joint * j, int num_joints, vect3 point)
{
	int i, v_idx;
	vect3 z;
	vect3 d;
	for (i = 1; i <= num_joints; i++)
	{
		for (v_idx = 0; v_idx < 3; v_idx++)
			z.v[v_idx] = j[i - 1].hb_i.m[v_idx][2];					//extract the unit vector corresponding to the axis of rotation of frame i-1 (i.e., the axis of rotation of q)
		for (v_idx = 0; v_idx < 3; v_idx++)
			d.v[v_idx] = point.v[v_idx] - j[i - 1].hb_i.m[v_idx][3];	//extract the difference between the target point in the base frame and the origin of frame i-1
		vect3 res;
		cross_pbr(&z, &d, &res);

		j[i].Si.v[0] = z.v[0];		//Si = [w, v]^T; 
		j[i].Si.v[1] = z.v[1];		//z_im1*q = w
		j[i].Si.v[2] = z.v[2];

		j[i].Si.v[3] = res.v[0];	//(z_im1 x (p - o_im1))*q = v
		j[i].Si.v[4] = res.v[1];
		j[i].Si.v[5] = res.v[2];
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

//void calc_tau(kinematic_chain * chain, vect6 f, float * tau)
//{
//	int i;
//	for (i = 1; i < chain->num_frames; i++)
//		tau[i] = vect6_dot(chain->j[i].Si, f);
//}

void calc_tau(joint * j, int num_joints, vect6 f, float * tau)
{
	int i;
	for (i = 1; i <= num_joints; i++)
		tau[i] = vect6_dot(j[i].Si, f);
}

void calc_tau3(joint * j, int num_joints, vect3 * f, float* tau)
{
	for (int i = 1; i <= num_joints; i++)
	{
		float dp = 0.f;
		for (int r = 0; r < 3; r++)
			dp += j[i].Si.v[r + 3] * f->v[r];
		tau[i] = dp;
	}
}

/*pass by reference htmatrix (special subset of mat4) and 3 vector)*/
void htmatrix_vect3_mult(mat4* m, vect3* v, vect3* ret)
{
	int rsize = 3; int vsize = 4;
	int r, i;
	for (r = 0; r < rsize; r++)
	{
		float tmp = 0;
		for (i = 0; i < vsize; i++)
		{
			if (i < 3)
				tmp = tmp + m->m[r][i] * v->v[i];
			if (i == 3)
				tmp = tmp + m->m[r][i];
		}
		ret->v[r] = tmp;
	}
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