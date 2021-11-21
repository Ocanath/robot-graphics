#ifndef KINEMATICS_H
#define KINEMATICS_H

#include "spatialAlgebra.h"
#include "sin_fast.h"

typedef struct dh_entry
{
	float d;
	float a;
	float alpha;

	//float sin_alpha;	//pre-store sin and cos of alpha, to avoid re-computing it every loop
	//float cos_alpha;
	//theta is stored separately
}dh_entry;

typedef struct joint
{
	float q;			//joint position
	float dq;			//joint velocity
	float ddq;			//joint acceleration
	vect6 Si;			//vector corresponding to the i'th column of the jacobian matrix. Si*q(i) = vi, where vi is the ith's joint's contribution to the total chain velocity in frame 0
	mat4 hb_i;			//homogeneous transformation relating the BASE frame to the current frame (i). hb_0 = him1_0
	mat4 him1_i;		//homogeneous transformation describing the rotation and translation from frame i-1 to the current frame (i). 
	//dh_entry dh;	//dh parameter describing the offset and angles from frame i-1 to the current frame (i)
	
	mat4 h_link;	//pre-computed, constant matrix. given that h_theta is the rotation matrix defined by q/theta, then him1_i = h_theta*h_link. 

	mat6 I_hat;			//6 matrix describing the spatial inertia of each frame with respect to the origin.

	struct joint * child;
}joint;

typedef struct kinematic_chain
{
	mat4 hw_b;
	mat4 hb_0;
	joint * j;	//list of joint variables, each containing the relevant information for that joint.
	int num_frames;
}kinematic_chain;


//void init_chain(kinematic_chain * chain, const dh_entry * table, const float * q_init, int num_frames);
//void chain_pre_allocate(kinematic_chain * chain, int num_frames);
//void chain_de_allocate(kinematic_chain * chain);
void init_forward_kinematics_urdf(joint* j, vect3* xyz, vect3* rpy, int num_joints);
void init_forward_kinematics_dh(joint* j, const dh_entry* dh, int num_joints);
void forward_kinematics(joint* j, int num_joints);
void calc_J_point(joint* j, int num_joints, vect3 point);
void copy_mat4(mat4* dest, mat4* src);
vect6 calc_w_v(kinematic_chain * chain, vect3 * w, vect3 * v);
void htmatrix_vect3_mult(mat4* m, vect3* v, vect3* ret);
vect3 h_origin(mat4 h);
vect4 h_origin_vect4(mat4 h);
mat4 quat_to_mat4(vect4 quat, vect3 origin);
void calc_tau(joint* j, int num_joints, vect6 f, float* tau);
void calc_tau3(joint* j, int num_joints, vect3* f, float* tau);	//faster alt to calc_tau

#endif