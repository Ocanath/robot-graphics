#include "kinematics_fixed.h"

const mat4_32b gl_identity_matrix_32b =
{
	{ONE_ROT, 0, 0, 0},
	{0, ONE_ROT, 0, 0},
	{0, 0, ONE_ROT, 0},
	{0, 0, 0, ONE_ROT}
};

/*
TODO:

Create fully encapsulated, isolated fixed point kinematics implementation 
(progress on this is far along).

Dump the results of this development into a .csv file. Load that into matlab
and automate comparison of the results. 

The fixed point things should be an *exact* match with the fixed point ones
implemented in matlab.
*/


/*
 * N is the binary radix of the sin and cosine of the joint angle.
 * max value is 29 (should be 30, but stability not guaranteed)
 *
 * Need to ensure that n matches the scale/radix of sin and cosine
 * in the corresponding joint before calling this function
 */
void forward_kinematics_64(mat4_32b_t* hb_0, joint32_t* f1_joint)
{
	if (f1_joint == NULL)		//catch null pointer case for now, just to be sure
		return;

	joint32_t * j = f1_joint;
	while (j != NULL)
	{
		int64_t cth = (int64_t)j->cos_q;
		int64_t sth = (int64_t)j->sin_q;

		mat4_32b_t* r = &j->h_link;
		mat4_32b_t* him1_i = &j->h_im1_i;	//specify lookup ptr first for faster loading

		int64_t r00 = (int64_t)r->m[0][0];
		int64_t r01 = (int64_t)r->m[0][1];
		int64_t r02 = (int64_t)r->m[0][2];
		int64_t r03 = (int64_t)r->m[0][3];
		int64_t r10 = (int64_t)r->m[1][0];
		int64_t r11 = (int64_t)r->m[1][1];
		int64_t r12 = (int64_t)r->m[1][2];
		int64_t r13 = (int64_t)r->m[1][3];
		
		int n = j->n_r;
		him1_i->m[0][0] = (int32_t)((cth * r00 - r10 * sth) >> n);
		him1_i->m[0][1] = (int32_t)((cth * r01 - r11 * sth) >> n);
		him1_i->m[0][2] = (int32_t)((cth * r02 - r12 * sth) >> n);
		him1_i->m[0][3] = (int32_t)((cth * r03 - r13 * sth) >> n);
		him1_i->m[1][0] = (int32_t)((cth * r10 + r00 * sth) >> n);
		him1_i->m[1][1] = (int32_t)((cth * r11 + r01 * sth) >> n);
		him1_i->m[1][2] = (int32_t)((cth * r12 + r02 * sth) >> n);
		him1_i->m[1][3] = (int32_t)((cth * r13 + r03 * sth) >> n);
		him1_i->m[2][0] = r->m[2][0];
		him1_i->m[2][1] = r->m[2][1];
		him1_i->m[2][2] = r->m[2][2];
		him1_i->m[2][3] = r->m[2][3];

		j = j->child;
	}

	joint32_t* parent = f1_joint;
	j = f1_joint;
	ht32_mult64_pbr(hb_0, &j->h_im1_i, &j->hb_i, f1_joint->n_r);	//load hb_1.		hb_0 * h0_1 = hb_1
	while (j->child != NULL)
	{
		j = j->child;
		int n = j->n_r;
		ht32_mult64_pbr(&parent->hb_i, &j->h_im1_i, &j->hb_i, n);
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
void calc_J_32b_point(mat4_32b_t* hb_0, joint32_t * chain_start, vect3_32b_t * point_b)
{
	int nt = chain_start->n_t;
	int nr = chain_start->n_r;
	int nSi = chain_start->n_si;
	int n_cross = -(nSi - (nt + nr));	//the amount we should left shift to achieve the marked Si radix
	
	/* 
		First, obtain the base case. This is contribution of the velocity from joint 1, and 
		is found using the vector formed by the origin of joint 1 and the point, and the 
		axis of rotation of joint 1 in the base frame.
	*/
	int r;
	vect3_32b_t z, d, o0_b, res;
	h32_origin_pbr(&o0_b, hb_0);
	for (r = 0; r < 3; r++)
	{
		z.v[r] = hb_0->m[r][2];
		d.v[r] = point_b->v[r] - hb_0->m[r][3];
	}
	joint32_t* j = chain_start;
	/*This combines a translational part with a rotational part. Shifting out the translational part leaves a rotational scale vector*/
	cross64_pbr(&z, &d, &res, n_cross);
	for (int r = 0; r < 3; r++)
	{
		j->Si.v[r] = z.v[r];
		j->Si.v[r + 3] = res.v[r];
	}

	joint32_t* parent = j;
	while(j->child != NULL)
	{
		j = j->child;
		for (int r = 0; r < 3; r++)
		{
			z.v[r] = parent->hb_i.m[r][2];
			d.v[r] = point_b->v[r] - parent->hb_i.m[r][3];
		}
		cross64_pbr(&z, &d, &res, n_cross);
		for (int r = 0; r < 3; r++)
		{
			j->Si.v[r] = z.v[r];
			j->Si.v[r + 3] = res.v[r];
		}
		parent = j;
	}
}
