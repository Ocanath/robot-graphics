#ifndef Z1_H
#define Z1_H
#include "model.h"
#include "kinematics.h"

#define NUM_JOINTS_Z1 6	//the actual number of joints. Lists will use number of joints PLUS ONE
#define NUM_LINKS_Z1 6


class Z1_arm
{
private:
	const vect3_t z1_joints_rpy[NUM_JOINTS_Z1 + 1] =
	{
		{0,0,0},
		{0,0,0},
		{0,0,0},
		{0,0,0},
		{0,0,0},
		{0,0,0},
		{0,0,0}
	};

	const vect3_t z1_joints_xyz[NUM_JOINTS_Z1 + 1] =
	{
		{0,			0,	0},

		{0,0,0},	//{0,			0,	0.585e1f},
		{0,			0,	0.045e1f},
		{-0.353e1f,	0,	0},
		{0.218e1f,	0,	0.057e1f},
		{0.07e1f,	0,	0},
		{0.0492e1f,	0,	0}
	};

	const char* z1_meshnames[NUM_LINKS_Z1] =
	{
		//"URDF/render/z1_Link00.stl",
		"URDF/render/z1_Link01.stl",
		"URDF/render/z1_Link02.stl",
		"URDF/render/z1_Link03.stl",
		"URDF/render/z1_Link04.stl",
		"URDF/render/z1_Link05.stl",
		"URDF/render/z1_Link06.stl"
	};

	const char z1_rotaxis[NUM_JOINTS_Z1 + 1] = {
		0,
		'z',
		'y',
		'y',
		'y',
		'z',
		'x',
	};

	mat4_t scale;
public:
	mat4_t hw_b;
	std::vector<AssetModel> modellist;
	joint joints[NUM_JOINTS_Z1 + 1] = { 0 };
	mat4_t hw_i;

	const vect3_t arm_anchors_f6[4] = {
		{{0.051000e1f, 0, 0}},
		{{0.051000e1f, -1.0, -1.0}},
		{{0.051000e1f, -1.0, 1.0}},
		{{0.051000e1f, 0, 1}}
	};

	const vect3_t targ_anchors[4] = {	//this set of points can be arbitrary, but best performance should theoretically be some homogeneous transformation of the arm anchors. This one is hand transformed with a very simple transform.
		{{0.0, 0, 0}},
		{{0.0, -1.0, -1.0}},
		{{0.0, -1.0, 1.0}},
		{{0.0, 0, 1}}
	};


	Z1_arm(void)
	{

		copy_mat4_t(&hw_b, (mat4_t*)&mat4_t_Identity);
		copy_mat4_t(&scale, (mat4_t*)&mat4_t_Identity);
		for (int i = 0; i < 3; i++)
			scale.m[i][i] = 10.f;
		for (int i = 0; i <= NUM_JOINTS_Z1; i++)
		{
			joints[i].rotaxis = z1_rotaxis[i];
		}
		for (int i = 0; i < NUM_LINKS_Z1; i++)
		{
			modellist.push_back((AssetModel)z1_meshnames[i]);
		}
		copy_mat4_t(&joints[0].hb_i, (mat4_t*)&mat4_t_Identity);
		init_forward_kinematics_urdf(joints, (vect3_t*)z1_joints_xyz, (vect3_t*)z1_joints_rpy, 6);
	}

	void z1_forward_kinematics(mat4_t* hb_0, joint* f1_joint)
	{
		if (f1_joint == NULL)
			return;

		joint* j = f1_joint;
		while (j != NULL)
		{
			//float sth = (float)sin((double)j->q);
			//float cth = (float)cos((double)j->q);
			//float sth = j->sin_q;
			//float cth = j->cos_q;

			mat4_t* r = &j->h_link;
			mat4_t* him1_i = &j->him1_i;	//specify lookup ptr first for faster loading

			if (j->rotaxis == 'x')
			{
				mat4_t hrot = Hx(j->q);
				mat4_t_mult_pbr(r , &hrot, him1_i);
			}
			else if (j->rotaxis == 'y')
			{
				mat4_t hrot = Hy(j->q);
				mat4_t_mult_pbr(r, &hrot, him1_i);
			}
			else //z, last option
			{
				mat4_t hrot = Hz(j->q);
				mat4_t_mult_pbr(r, &hrot, him1_i);
			}

			j = j->child;
		}

		joint* parent = f1_joint;
		j = f1_joint;
		mat4_t_mult_pbr(hb_0, &j->him1_i, &j->hb_i);	//load hb_1.		hb_0 * h0_1 = hb_1
		while (j->child != NULL)
		{
			j = j->child;
			mat4_t_mult_pbr(&parent->hb_i, &j->him1_i, &j->hb_i);
			parent = j;
		}
	}
	
	void z1_jacobian_entry(vect6_t* Si, vect3_t* p_b, mat4_t* hb_parent, char axis)
	{
		vect3_t rotaxis, d, res;
		int axis_index = (int)axis - (int)'x';	//hack to produce correct axis index. x = 0, y = 1, z = 2
		
		for (int r = 0; r < 3; r++)
		{
			rotaxis.v[r] = hb_parent->m[r][axis_index];	//parent frame contains the origin of the axis of rotation of the current joint
			d.v[r] = p_b->v[r] - hb_parent->m[r][3];
		}
		cross_pbr(&rotaxis, &d, &res);	//axis of rotation cross vector formed by target point minus origin of current axis of rotation
		for (int r = 0; r < 3; r++)
		{
			Si->v[r] = rotaxis.v[r];
			Si->v[r + 3] = res.v[r];
		}
	}
	/*
	* traverse the arm as a singly linked list kinematic structure
	* special jacobian calculation because the axis of rotation can be different now
	* only calculate the 
	*/
	void z1_jacobian(mat4_t* hb_0, joint* chain_start, vect3_t * point_b)
	{
		joint* j = chain_start;
		z1_jacobian_entry(&j->Si, point_b, hb_0, j->rotaxis);
		joint* parent = j;
		while (j->child != NULL)
		{
			j = j->child;
			z1_jacobian_entry(&j->Si, point_b, &j->hb_i, j->rotaxis);
			parent = j;
		}
	}

	//convenience function/wrapper for z1 fk
	void fk(void)
	{
		//load_q(&joints[1]);
		z1_forward_kinematics(&joints[0].hb_i, &joints[1]);
	}

	/*input arguments:
	Note: you can use rpy xyz to conveniently construct the input ht matrix
	mat4_t ht_b_targ = get_rpy_xyz_htmatrix(p_targ, rpy_targ);

	*/
	void num_ik(mat4_t* ht_b_targ)
	{
		
		fk();
		/*per-anchor*/
		for (int anchor_idx = 0; anchor_idx < 4; anchor_idx++)
		{
			/*obtain the anchor*/
			vect3_t anchor_b_current_loc;
			htmatrix_vect3_mult(&joints[6].hb_i, (vect3_t*)&arm_anchors_f6[anchor_idx], &anchor_b_current_loc);
			
			/*Must make SPECIFIC jacobian calculation which uses varying axis of rotation.*/
			z1_jacobian(&joints[0].hb_i, &joints[1], &anchor_b_current_loc);

			vect3_t anchor_b_target_loc;
			htmatrix_vect3_mult(ht_b_targ, (vect3_t*)&targ_anchors[anchor_idx], &anchor_b_target_loc);

			vect3_t virtual_force = { {0,0,0} };
			for (int i = 0; i < 3; i++)
			{
				double dvf = ((double)anchor_b_target_loc.v[i] - (double)anchor_b_current_loc.v[i]);
				dvf = dvf / 20.0;	//small scale for numerical stabilities
				virtual_force.v[i] = (float)dvf;
			}
			
			for (int i = 1; i <= NUM_JOINTS_Z1; i++)
			{
				joint* j = &joints[i];
				
				double tau = 0.;
				for (int r = 0; r < 3; r++)
					tau += (((double)j->Si.v[r + 3]) * ((double)virtual_force.v[r])) / 20.0;
				j->q += (float)tau;
			}
		}
	}
	void render_arm(Shader& s)
	{
		for (int i = 1; i <= NUM_JOINTS_Z1; i++)
		{
			hw_i = mat4_t_mult(hw_b, joints[i].hb_i);
			hw_i = mat4_t_mult(hw_i, scale);
			glm::mat4 model = ht_matrix_to_mat4_t(hw_i);
			s.setMat4("model", model);
			modellist[i-1].Draw(s, NULL);
		}
	}
};



#endif
