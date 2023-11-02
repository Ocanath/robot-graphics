#ifndef Z1_H
#define Z1_H
#include "model.h"
#include "kinematics.h"

#define NUM_JOINTS_Z1 6	//the actual number of joints. Lists will use number of joints PLUS ONE
#define NUM_LINKS_Z1 6



class Z1_arm
{
private:
	mat4_t homecfg_targ = { 0 };

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

		{0,0,0},	//{0,			0,	0.585f},
		{0,			0,	0.045f},
		{-0.353f,	0,	0},
		{0.218f,	0,	0.057f},
		{0.07f,	0,	0},
		{0.0492f,	0,	0}
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

	

public:
	mat4_t hw_b;
	std::vector<AssetModel> modellist;
	joint joints[NUM_JOINTS_Z1 + 1] = { 0 };
	mat4_t hw_i;
	mat4_t scale;

	const vect3_t targ_anchors[4] = {	//this set of points can be arbitrary, but best performance should theoretically be some homogeneous transformation of the arm anchors. This one is hand transformed with a very simple transform.
		{{0.0, 0, 0}},
		{{0.0, -2.0, -2.0}},
		{{0.0, -2.0, 2.0}},
		{{0.0, 0, 2}}
	};

	const mat4_t hf6_targ =
	{
		{
			{1, 0, 0, 103.e-3},
			{0, 1, 0, 0},
			{0, 0, 1, 0},
			{0, 0, 0, 1}
		}
	};

	vect3_t arm_anchors_f6[4] = {0};


	Z1_arm(void)
	{
		for (int anchor_idx = 0; anchor_idx < 4; anchor_idx++)
		{
			arm_anchors_f6[anchor_idx] = mat4_t_vect3_mult(hf6_targ, targ_anchors[anchor_idx]);
		}
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
		fk();
		homecfg_targ = get_targ_from_cur_cfg();
	}

	/*wrapper / read only alias for the home configuration*/
	mat4_t init_targ(void)
	{
		return homecfg_targ;
	}

	void z1_limit_joints(void)
	{
		joints[1].q = limit_val(joints[1].q, -2.6179938779914944, 2.6179938779914944);
		joints[2].q = limit_val(joints[2].q, 0, 3.141592653589793);
		joints[3].q = limit_val(joints[3].q, -4.782202150464463, 0);
		joints[4].q = limit_val(joints[4].q, -1.7453292519943295, 1.5707963267948966);
		joints[5].q = limit_val(joints[5].q, -1.7278759594743864, 1.7278759594743864);
		joints[6].q = limit_val(joints[6].q, -2.792526803190927, 2.792526803190927);
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


	mat4_t get_targ_from_cur_cfg(void)
	{
		fk(); //update cur cfg
		return mat4_t_mult(joints[6].hb_i, hf6_targ);
	}


	/*input arguments:
	Note: you can use rpy xyz to conveniently construct the input ht matrix
	mat4_t ht_b_targ = get_rpy_xyz_htmatrix(p_targ, rpy_targ);

	*/
	double num_ik(mat4_t* ht_b_targ, int iters)
	{
		double td = 0;
		for (int ncyc = 0; ncyc < iters; ncyc++)	//iterate for specified number of cycs
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

				/*compute virtual force vector between each anchor and target location in the base frame*/
				vect3_t virtual_force = { {0,0,0} };
				for (int i = 0; i < 3; i++)
				{
					double dvf = ((double)anchor_b_target_loc.v[i] - (double)anchor_b_current_loc.v[i]);
					dvf = dvf / 16.0;	//small scale for numerical stabilities
					virtual_force.v[i] = (float)dvf;
				}

				for (int i = 1; i <= NUM_JOINTS_Z1; i++)
				{
					joint* j = &joints[i];

					double tau = 0.;
					for (int r = 0; r < 3; r++)
						tau += (((double)j->Si.v[r + 3]) * ((double)virtual_force.v[r])) / 16.0;
					j->q += (float)tau;
				}
				z1_limit_joints();	//impose rigid joint stoppers, brute force limitng the workspace for IK

				if (ncyc == iters - 1)
				{
					double x = (double)virtual_force.v[0];
					double y = (double)virtual_force.v[1];
					double z = (double)virtual_force.v[2];
					td += sqrt(x * x + y * y + z * z);//double precision vf vector magnitude, representing distance to 
				}
			}
		}
		return td;
	}
	void render_arm(Shader& s)
	{
		for (int i = 1; i <= NUM_JOINTS_Z1; i++)
		{
			mat4_t hwb_sc = mat4_t_mult(hw_b, scale);
			hw_i = mat4_t_mult(hwb_sc, joints[i].hb_i);
			//hw_i = mat4_t_mult(hw_i, scale);
			glm::mat4 model = ht_matrix_to_mat4_t(hw_i);
			s.setMat4("model", model);
			modellist[i-1].Draw(s, NULL);
		}
	}
	/*piecewise function which punches the robot
		INPUTS:
			init_cfg: pointer to the homogeneous transformation representing the initial configuration of the z1
			ft: time variable. scalable. each segment of the path is calibrated for 1 ft unit. so if ft = t, each primitive will take 1 second.
	*/
	mat4_t generate_punch_path(mat4_t* init_cfg, double ft)
	{
		mat4_t ret_cfg = *init_cfg;	//copy init config
		//const int xp1 = 
		if (ft >= 0. && ft <= 1.)
		{
			//ret_cfg = 
		}
		return ret_cfg;
	}


};



#endif
