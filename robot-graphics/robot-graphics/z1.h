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

	//convenience function/wrapper for z1 fk
	void fk(void)
	{
		//load_q(&joints[1]);
		z1_forward_kinematics(&joints[0].hb_i, &joints[1]);
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
