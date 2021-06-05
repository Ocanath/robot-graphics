#include "dynahex.h"



const dh_entry hexleg_dh[NUM_FRAMES_HEXLEG] = {
	{0,			0,					0,		0,0},		//d,a,alpha, sin_alpha, sin_theta
	{65.66f,	-53.2f,				PI/2,	0,0},
	{29.00f,	-100.46602344f,		PI,		0,0},
	{21.50f,	-198.31677025f,		0.f,	0,0}
};


void init_dh_kinematics(dynahex_t * h)
{
	mat4 hb_0_leg0 =
	{
		{
			{1.f,	0,		0,		109.7858f},
			{0,		1.f,	0,		0},
			{0,		0,		1.f,	0},
			{0,		0,		0,		1.f}
		}
	};
	const float angle_f = (2 * PI) / 6.f;
	for (int leg = 0; leg < NUM_LEGS; leg++)
	{
		joint * j = h->leg[leg].chain;
		for (int i = 0; i < NUM_FRAMES_HEXLEG; i++)
			j[i].dh = hexleg_dh[i];
		
		j[0].him1_i = mat4_mult(hb_0_leg0, Hz(PI));
		j[0].him1_i = mat4_mult(j[0].him1_i, Hx(PI));
		j[0].him1_i = mat4_mult(Hz(leg * angle_f), j[0].him1_i);
		copy_mat4(&j[0].hb_i, &j[0].him1_i);

		init_forward_kinematics(j, NUM_JOINTS_HEXLEG);
	}
}

void forward_kinematics_dynahexleg(dynahex_t* h)
{
	for (int leg = 0; leg < NUM_LEGS; leg++)
	{
		joint* j = h->leg[leg].chain;
		forward_kinematics(j, NUM_JOINTS_HEXLEG);
		for (int r = 0; r < 3; r++)
			h->leg[leg].ef_b.v[r] = j[3].hb_i.m[r][3];
		calc_J_point(j, NUM_JOINTS_HEXLEG, h->leg[leg].ef_b);
	}
}