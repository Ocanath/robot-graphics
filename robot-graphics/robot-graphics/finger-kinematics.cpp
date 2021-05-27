#include "sin_fast.h"
#include <stdint.h>
#include <math.h>
#include "finger-kinematics.h"
#include "kinematics.h"


static const float l0 = 1.f;
static const float l1 = 3.f;
static const float l2 = 3.f;
static const float l3 = 1.f;
//static const vect3_t p0 = {0,0,0};
static const vect3 p3 = {l0,0,0};


/*helper function to copy memory vect3s*/
void copy_vect3(vect3* dest, vect3* ref)
{
	for (int i = 0; i < 3; i++)
		dest->v[i] = ref->v[i];
}

/*Get the distance between two 3 vectors*/
float dist_vect3(vect3 p0, vect3 p1)
{
    double sum_sq = 0.f;
    for(int i = 0; i < 3; i++)
    {
        float tmp = (p0.v[i] - p1.v[i]);
        tmp = tmp*tmp;
        sum_sq += (double)tmp;
    }
    return (float)(sqrt(sum_sq));
}

/*
    Get the intersection of two circles in the x-y plane. Two solutions for valid inputs. 
    Invalid inputs are those which result in the following cases:
        1. No intersection 
        2. infinite intersections
        3. one solution
    It is simple to check for and reject these cases, but to save a little time and complexity we will 
    assume correct inputs (all inputs for the 4bar linkage case are correct inputs)
*/
uint8_t get_intersection_circles(vect3 o0, float r0, vect3 o1, float r1, vect3 solutions[2] )
{
    float d = dist_vect3(o0,o1);
    //can do validity checks here (intersecting, contained in each other, equivalent) but they're unnecessary for our application
    
    //get some reused squares out of the way
    float r0_sq = r0*r0;
    float r1_sq = r1*r1;
    float d_sq = d*d;

    // solve for a
    float a = (r0_sq - r1_sq + d_sq)/(2*d);

    // solve for h
    float h_sq = r0_sq - a*a;
    float h = sqrt(h_sq);

    float one_by_d = 1.f/d;
    // find p2
    vect3 p2;
    p2.v[2] = 0.f;
    for(int i = 0; i < 2; i++)
        p2.v[i]= o0.v[i] + a*(o1.v[i]-o0.v[i])*one_by_d;
    
    float t1 = h*(o1.v[1]-o0.v[1])*one_by_d;
    float t2 = h*(o1.v[0]-o0.v[0])*one_by_d;

    solutions[0].v[0] = p2.v[0] + t1;
    solutions[0].v[1] = p2.v[1] - t2;
    solutions[0].v[2] = 0.f;
    
    solutions[1].v[0] = p2.v[0] - t1;
    solutions[1].v[1] = p2.v[1] + t2;
    solutions[1].v[2] = 0.f;
    return 1;
}

/**/
float get_4bar_driven_angle(float q1)
{
    float cq1 = cos_fast(q1);
    float sq1 = sin_fast(q1);
    vect3 p1 = 
    {
        l1*cq1,
        l1*sq1,
        0
    };
    vect3 sols[2];
    get_intersection_circles(p3,l2,p1,l3,sols);
    vect3 p2;
    copy_vect3(&p2, &sols[1]);
    
    // calculate the linkage intermediate angle!
    float q2pq1 = atan2_approx(p2.v[1]-l1*sq1, p2.v[0]-l1*cq1);
    float q2 = q2pq1-q1;
    return q2;
}

const mat4 id_matrix = {
	{
		{1.f,0.f,0.f,0.f},
		{0.f,1.f,0.f,0.f},
		{0.f,0.f,1.f,0.f},
		{0.f,0.f,0.f,1.f},
	}
};

void htmatrix_vect3_mult(mat4 * m, vect3 * v, vect3 * ret)
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
	
//l1 = a1
//l3 = a2

const dh_entry generic_finger_dh[2] = {
	{0.f, 38.6104f, 0.f, 0.f, 0.f},	//d, a, alpha, sin_alpha, cos_alpha
	{0.f, 36.875f, 0.f, 0.f, 0.f}	//d, a, alpha, sin_alpha, cos_alpha
};

/*
Approximate Origin of index fingertip in frame 2 for whatever idk shrot finger or somrthign
26.6442
15.1816
0
*/

const dh_entry generic_thumb_dh[2] = {
	{-14.7507f, 27.703f,	75.f*PI/180,	0.f, 0.f},	//d, a, alpha, sin_alpha, cos_alpha
	{3.9348f,	69.23921f,	0.f,			0.f, 0.f}			//d, a, alpha, sin_alpha, cos_alpha
};

/**/
void finger_kinematics(kinematic_hand_t * kh)
{
	for(int ch = 0; ch < 5; ch++)
	{
		joint * j = kh->finger[ch].chain;
		if(ch != 4)
		{	
			float q1 = j[1].q;	//DEG_TO_RAD*(hand->mp[ch].q+4.84f);
			float q2 = get_4bar_driven_angle(q1);
			j[2].q = q2;
			forward_kinematics(j,2);			
			vect3 ref = {26.6442f, 15.1816f, 0.f};
			htmatrix_vect3_mult(&j[2].h0_i, &ref, &kh->finger[ch].ef_pos_0);
		}
		else
		{
			forward_kinematics(j,2);
			for(int r = 0; r < 3; r++)
				kh->finger[ch].ef_pos_0.v[r] = j[2].h0_i.m[r][3];
		}
	}
}

/**/
void init_finger_kinematics(kinematic_hand_t * kh)
{
	for(int ch = 0; ch < 5; ch++)
	{
		joint * j = kh->finger[ch].chain;
		if(ch != 4)
		{
			j[1].dh = generic_finger_dh[0];
			j[2].dh = generic_finger_dh[1];	//hard copy. same line count as for loop
		}
		else
		{
			j[1].dh = generic_thumb_dh[0];
			j[2].dh = generic_thumb_dh[1];
		}
		init_forward_kinematics(j,2);
	}
}
