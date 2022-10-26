#include "sin_fast.h"
#include <stdint.h>
#include <math.h>
#include "finger-kinematics.h"
#include "kinematics.h"


//static const float l0 = 9.47966f;
static const vect3_t p3 = { 9.47966f,-0.62133f,0 };
static const float l1 = 38.6104f;
static const float l2 = 36.875f;
static const float l3 = 9.1241f;


enum { INDEX, MIDDLE, RING, PINKY, THUMB };

/*helper function to copy memory vect3s*/
void copy_vect3(vect3_t* dest, vect3_t* ref)
{
	for (int i = 0; i < 3; i++)
		dest->v[i] = ref->v[i];
}

/*Get the distance between two 3 vectors*/
float dist_vect3(vect3_t p0, vect3_t p1)
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
uint8_t get_intersection_circles(vect3_t o0, float r0, vect3_t o1, float r1, vect3_t solutions[2] )
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
    vect3_t p2;
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
    vect3_t p1 = 
    {
        l1*cq1,
        l1*sq1,
        0
    };
    vect3_t sols[2];
    get_intersection_circles(p3,l2,p1,l3,sols);
    vect3_t p2;
    copy_vect3(&p2, &sols[1]);
    
    // calculate the linkage intermediate angle!
    float q2pq1 = atan2_approx(p2.v[1]-l1*sq1, p2.v[0]-l1*cq1);
    float q2 = q2pq1-q1;
	return fmod_2pi(q2 + PI) - PI;;
}

const mat4_t id_matrix = {
	{
		{1.f,0.f,0.f,0.f},
		{0.f,1.f,0.f,0.f},
		{0.f,0.f,1.f,0.f},
		{0.f,0.f,0.f,1.f},
	}
};
	
//l1 = a1
//l3 = a2

const dh_entry generic_finger_dh[3] = {
	{0.f, 0.f, 0.f},
	{0.f, 38.6104f, 0.f},	//d, a, alpha
	{0.f, 9.1241f, 0.f}	//d, a, alpha
};

/*
Approximate Origin of index fingertip in frame 2 for whatever idk shrot finger or somrthign
26.6442
15.1816
0
*/
const dh_entry generic_thumb_dh[3] = {
	{0.f, 0.f, 0.f},
	{-14.7507f, 27.703f,	75.f*PI/180},	//d, a, alpha
	{3.9348f,	69.23921f,	0.f}			//d, a, alpha
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
			load_q(j->child);
			forward_kinematics(&j->hb_i, j->child);			
			vect3_t ref = {24.32f, -11.61f, 0.59f};
			vect3_t * res = &kh->finger[ch].ef_pos_b;
			htmatrix_vect3_mult(&j[2].hb_i, &ref, res);
			calc_J_point(&j[0].him1_i, j->child, res);
		}
		else
		{
			load_q(j->child);
			forward_kinematics(&j->hb_i,j->child);
			vect3_t ref = { -0.75f, -1.03f, -0.21f };
			vect3_t* res = &kh->finger[ch].ef_pos_b;
			htmatrix_vect3_mult(&j[2].hb_i, &ref, res);
			calc_J_point(&j[0].him1_i, j->child, res);
		}
	}
}



/**/
void init_finger_kinematics(kinematic_hand_t * kh)
{
	{
	 int fidx = INDEX;
		mat4_t tf = Hy(PI / 2 + 0.051012f);	//ish;
	tf = mat4_t_mult(tf, Hz(-0.250689f));	//
		tf.m[0][3] = -9.49f;
		tf.m[1][3] = -13.04f;
		tf.m[2][3] = -62.95f;
		copy_mat4_t(&kh->finger[fidx].chain[0].him1_i, &tf);	//him1_0 = hb_0
		copy_mat4_t(&kh->finger[fidx].chain[0].hb_i, &tf);
	}
	{
		int fidx = MIDDLE;
		mat4_t tf = Hy(PI / 2 + 0.086766f/2);	//ish
		tf = mat4_t_mult(tf, Hz(-0.252478f));	//
		tf.m[0][3] = 9.65;
		tf.m[1][3] = -15.31;
		tf.m[2][3] = -67.85f;
		copy_mat4_t(&kh->finger[fidx].chain[0].him1_i, &tf);	//him1_0 = hb_0
		copy_mat4_t(&kh->finger[fidx].chain[0].hb_i, &tf);
	}
	{
		int fidx = RING;
		mat4_t tf = Hy(PI / 2 + 0.051012f);	//ish
		tf = mat4_t_mult(tf, Hz(-0.250689f));	//
		tf.m[0][3] = 29.95f;
		tf.m[1][3] = -14.21f;
		tf.m[2][3] = -67.29f;
		copy_mat4_t(&kh->finger[fidx].chain[0].him1_i, &tf);	//him1_0 = hb_0
		copy_mat4_t(&kh->finger[fidx].chain[0].hb_i, &tf);
	}
	{
		int fidx = PINKY;
		mat4_t tf = Hy(PI / 2 + 0.111526539f/3);
		tf = mat4_t_mult(tf, Hz(-0.2486f));	//
		tf.m[0][3] = 49.52;
		tf.m[1][3] = -11.f;
		tf.m[2][3] = -63.03;
		copy_mat4_t(&kh->finger[fidx].chain[0].him1_i, &tf);	//him1_0 = hb_0
		copy_mat4_t(&kh->finger[fidx].chain[0].hb_i, &tf);
	}
	kh->finger[THUMB].chain[0].him1_i = mat4_t_I();
	kh->finger[THUMB].chain[0].hb_i = mat4_t_I();
	for(int ch = 0; ch < 5; ch++)
	{
		joint * j = kh->finger[ch].chain;
		if(ch != 4)
		{
			init_forward_kinematics_dh(j, generic_finger_dh, 2);
		}
		else
		{
			init_forward_kinematics_dh(j, generic_thumb_dh, 2);
		}		
	}
}
