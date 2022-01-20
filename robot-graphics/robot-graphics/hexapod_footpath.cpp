#include "hexapod_footpath.h"
#include <math.h>

/*
This file contains functions dedicated to calculating parametric piecewise functions dedicated to 
generating walking foot position targets for hexapod robots. It may serve as a floating point basis for
future fixed point versions of this code.

The core idea here is to generate xyz targets by rotating and translating a planar, piecewise
function [x(t) y(t)] which creates a 'dome' path for a foot to traverse through.

The path is parameterized by w, the width (in x) and h, the height (in y).
The period of the motion is parameterizable as well.

at t=0, path is [0 0]
at t=period, path is [0 0]

Using closed form integrals, the footspeed can be normalized for the entire motion period.
*/



/*
Return the closed form integral of the piecewise function:

x(t) = -(w/2)*t
y(t) = h(-t^2+1)

*/
float parabola_integral_closedform(float w, float h, float t)
{
	float ww = w * w;
	float hh = h * h;
	float tt = t * t;

	float arg = 16.f * hh * tt + ww;
	float a = sqrt((double)arg);
	float b = 4.f * h * t;

	arg = h * (a + b);
	float tmpl = a * b + ww * log(arg);
	return tmpl / (h * 16.f);
}

/*
* Return the length of the parabola from -1 <= t <= 1
*/
float parabola_length(float w, float h)
{
	return parabola_integral_closedform(w, h, 1) - parabola_integral_closedform(w, h, -1);
}

/*
* Get the percentage of the period you spend in the FIRST HALF of the LINEAR PORTION
* of the footpath, which will generate a uniform foot velocity throughout the entire
* motion period.
* 
* Note: p2 will always be equal to 1-p1.
*/
float get_p1_uniform_speed(float h, float w)
{
	/*
	Get the distance travelled by the foot in the parabolic arc, and the percentage of that travel which
	corresponds to linear travel.
	*/
	float len = parabola_length(w, h);
	float perc_step = w / (w + len);

	/*
	IMPORTANT: p1 and p2 determine the speed of each segment of the motion.
	Consider passing p1 and p2 as function arguments, and calculating them elsewhere
	*/
	float p1 = perc_step / 2.f;
	//float p2 = 1.f - p1;	//note

	return p1;
}

/*Computes the piecewise parametric function to create a footpath.
* 
* The resultant vector must be rotated and translated before it is a legitimate result for walking motion generation

INPUTS:
	time: current time, in same time units as period
	period: total time the motion is scheduled to take
	h: the height of the path (y)
	w: the width of the path. will go x = +w/2 to x=-w/2, starting at (0,0)
	p1: the percentage of the total period after which the parabolic arc will begin.
		It will return to the second half of the linear motion at p2, which is equal
		to 1-p1
OUTPUTS:
	v: output vector. Z always = 0. Pass by reference.
*/
void foot_path(float time, float h, float w, float p1, float period, vect3_t* v)
{
	float t = fmod_2pi(time*TWO_PI/period)/TWO_PI;	//easy way of getting time normalized to 0-1 using prior work, without needing standard lib fmod
	
	float p2 = 1.f - p1;

	if (t >= 0 && t < p1)
	{
		t = (t - 0) / (p1 - 0);	//parametric function expects 0-1. 

		v->v[0] = t * (w / 2);
		v->v[1] = 0;
		v->v[2] = 0;
	}
	else if(t >= p1 && t < p2)
	{
		t = 2.f * (t - p1) / (p2 - p1) - 1.f; // this function expects t = -1 to 1
		
		v->v[0] = -t * w / 2.f;
		v->v[1] = (-t*t + 1) * h;
		v->v[2] = 0;
	}
	else if(t >= p2 && t < 1)
	{
		t = (t - p2) / (1 - p2);

		v->v[0] = t * (w / 2) - (w / 2);
		v->v[1] = 0;
		v->v[2] = 0;
	}
	else
	{
		for (int i = 0; i < 3; i++)
			v->v[i] = 0;
	}
}


