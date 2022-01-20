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
void foot_path(float time, float h, float w, float period, vect3_t* v)
{
	float t = fmod_2pi(time*TWO_PI/period)/TWO_PI;	//easy way of getting time normalized to 0-1 using prior work, without needing standard lib fmod
	
	float p1 = 0.25f;	//half down, half up. 
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

		/*
			-1 to 1, except that the first couple of derivatives are lower at the ends.
			Consequence for smoother motion is that the footspeed is much higher in the middle of the motion
		*/
		t = sin(HALF_PI * t);

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


