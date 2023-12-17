#include "hexapod_footpath.h"
#include <math.h>
#include <stdio.h>
#define ONE_12B		4096
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
		for(int i = 0; i < 1; i++)	//the more iterations of this you run, the more like a step function this becomes. 
			t = sin_fast(HALF_PI * t);
		
		v->v[0] = -t * w / 2.f;
		//v->v[0] = -sin_fast(t)*sin_fast(t + HALF_PI) * 2.199f * (w / 2.f);
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

uint8_t tick_flag = 0;
/*
* Period representation is in seconds*4096
*/
void foot_path_fixed(uint32_t time_ms, int32_t h, int32_t w, int32_t period, vect3_32b_t* v)
{
	if (period == 0)
		return;
	if (v == NULL)
		return;
	
	//64 bit representation means:
	//the maximum runtime of this function goes from 166 seconds
	//to 81 million years (without there being jitter in the time
	//signal)
	uint64_t t64 = (time_ms * ONE_12B) / 1000;	//time in 4096unit/sec

	//scale by period, wrap to 2*pi to normalize
	int32_t t = ((int32_t)((t64 * (uint64_t)TWO_PI_12B) / (uint64_t)period) % TWO_PI_12B);
	t = (t * ONE_12B) / TWO_PI_12B;

	
	uint32_t p1 = 1024;	//.25*4096
	uint32_t p2 = ONE_12B - p1;

	if (t >= 0 && t < p1)
	{
		t = t * ONE_12B / p1;	//parametric function expects 0-1. 

		v->v[0] = (t * (w / 2)) >> 12;	//original w radix preserved
		v->v[1] = 0;
		v->v[2] = 0;
	}
	else if (t >= p1 && t < p2)
	{
		//t = 2.f * (t - p1) / (p2 - p1) - 1.f; // this function expects t = -1 to 1
		t = (2 * ((t - p1)*ONE_12B)) / (p2 - p1) - ONE_12B;

		/*
			-1 to 1, except that the first couple of derivatives are lower at the ends.
			Consequence for smoother motion is that the footspeed is much higher in the middle of the motion
		*/
		for (int i = 0; i < 1; i++)	//the more iterations of this you run, the more like a step function this becomes. 
			t = sin_lookup((PI_12B * t) >> 1, 12);

		v->v[0] = (-t * w / 2) >> 12;
		v->v[1] = ( (((-t * t) >> 12) + ONE_12B) * h ) >> 12;
		v->v[2] = 0;
	}
	else if (t >= p2 && t < 1)
	{
		t = ((t - p2) * ONE_12B) / (1 - p2);

		v->v[0] = (t * (w / 2)) >> 12 - (w / 2);
		v->v[1] = 0;
		v->v[2] = 0;
	}
}