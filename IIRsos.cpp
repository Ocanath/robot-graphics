/*
 * iirSOS.c
 *
 *  Created on: May 9, 2018
 *      Author: Ocanath
 */
#include "IIRsos.h"


/*These are two copies of the same filter for use with emg filtering
matlab fda tool:
Butterworth IIR
200Hz (5ms)
3-5Hz cutoff frequency
*/
const iirSOS lpf_template = {
    -1.822695,
    0.837182,
    1.000000,
    2.000000,
    1.000000,
    0.003622,
    {0, 0, 0}
};

float sos_f(iirSOS * f, float newSample)
{
	f->w[2] = f->w[1];
	f->w[1] = f->w[0];
	f->w[0] = newSample - f->a1*f->w[1] - f->a2*f->w[2];
	return f->gain*(f->b0 * f->w[0] + f->b1*f->w[1] + f->b2*f->w[2]);
}

float filter_iir(iirSOS * filter, int order, float new_sample)
{
	float filtered_sample;
	int i = 0;
	filtered_sample = sos_f(&(filter[i]), new_sample);
	for(i=1;i<order;i++)
		filtered_sample = sos_f(&(filter[i]), filtered_sample);
	return filtered_sample;
}
