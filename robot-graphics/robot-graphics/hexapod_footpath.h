#ifndef HEXAPOD_FOOTPATH_H
#define HEXAPOD_FOOTPATH_H
#include "sin_fast.h"
#include "spatialAlgebra.h"

float parabola_length(float w, float h);
float get_p1_uniform_speed(float h, float w);
void foot_path(float time, float h, float w, float period, vect3_t* v);

#endif // !HEXAPOD_FOOTPATH_H
