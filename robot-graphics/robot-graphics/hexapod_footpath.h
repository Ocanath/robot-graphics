#ifndef HEXAPOD_FOOTPATH_H
#define HEXAPOD_FOOTPATH_H
#include "sin_fast.h"
#include "spatialAlgebra.h"
#include "vect_fixed.h"

void foot_path(float time, float h, float w, float period, vect3_t* v);
void foot_path_fixed(uint32_t time_ms, int32_t h, int32_t w, int32_t period, vect3_32b_t* v);

#endif // !HEXAPOD_FOOTPATH_H
