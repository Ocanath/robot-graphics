#include "kinematics.h"
#include "kinematics_fixed.h"
#include <stdio.h>

void print_mat3(mat3_t m);
void print_mat4_t(mat4_t m);
void print_vect6(vect6_t v);
void print_vect4(vect4_t v);
void print_vect3(vect3_t v);
void generate_unit_cube(unsigned int * cubeVBO, unsigned int * cubeVAO);
void print_mat4_32b(mat4_32b_t m);