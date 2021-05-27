#ifndef SPATIALALGEBRA_H
#define SPATIALALGEBRA_H

#include <math.h>

#define X_IDX 0
#define Y_IDX 0
#define Z_IDX 0

#define PI 3.14159265359f

/*
	Rotation matrices, skew symmetric matrices take this type
*/
typedef struct mat3
{
	float m[3][3];
}mat3;

/*
	homogeneous transformation matrices take this type	
*/
typedef struct mat4
{
	float m[4][4];
}mat4;

/*
	spatial coordinate transformations and spatial inertia tensors take this type.
	NOTE: future implementations shouldn not use this variable type, instead opting for optimizations that are mathematically equivalent
	(such as storage of a spatial transformation as a 1 mat3, and a 3 vector)
*/
typedef struct mat6
{
	float m[6][6];
}mat6;

/*
	Line, free vectors. 
*/
typedef struct vect3
{
	float v[3];
}vect3;

/*
homogeneous transformation matrix vectors (lol)
*/
typedef struct vect4
{
	float v[4];
}vect4;

/*
	spatial vectors
*/
typedef struct vect6
{
	float v[6];
}vect6;


extern const mat4 mat4_Identity;
extern const mat3 mat3_Identity;


vect6 spatial_vect6_cross(vect6 a, vect6 b);
mat6 spatial_cross_operator(vect6 v);
vect3 cross(vect3 v_a, vect3 v_b);
void cross_pbr(vect3* v_a, vect3* v_b, vect3* ret);
mat3 skew(vect3 vin);
vect6 vect6_scale(vect6 v_a, float scale);
vect4 vect4_scale(vect4 v_a, float scale);
vect3 vect3_scale(vect3 v_a, float scale);
float vect6_dot(vect6 v_a, vect6 v_b);
float vect4_dot(vect4 v_a, vect4 v_b);
float vect3_dot(vect3 v_a, vect3 v_b);
vect6 vect6_add(vect6 v_a, vect6 v_b);
vect4 vect4_add(vect4 v_a, vect4 v_b);
vect3 vect3_add(vect3 v_a, vect3 v_b);
float vect3_magnitude(vect3 v);
vect3 vect3_normalize(vect3 v);
vect6 zero_vect6();
vect6 mat6_vect6_mult(mat6 m, vect6 v);
vect4 mat4_vect4_mult(mat4 m, vect4 v);
vect3 mat4_vect3_mult(mat4 m, vect3 v);	//untested
vect3 mat3_vect3_mult(mat3 m, vect3 v);
mat6 mat6_mult(mat6 m1, mat6 m2);
mat4 mat4_mult(mat4 m1, mat4 m2);
void mat4_mult_pbr(mat4* m1, mat4* m2, mat4* ret);
mat3 mat3_mult(mat3 m1, mat3 m2);
mat6 mat6_add(mat6 m1, mat6 m2);
mat4 mat4_add(mat4 m1, mat4 m2);
mat3 mat3_add(mat3 m1, mat3 m2);
mat4 mat4_I();
mat3 mat3_I();
mat3 Rx(float angle);
mat3 Ry(float angle);
mat3 Rz(float angle);
mat4 ht_inverse(mat4 hin);
mat4 Hx(float angle);
mat4 Hy(float angle);
mat4 Hz(float angle);
mat3 mat3_T(mat3 in);
void matrix_vect_multiply(float ** M1, int rsize, float * v, int vsize, float * out);
mat6 h4_to_X(mat4 ha_b);
vect6 c_change_spatial(mat4 ha_b, vect6 vb);

vect4 vect3_to_vect4(vect3 in);
vect3 vect4_to_vect3(vect4 in);

#endif