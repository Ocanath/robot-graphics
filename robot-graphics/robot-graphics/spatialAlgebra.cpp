#include "spatialAlgebra.h"

/*
TOOD: Benchmark speed
TODO: implement bottom method using float pointers instead of copying the top and bottom of each into a new location (for speed increase)
*/
vect6 spatial_vect6_cross(vect6 a, vect6 b)
{
	//mat6 across = spatial_cross_operator(a);
	//vect6 ret = mat6_vect6_mult(across, b);
	//return ret;
	vect3 top_a;
	vect3 bot_a;
	vect3 top_b;
	vect3 bot_b;
	int i;
	for (i = 0; i < 3; i++)
	{
		top_a.v[i] = a.v[i];
		bot_a.v[i] = a.v[i + 3];
		top_b.v[i] = b.v[i];
		bot_b.v[i] = b.v[i + 3];
	}
	vect3 top = cross(top_a, top_b);
	vect3 bot = vect3_add(cross(bot_a, top_b), cross(top_a, bot_b));	
	vect6 ret;
	for (i = 0; i < 3; i++)
	{
		ret.v[i] = top.v[i];
		ret.v[i + 3] = bot.v[i];
	}
	return ret;
}

mat6 spatial_cross_operator(vect6 v)
{
	vect3 a;
	vect3 b;
	int i;
	for (i = 0; i < 3; i++)
	{
		a.v[i] = v.v[i];
		b.v[i] = v.v[i + 3];
	}
	mat6 ret;
	mat3 ax = skew(a);
	mat3 bx = skew(b);
	int r, c;
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
		{
			ret.m[r][c] = ax.m[r][c];
			ret.m[r + 3][c + 3] = ax.m[r][c];
			ret.m[r + 3][c] = bx.m[r][c];
			ret.m[r][c + 3] = 0;
		}
	}
	return ret;
}

mat6 h4_to_X(mat4 ha_b)
{
	mat3 E;
	int r, c;
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
			E.m[r][c] = ha_b.m[r][c];
	}
	vect3 ob;
	for (r = 0; r < 3; r++)
		ob.v[r] = ha_b.m[r][3];
	mat3 rx = skew(ob);
	mat3 rx_E = mat3_mult(rx, E);
	mat6 ret;
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
		{
			ret.m[r][c] = E.m[r][c];
			ret.m[r + 3][c + 3] = E.m[r][c];
			ret.m[r][c + 3] = 0;
		}
	}
	for (r = 3; r < 6; r++)
	{
		for (c = 0; c < 3; c++)
			ret.m[r][c] = rx_E.m[r-3][c];
	}
	return ret;
}
vect6 c_change_spatial(mat4 ha_b, vect6 vb)
{
	vect3 v, w;
	int r,c;
	for (r = 0; r < 3; r++)
	{
		w.v[r] = vb.v[r];
		v.v[r] = vb.v[r + 3];
	}
	mat3 E;
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
			E.m[r][c] = ha_b.m[r][c];
	}
	vect3 w_ret = mat3_vect3_mult(E, w);
	vect3 ob;
	for (r = 0; r < 3; r++)
		ob.v[r] = ha_b.m[r][3];
	mat3 rx = skew(ob);
	vect3 v_ret = vect3_add(mat3_vect3_mult(rx, mat3_vect3_mult(E, w)), mat3_vect3_mult(E,v));
	vect6 ret;
	for (r = 0; r < 3; r++)
	{
		ret.v[r] = w_ret.v[r];
		ret.v[r + 3] = v_ret.v[r];
	}
	return ret;
}

mat3 mat3_T(mat3 in)
{
	mat3 ret;
	int r, c;
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
		{
			ret.m[r][c] = in.m[c][r];
		}
	}
	return ret;
}


mat3 Rx(float angle)
{
	mat3 ret;
	ret.m[0][0] = 1;	ret.m[0][1] = 0;			ret.m[0][2] = 0;			
	ret.m[1][0] = 0;	ret.m[1][1] = cos(angle);	ret.m[1][2] = -sin(angle);	
	ret.m[2][0] = 0;	ret.m[2][1] = sin(angle);	ret.m[2][2] = cos(angle);		
	return ret;
}
/*Returns rotation about coordinate. 0 = identity*/
mat3 Ry(float angle)
{
	mat3 ret;
	ret.m[0][0] = cos(angle);	ret.m[0][1] = 0;	ret.m[0][2] = sin(angle);	
	ret.m[1][0] = 0;			ret.m[1][1] = 1;	ret.m[1][2] = 0;			
	ret.m[2][0] = -sin(angle);	ret.m[2][1] = 0;	ret.m[2][2] = cos(angle);			
	return ret;
}
/*Returns rotation about coordinate. 0 = identity*/
mat3 Rz(float angle)
{
	mat3 ret;
	ret.m[0][0] = cos(angle);	ret.m[0][1] = -sin(angle);		ret.m[0][2] = 0;
	ret.m[1][0] = sin(angle);	ret.m[1][1] = cos(angle);		ret.m[1][2] = 0;	
	ret.m[2][0] = 0;			ret.m[2][1] = 0;				ret.m[2][2] = 1;	

	return ret;
}

/*returns the inverse of a homogeneous transform type mat4 matrix*/
mat4 ht_inverse(mat4 hin)
{
	mat4 hout;
	int r; int c;
	for (r = 0; r < 3; r++)
	{
		for (c = 0; c < 3; c++)
		{
			hout.m[r][c] = hin.m[c][r];
		}
	}
	hout.m[0][3] = -(hout.m[0][0] * hin.m[0][3] + hout.m[0][1] * hin.m[1][3] + hout.m[0][2] * hin.m[2][3]);
	hout.m[1][3] = -(hout.m[1][0] * hin.m[0][3] + hout.m[1][1] * hin.m[1][3] + hout.m[1][2] * hin.m[2][3]);
	hout.m[2][3] = -(hout.m[2][0] * hin.m[0][3] + hout.m[2][1] * hin.m[1][3] + hout.m[2][2] * hin.m[2][3]);

	hout.m[3][0] = 0; hout.m[3][1] = 0; hout.m[3][2] = 0; hout.m[3][3] = 1.0;
	return hout;
}

/*returns homogeneous transform mat4 matrix which is the rotation 'analge' around the x axis */
mat4 Hx(float angle)
{
	mat4 ret;
	ret.m[0][0] = 1;	ret.m[0][1] = 0;			ret.m[0][2] = 0;			ret.m[0][3] = 0;
	ret.m[1][0] = 0;	ret.m[1][1] = cos(angle);	ret.m[1][2] = -sin(angle);	ret.m[1][3] = 0;
	ret.m[2][0] = 0;	ret.m[2][1] = sin(angle);	ret.m[2][2] = cos(angle);	ret.m[2][3] = 0;
	ret.m[3][0] = 0;	ret.m[3][1] = 0;			ret.m[3][2] = 0;			ret.m[3][3] = 1;
	return ret;
}

/*Returns rotation about coordinate. 0 = identity*/
mat4 Hy(float angle)
{
	mat4 ret;
	ret.m[0][0] = cos(angle);	ret.m[0][1] = 0;	ret.m[0][2] = sin(angle);	ret.m[0][3] = 0;
	ret.m[1][0] = 0;			ret.m[1][1] = 1;	ret.m[1][2] = 0;			ret.m[1][3] = 0;
	ret.m[2][0] = -sin(angle);	ret.m[2][1] = 0;	ret.m[2][2] = cos(angle);	ret.m[2][3] = 0;
	ret.m[3][0] = 0;			ret.m[3][1] = 0;	ret.m[3][2] = 0;			ret.m[3][3] = 1;
	return ret;
}
/*Returns rotation about coordinate. 0 = identity*/
mat4 Hz(float angle)
{
	mat4 ret;
	ret.m[0][0] = cos(angle);	ret.m[0][1] = -sin(angle);		ret.m[0][2] = 0;	ret.m[0][3] = 0;
	ret.m[1][0] = sin(angle);	ret.m[1][1] = cos(angle);		ret.m[1][2] = 0;	ret.m[1][3] = 0;
	ret.m[2][0] = 0;			ret.m[2][1] = 0;				ret.m[2][2] = 1;	ret.m[2][3] = 0;
	ret.m[3][0] = 0;			ret.m[3][1] = 0;				ret.m[3][2] = 0;	ret.m[3][3] = 1;
	return ret;
}
/*Returns vector cross product between 3 vectors A and B*/
vect3 cross(vect3 v_a, vect3 v_b)
{
	vect3 ret;
	ret.v[0] = -v_a.v[2]*v_b.v[1] + v_a.v[1]*v_b.v[2];
	ret.v[1] = v_a.v[2]*v_b.v[0] - v_a.v[0]*v_b.v[2];
	ret.v[2] = -v_a.v[1]*v_b.v[0] + v_a.v[0]*v_b.v[1];
	return ret;
}
/*Returns skew symmetric matrix of 3 vector vin*/
mat3 skew(vect3 vin)
{
	mat3 ret;
	float x = vin.v[0];	float y = vin.v[1];	float z = vin.v[2];
	ret.m[0][0] = 0;	ret.m[0][1] = -z;	ret.m[0][2] = y;
	ret.m[1][0] = z;	ret.m[1][1] = 0;	ret.m[1][2] = -x;
	ret.m[2][0] = -y;	ret.m[2][1] = x;	ret.m[2][2] = 0;
	return ret;
}
/*Multiples vector v_a by scalar scale*/
vect6 vect6_scale(vect6 v_a, float scale)
{
	vect6 ret; int i;
	for (i = 0; i<6; i++)
		ret.v[i] = v_a.v[i] * scale;
	return ret;
}
/*Multiples vector v_a by scalar scale*/
vect4 vect4_scale(vect4 v_a, float scale)
{
	vect4 ret; int i;
	for(i=0;i<4;i++)
		ret.v[i] = v_a.v[i] * scale;
	return ret;
}
/*Multiples vector v_a by scalar scale*/
vect3 vect3_scale(vect3 v_a, float scale)
{
	vect3 ret; int i;
	for (i = 0; i<3; i++)
		ret.v[i] = v_a.v[i] * scale;
	return ret;
}
/*6 vector dot product*/
float vect6_dot(vect6 v_a, vect6 v_b)
{
	float ret = 0;
	int i;
	for (i = 0; i < 6; i++)
		ret += v_a.v[i] * v_b.v[i];
	return ret;
}
/*4 vector dot product (note: for homogeneous transformation matrices, the 4th term is always 1)*/
float vect4_dot(vect4 v_a, vect4 v_b)
{
	float ret = 0;
	int i;
	for (i = 0; i < 4; i++)
		ret += v_a.v[i] * v_b.v[i];
	return ret;
}
/*3 vector dot product*/
float vect3_dot(vect3 v_a, vect3 v_b)
{
	float ret = 0;
	int i;
	for (i = 0; i < 3; i++)
		ret += v_a.v[i] * v_b.v[i];
	return ret;
}

vect6 vect6_add(vect6 v_a, vect6 v_b)
{
	vect6 ret;
	int dim = 6; int i;
	for (i = 0; i < dim; i++)
		ret.v[i] = v_a.v[i] + v_b.v[i];
	return ret;
}

vect4 vect4_add(vect4 v_a, vect4 v_b)
{
	vect4 ret;
	int dim = 4; int i;
	for (i = 0; i < dim; i++)
		ret.v[i] = v_a.v[i] + v_b.v[i];
	return ret;
}

vect6 zero_vect6()
{
	vect6 ret;
	int i;
	for (i = 0; i < 6; i++)
		ret.v[i] = 0;
	return ret;
}

vect3 vect3_add(vect3 v_a, vect3 v_b)
{
	vect3 ret;
	int dim = 3; int i;
	for (i = 0; i < dim; i++)
		ret.v[i] = v_a.v[i] + v_b.v[i];
	return ret;
}
float vect3_magnitude(vect3 v)
{
	return sqrt( v.v[0]*v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2]);
}
vect3 vect3_normalize(vect3 v)
{
	float scale = 1/(sqrt(v.v[0] * v.v[0] + v.v[1] * v.v[1] + v.v[2] * v.v[2]));
	v.v[0] *= scale;
	v.v[1] *= scale;
	v.v[2] *= scale;
	return v;
}
vect6 mat6_vect6_mult(mat6 m, vect6 v)
{
	int rsize = 6; int vsize = 6;
	vect6 ret;
	int r, i;
	for (r = 0; r < rsize; r++)
	{
		float tmp = 0;
		for (i = 0; i < vsize; i++)
		{
			tmp = tmp + m.m[r][i] * v.v[i];
		}
		ret.v[r] = tmp;
	}
	return ret;
}

vect4 mat4_vect4_mult(mat4 m, vect4 v)
{
	int rsize = 4; int vsize = 4;
	vect4 ret;
	int r, i;
	for (r = 0; r < rsize; r++)
	{
		float tmp = 0;
		for (i = 0; i < vsize; i++)
		{
			tmp = tmp + m.m[r][i] * v.v[i];
		}
		ret.v[r] = tmp;
	}
	return ret;
}
vect3 mat4_vect3_mult(mat4 m, vect3 v)
{
	int rsize = 3; int vsize = 4;
	vect3 ret;
	int r, i;
	for (r = 0; r < rsize; r++)
	{
		float tmp = 0;
		for (i = 0; i < vsize; i++)
		{
			if (i < 3)
				tmp = tmp + m.m[r][i] * v.v[i];
			if (i == 3)
				tmp = tmp + m.m[r][i];
		}
		ret.v[r] = tmp;
	}
	return ret;
	return ret;
}

vect3 mat3_vect3_mult(mat3 m, vect3 v)
{
	int rsize = 3; int vsize = 3;
	vect3 ret;
	int r, i;
	for (r = 0; r < rsize; r++)
	{
		float tmp = 0;
		for (i = 0; i < vsize; i++)
		{
			tmp = tmp + m.m[r][i] * v.v[i];
		}
		ret.v[r] = tmp;
	}
	return ret;
}

mat6 mat6_mult(mat6 m1, mat6 m2)
{
	mat6 ret;
	int dim = 6;
	int out_r; int out_c; int i;
	for (out_r = 0; out_r < dim; out_r++)
	{
		for (out_c = 0; out_c < dim; out_c++)
		{
			float tmp = 0;
			for (i = 0; i < dim; i++)
			{
				tmp = tmp + m1.m[out_r][i] * m2.m[i][out_c];
			}
			ret.m[out_r][out_c] = tmp;
		}
	}
	return ret;
}

mat4 mat4_mult(mat4 m1, mat4 m2)
{
	mat4 ret;
	int dim = 4;
	int out_r; int out_c; int i;
	for (out_r = 0; out_r < dim; out_r++)
	{
		for (out_c = 0; out_c < dim; out_c++)
		{
			float tmp = 0;
			for (i = 0; i < dim; i++)
			{
				tmp = tmp + m1.m[out_r][i] * m2.m[i][out_c];
			}
			ret.m[out_r][out_c] = tmp;
		}
	}
	return ret;
}

mat3 mat3_mult(mat3 m1, mat3 m2)
{
	mat3 ret;
	int dim = 3;
	int out_r; int out_c; int i;
	for (out_r = 0; out_r < dim; out_r++)
	{
		for (out_c = 0; out_c < dim; out_c++)
		{
			float tmp = 0;
			for (i = 0; i < dim; i++)
			{
				tmp = tmp + m1.m[out_r][i] * m2.m[i][out_c];
			}
			ret.m[out_r][out_c] = tmp;
		}
	}
	return ret;
}

mat6 mat6_add(mat6 m1, mat6 m2)
{
	mat6 ret;
	int dim = 6;
	int r, c;
	for (r = 0; r < dim; r++)
	{
		for (c = 0; c < dim; c++)
		{
			ret.m[r][c] = m1.m[r][c] + m2.m[r][c];
		}
	}
	return ret;
}

mat4 mat4_add(mat4 m1, mat4 m2)
{
	mat4 ret;
	int dim = 4;
	int r, c;
	for (r = 0; r < dim; r++)
	{
		for (c = 0; c < dim; c++)
		{
			ret.m[r][c] = m1.m[r][c] + m2.m[r][c];
		}
	}
	return ret;
}

mat3 mat3_add(mat3 m1, mat3 m2)
{
	mat3 ret;
	int dim = 3;
	int r, c;
	for (r = 0; r < dim; r++)
	{
		for (c = 0; c < dim; c++)
		{
			ret.m[r][c] = m1.m[r][c] + m2.m[r][c];
		}
	}
	return ret;
}



const mat4 mat4_Identity = {
	{
		{ 1, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 1, 0 },
		{ 0, 0, 0, 1 }
	}
};
const mat3 mat3_Identity = {
	{
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 }
	}
};
mat4 mat4_I()
{
	return mat4_Identity;
}
mat3 mat3_I()
{
	return mat3_Identity;
}


void matrix_vect_multiply(float ** M1, int rsize, float * v, int vsize, float * out)
{
	//multiply
	int r, i;
	for (r = 1; r <= rsize; r++)
	{
		float tmp = 0;
		for (i = 1; i <= vsize; i++)
		{
			tmp = tmp + M1[r - 1][i - 1] * v[i];
		}
		out[r] = tmp;
	}
}


vect3 vect4_to_vect3(vect4 in)
{
	vect3 ret;
	int i;
	for (i = 0; i < 3; i++)
		ret.v[i] = in.v[i];
	return ret;
}

vect4 vect3_to_vect4(vect3 in)
{
	vect4 ret;
	int i;
	for (i = 0; i < 3; i++)
		ret.v[i] = in.v[i];
	ret.v[3] = 1;
	return ret;
}