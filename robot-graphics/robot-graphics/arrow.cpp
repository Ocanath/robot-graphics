#include "arrow.h"



glm::mat4 pm4_to_glmm4(mat4_t * H)
{
	double HC_W_T[16];
	int r, c;
	for (r = 0; r < 4; r++)
	{
		for (c = 0; c < 4; c++)
		{
			HC_W_T[4 * c + r] = H->m[r][c];
		}
	}
	return glm::make_mat4(HC_W_T);
}

void draw_arrow(AssetModel * arrow, Shader * shader, mat4_t * hw_arrow)
{
	glm::mat4 model = pm4_to_glmm4(hw_arrow);
	shader->setMat4("model", model);
	arrow->Draw(*shader, NULL);
}

void draw_arrow_between_two_points(vect3_t * p1, vect3_t * p2, AssetModel * arrow, Shader * shader, float thinfactor)
{
	vect3_t dif;
	for (int i = 0; i < 3; i++)
		dif.v[i] = p2->v[i] - p1->v[i];
	mat4_t hw_arrow = {};
	hw_arrow.m[3][3] = 1;
	//comment if it disappears
	for(int r = 0; r < 3; r++)
		hw_arrow.m[r][3] = p1->v[r];

	for (int r = 0; r < 3; r++)
		hw_arrow.m[r][2] = dif.v[r];

	vect3_t ref_vector;
	for (int r = 0; r < 3; r++)
		ref_vector.v[r] = dif.v[2 - r]+.1;	//load reference. easy method to make sure the cross product cannot ever be zero
	vect3_t res;
	cross_pbr(&dif, &ref_vector, &res);	
	vect_normalize(res.v, 3);
	for (int r = 0; r < 3; r++)
		hw_arrow.m[r][1] = res.v[r]*thinfactor;

	cross_pbr(&res, &dif, &ref_vector);	//load into ref_vector to avoid having to copy res into something else
	vect_normalize(ref_vector.v, 3);
	for (int r = 0; r < 3; r++)
		hw_arrow.m[r][0] = ref_vector.v[r]*thinfactor;

	draw_arrow(arrow, shader, &hw_arrow);
}
