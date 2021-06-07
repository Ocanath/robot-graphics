#include "RenderBullet.h"


RenderBulletObject::RenderBulletObject()
{
	vect3 cube_color = { { 1., 1., 1. } };
	vect3 cube_dim = { { 0.5, 0.5, 0.5 } };
	mat4 hw_cube = { {
		{1,0,0,0},
		{0,1,0,0},
		{0,0,1,0},
		{0,0,0,1}
	} };
	body = NULL;
}

void RenderBulletObject::render(glm::mat4 CameraProjection, glm::mat4 View, Shader & shader, unsigned int cubeVAO)
{
	btTransform trans;
	body->getMotionState()->getWorldTransform(trans);
	vect3 origin = { { trans.getOrigin().getX(), trans.getOrigin().getY(), trans.getOrigin().getZ() } };
	btQuaternion btQ = trans.getRotation();
	vect4 quat = { { btQ.getW(), btQ.getX(), btQ.getY(), btQ.getZ() } };
	hw_cube = quat_to_mat4(quat, origin);

	for (int c = 0; c < 3; c++)
	{
		for (int r = 0; r < 3; r++)
		{
			hw_cube.m[r][c] *= cube_dim.v[c];
		}
	}

	glm::mat4 model = ht_matrix_to_mat4(hw_cube);
	shader.setMat4("model", model);

	glBindVertexArray(cubeVAO);
	glDrawArrays(GL_TRIANGLES, 0, 36);
}

//RenderBulletCube::~RenderBulletCube()
//{
//	delete col_shape;
//}