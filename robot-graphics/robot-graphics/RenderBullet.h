#ifndef RENDER_BULLET_H
#define RENDER_BULLET_H
#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "glfw_interface.h"
#include "shader-reader.h"
#include "model.h"

#include "kinematics.h"
#include "btBulletDynamicsCommon.h"
#include <thread>
#include <iostream>

class RenderBulletObject
{
private:

public:
	btRigidBody* body;
	vect3 cube_dim;
	mat4 hw_cube;
	AssetModel * modelref;
	float dyn_to_render_f; //sim coordinates might not be the same as render for calculability reasons
	RenderBulletObject();
	//~RenderBulletCube();
	void render(Shader& shader);
};

//class RenderBulletSphere
//{
//public:
//	btRigidBody * body;
//	vect3 color;
//	float radius;
//	mat4 hw_obj;
//	void render(glm::mat4 CameraProjection, glm::mat4 View, Shader & shader);
//};

#endif