#include "spatialAlgebra.h"
#include "sin_fast.h"
#include "glad/glad.h"
#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>
#include "glfw_interface.h"
#include "shader-reader.h"

void create_prism_voxel(float triangularprism_vertices[8 * 3 * 8])
{

	int vertex_idx = 0;
	float bottom_radius = 0.5;
	float top_radius = 0.25;
	float height = 1.0;
	vect3_t top_vertices[3] = {};
	vect3_t bottom_vertices[3] = {};
	//setup of the base core triangle
	for (int i = 0; i < 3; i++)
	{
		top_vertices[i].v[0] = top_radius * cos((float)i * 120.f * DEG_TO_RAD);
		top_vertices[i].v[1] = top_radius * sin((float)i * 120.f * DEG_TO_RAD);
		top_vertices[i].v[2] = height;
	}
	//setup of the base core triangle
	for (int i = 0; i < 3; i++)
	{
		bottom_vertices[i].v[0] = bottom_radius * cos((float)i * 120.f * DEG_TO_RAD);
		bottom_vertices[i].v[1] = bottom_radius * sin((float)i * 120.f * DEG_TO_RAD);
		bottom_vertices[i].v[2] = 0;
	}


	//base
	for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
	{
		for (int i = 0; i < 3; i++)
		{
			triangularprism_vertices[vertex_idx++] = top_vertices[triangle_idx].v[i];
		}
		triangularprism_vertices[vertex_idx++] = 0.f;
		triangularprism_vertices[vertex_idx++] = 0.f;
		triangularprism_vertices[vertex_idx++] = -1.f;

		triangularprism_vertices[vertex_idx++] = top_vertices[triangle_idx].v[0];
		triangularprism_vertices[vertex_idx++] = top_vertices[triangle_idx].v[1];
	}

	//top
	for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
	{
		for (int i = 0; i < 3; i++)
		{
			triangularprism_vertices[vertex_idx++] = bottom_vertices[triangle_idx].v[i];
		}
		triangularprism_vertices[vertex_idx++] = 0.f;
		triangularprism_vertices[vertex_idx++] = 0.f;
		triangularprism_vertices[vertex_idx++] = -1.f;

		triangularprism_vertices[vertex_idx++] = bottom_vertices[triangle_idx].v[0];
		triangularprism_vertices[vertex_idx++] = bottom_vertices[triangle_idx].v[1];
	}



	{
		vect3_t* varr[3] = { &bottom_vertices[0], &bottom_vertices[1], &top_vertices[0] };
		vect3_t norm = {};
		cross_pbr(varr[0], varr[1], &norm);
		for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
		{
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = varr[triangle_idx]->v[i];
			}
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = norm.v[i];
			}
			triangularprism_vertices[vertex_idx++] = 0;
			triangularprism_vertices[vertex_idx++] = 1;	//placeholder for now
		}
	}

	{
		vect3_t* varr[3] = { &top_vertices[0], &top_vertices[1], &bottom_vertices[1] };
		vect3_t norm = {};
		cross_pbr(varr[0], varr[1], &norm);
		for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
		{
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = varr[triangle_idx]->v[i];
			}
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = norm.v[i];
			}
			triangularprism_vertices[vertex_idx++] = 0;
			triangularprism_vertices[vertex_idx++] = 1;	//placeholder for now
		}
	}

	{
		vect3_t* varr[3] = { &bottom_vertices[1], &bottom_vertices[2], &top_vertices[1] };
		vect3_t norm = {};
		cross_pbr(varr[0], varr[1], &norm);
		for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
		{
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = varr[triangle_idx]->v[i];
			}
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = norm.v[i];
			}
			triangularprism_vertices[vertex_idx++] = 0;
			triangularprism_vertices[vertex_idx++] = 1;	//placeholder for now
		}
	}

	{
		vect3_t* varr[3] = { &top_vertices[1], &top_vertices[2], &bottom_vertices[2] };
		vect3_t norm = {};
		cross_pbr(varr[0], varr[1], &norm);
		for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
		{
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = varr[triangle_idx]->v[i];
			}
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = norm.v[i];
			}
			triangularprism_vertices[vertex_idx++] = 0;
			triangularprism_vertices[vertex_idx++] = 1;	//placeholder for now
		}
	}


	{
		vect3_t* varr[3] = { &bottom_vertices[2], &bottom_vertices[0], &top_vertices[2] };
		vect3_t norm = {};
		cross_pbr(varr[0], varr[1], &norm);
		for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
		{
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = varr[triangle_idx]->v[i];
			}
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = norm.v[i];
			}
			triangularprism_vertices[vertex_idx++] = 0;
			triangularprism_vertices[vertex_idx++] = 1;	//placeholder for now
		}
	}

	{
		vect3_t* varr[3] = { &top_vertices[2], &top_vertices[0], &bottom_vertices[0] };
		vect3_t norm = {};
		cross_pbr(varr[0], varr[1], &norm);
		for (int triangle_idx = 0; triangle_idx < 3; triangle_idx++)
		{
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = varr[triangle_idx]->v[i];
			}
			for (int i = 0; i < 3; i++)
			{
				triangularprism_vertices[vertex_idx++] = norm.v[i];
			}
			triangularprism_vertices[vertex_idx++] = 0;
			triangularprism_vertices[vertex_idx++] = 1;	//placeholder for now
		}
	}

}



void draw_prism(Shader*shader, unsigned int prismVAO)

{
	glBindVertexArray(prismVAO);

	mat4_t hw_prism = {};
	for (int rc = 0; rc < 4; rc++)
		hw_prism.m[rc][rc] = 1.0f;
	hw_prism.m[1][3] = 2.0;
	hw_prism.m[2][3] = 5.0;
	glm::mat4 model = ht_matrix_to_mat4_t(hw_prism);		//this is so fucking wasteful to do it this way holy shit. Maybe rewrite setMat4 for efficiency
	shader->setMat4("model", model);

	glBindVertexArray(prismVAO);
	glDrawArrays(GL_TRIANGLES, 0, 24);
}

