#include <glad/glad.h>

#include <stdio.h>
#include <stdlib.h>

#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "glfw_interface.h"
#include "kinematics.h"

#include "utils.h"

#include "shader-reader.h"

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}

int main(void)
{

	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	int winx = 1200;
	int winy = 800;
	GLFWwindow* window = glfwCreateWindow(winx, winy, "Jesse's Crappy World", NULL, NULL);
	glfwMakeContextCurrent(window);
	if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress))
		return -1;	//glad init failed
	glViewport(0, 0, winx, winy);
	glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);

	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LESS);

	/*setup camera*/
	glm::mat4 CameraProjection = glm::perspective(120.0f, (float)winx / (float)winy, 0.1f, 10000.0f);
	glm::mat4 View = glm::mat4(1.0);
	glm::mat4 Model = glm::mat4(1.0);
	glm::mat4 MVP = CameraProjection * View * Model;
	dh_entry CamDH[CAM_NUM_FRAMES];		//stack memory for forward kinematics
	joint cam_joints[CAM_NUM_FRAMES];
	CamControlStruct Player;				//specialized camera structure. carries around movement parameters
	init_cam(&Player, cam_joints);
	Player.CamRobot.hb_0 = mat4_mult(Hx(PI), mat4_I());
	Player.CamRobot.hw_b = mat4_I();		//END initializing camera
	Player.CamRobot.hw_b.m[0][3] = -20.902f;
	Player.CamRobot.hw_b.m[1][3] = -20.881f;
	Player.CamRobot.hw_b.m[2][3] = 10.917695f;
	Player.CamRobot.j[1].q = 4.414204f;
	Player.CamRobot.j[2].q = -1.540796f;
	Player.lock_in_flag = 0;
	Player.look_at_flag = 0;

	glfwSetCursorPos(window, winx / 2, winy / 2);

	Shader lightingShader("2.1.basic_lighting.vs", "2.1.basic_lighting.fs");
	Shader lampShader("2.1.lamp.vs", "2.1.lamp.fs");

	/**/
	unsigned int cubeVBO, cubeVAO;
	generate_unit_cube(&cubeVBO, &cubeVAO);
	// position attribute
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	// normal attribute
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	// second, configure the light's VAO (VBO stays the same; the vertices are the same for the light object which is also a 3D cube)
	unsigned int lightVAO;
	glGenVertexArrays(1, &lightVAO);
	glBindVertexArray(lightVAO);
	//
	glBindBuffer(GL_ARRAY_BUFFER, cubeVBO);
	// note that we update the lamp's position attribute's stride to reflect the updated buffer data
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	double start_time = glfwGetTime();
	double prev_time = start_time;
	while (!glfwWindowShouldClose(window))
	{
		double time = glfwGetTime();
		double fps = 1.0 / (time - prev_time);
		prev_time = time;

		// render
		// ------
		glClearColor(0, 0, 0, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		mat4 hw_light = mat4_Identity;
		hw_light.m[0][3] = 0;
		hw_light.m[1][3] = -50;
		hw_light.m[2][3] = 30;
		vect3 lp = h_origin(hw_light);
		glm::vec3 lightPos(lp.v[0], lp.v[1], lp.v[2]);

		// be sure to activate shader when setting uniforms/drawing objects
		lightingShader.use();
		lightingShader.setVec3("lightColor", 1.0f, 0.98f, 0.941176471f);
		lightingShader.setVec3("lightPos", lightPos);

		Player.cur_V.v[0] = 0;	Player.cur_V.v[1] = 0;	Player.cur_V.v[2] = 0;
		Player.xyV = 50.0 * 1.f;

		View = keyboard_cam_control(window, &Player, fps, h_origin(mat4_Identity));
		MVP = CameraProjection * View * Model;

		lightingShader.setMat4("projection", CameraProjection);
		lightingShader.setMat4("view", View);

		lightingShader.setVec3("objectColor", 0.8f, 0.8f, 0.8f);


		for (int xc = -100; xc < 100; xc+= 10)
		{
			for (int yc = -100; yc < 100; yc+= 10)
			{
				for (int zc = -100; zc < 100; zc+= 10)
				{
					mat4 hw_cube = { 
						{
							{1, 0, 0, (float)xc},
							{0, 1, 0, (float)yc},
							{0, 0, 1, (float)zc},
							{0, 0, 0, 1.f}
						}
					};
					glm::mat4 model = ht_matrix_to_mat4(hw_cube);
					lightingShader.setVec3("objectColor", .4f, .4f, .4f);
					lightingShader.setMat4("model", model);

					glBindVertexArray(cubeVAO);
					glDrawArrays(GL_TRIANGLES, 0, 36);

				}
			}
		}

		glm::mat4 model;
		lampShader.use();
		lampShader.setMat4("projection", CameraProjection);
		lampShader.setMat4("view", View);
		model = glm::mat4(1.0f);
		model = glm::translate(model, lightPos);
		model = glm::scale(model, glm::vec3(2.0f)); // a smaller cube
		lampShader.setMat4("model", model);

		glBindVertexArray(lightVAO);
		glDrawArrays(GL_TRIANGLES, 0, 36);

		glfwSwapBuffers(window);
		glfwPollEvents();
		
	}
	glfwTerminate();
	return 0;
}