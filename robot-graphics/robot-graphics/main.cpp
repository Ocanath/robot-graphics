#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "stb_header.h"
#include "stb_image.h"

#include "model.h"

#include "glfw_interface.h"
#include "kinematics.h"

#include "utils.h"


#include <iostream>



// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}


// utility function for loading a 2D texture from file
// ---------------------------------------------------
unsigned int loadTexture(char const* path)
{
	unsigned int textureID;
	glGenTextures(1, &textureID);

	int width, height, nrComponents;
	unsigned char* data = stbi_load(path, &width, &height, &nrComponents, 0);
	if (data)
	{
		GLenum format;
		if (nrComponents == 1)
			format = GL_RED;
		else if (nrComponents == 3)
			format = GL_RGB;
		else if (nrComponents == 4)
			format = GL_RGBA;

		glBindTexture(GL_TEXTURE_2D, textureID);
		glTexImage2D(GL_TEXTURE_2D, 0, format, width, height, 0, format, GL_UNSIGNED_BYTE, data);
		glGenerateMipmap(GL_TEXTURE_2D);

		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR_MIPMAP_LINEAR);
		glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

		stbi_image_free(data);
	}
	else
	{
		std::cout << "Texture failed to load at path: " << path << std::endl;
		stbi_image_free(data);
	}

	return textureID;
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
	//glDepthFunc(GL_LESS);

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
	Player.CamRobot.hw_b.m[0][3] = -8.7596f;
	Player.CamRobot.hw_b.m[1][3] = -8.809f;
	Player.CamRobot.hw_b.m[2][3] = 1.04f;
	Player.CamRobot.j[1].q = fmod(1121.107f + PI, 2*PI)-PI;
	Player.CamRobot.j[2].q = -1.338593f;
	Player.lock_in_flag = 0;
	Player.look_at_flag = 0;
	

	glfwSetCursorPos(window, winx / 2, winy / 2);

	Shader lightingShader("6.multiple_lights.vs", "6.multiple_lights.fs");
	Shader lightCubeShader("6.light_cube.vs", "6.light_cube.fs");

	// set up vertex data (and buffer(s)) and configure vertex attributes
// ------------------------------------------------------------------
	float vertices[] = {
		// positions          // normals           // texture coords
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,
		 0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  0.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  1.0f,  1.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  1.0f,
		-0.5f, -0.5f, -0.5f,  0.0f,  0.0f, -1.0f,  0.0f,  0.0f,

		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,
		 0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  1.0f,  1.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  1.0f,
		-0.5f, -0.5f,  0.5f,  0.0f,  0.0f,  1.0f,  0.0f,  0.0f,

		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		-0.5f, -0.5f, -0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		-0.5f, -0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
		-0.5f,  0.5f,  0.5f, -1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,
		 0.5f,  0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  1.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		 0.5f, -0.5f, -0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  1.0f,
		 0.5f, -0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  0.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  1.0f,  0.0f,  0.0f,  1.0f,  0.0f,

		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,
		 0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  1.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
		 0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  1.0f,  0.0f,
		-0.5f, -0.5f,  0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  0.0f,
		-0.5f, -0.5f, -0.5f,  0.0f, -1.0f,  0.0f,  0.0f,  1.0f,

		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f,
		 0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  1.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
		 0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  1.0f,  0.0f,
		-0.5f,  0.5f,  0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  0.0f,
		-0.5f,  0.5f, -0.5f,  0.0f,  1.0f,  0.0f,  0.0f,  1.0f
	};

	// first, configure the cube's VAO (and VBO)
	unsigned int VBO, cubeVAO;
	glGenVertexArrays(1, &cubeVAO);
	glGenBuffers(1, &VBO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

	glBindVertexArray(cubeVAO);
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);
	glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(3 * sizeof(float)));
	glEnableVertexAttribArray(1);
	glVertexAttribPointer(2, 2, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)(6 * sizeof(float)));
	glEnableVertexAttribArray(2);
	// second, configure the light's VAO (VBO stays the same; the vertices are the same for the light object which is also a 3D cube)
	unsigned int lightCubeVAO;
	glGenVertexArrays(1, &lightCubeVAO);
	glBindVertexArray(lightCubeVAO);

	glBindBuffer(GL_ARRAY_BUFFER, VBO);
	// note that we update the lamp's position attribute's stride to reflect the updated buffer data
	glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 8 * sizeof(float), (void*)0);
	glEnableVertexAttribArray(0);

	unsigned int container_diffuseMap = loadTexture("img/container2.png");
	unsigned int container_specularMap = loadTexture("img/container2_specular.png");
	unsigned int stone_diffuse_map = loadTexture("img/large_stone_tiled.png");
	unsigned int stone_specular_map = loadTexture("img/large_stone_tiled_specular.png");

	lightingShader.use();
	lightingShader.setInt("material.diffuse", 0);
	lightingShader.setInt("material.specular", 1);

	// tell stb_image.h to flip loaded texture's on the y-axis (before loading model).
	stbi_set_flip_vertically_on_load(true);
	Shader model_shader("1.model_loading.vs", "1.model_loading.fs");
	//Model mf("misc_models/backpack.obj");
	AssetModel ourModel("misc_models/backpack/backpack.obj");

	AssetModel psyhand_thumbcap("misc_models/psyonic-hand/thumb-F2.STEP");


	double start_time = glfwGetTime();
	double prev_time = start_time;
	
	while (!glfwWindowShouldClose(window))
	{
		double time = glfwGetTime();
		double fps = 1.0 / (time - prev_time);
		prev_time = time;

		// render
		// ------
		glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		lightingShader.use();
		vect3 cam_origin = h_origin(Player.CamRobot.hw_b);
		glm::vec3 camera_position = glm::vec3(cam_origin.v[0], cam_origin.v[1], cam_origin.v[2]);
		lightingShader.setVec3("viewPos", camera_position);
		lightingShader.setFloat("material.shininess", 32.0f);
		glm::vec3 point_light_positions[4] = {
			{-9,-9,9},
			{-9,9,9},
			{9,-9,9},
			{9,9,9}
		};
		const float quadratic = 0.028f;

		lightingShader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
		lightingShader.setVec3("dirLight.ambient", 0.05f, 0.05f, 0.05f);
		lightingShader.setVec3("dirLight.diffuse", 0.4f, 0.4f, 0.4f);
		lightingShader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);
		// point light 1
		lightingShader.setVec3("pointLights[0].position", point_light_positions[0]);
		lightingShader.setVec3("pointLights[0].ambient", 0.05f, 0.05f, 0.05f);
		lightingShader.setVec3("pointLights[0].diffuse", 0.8f, 0.8f, 0.8f);
		lightingShader.setVec3("pointLights[0].specular", 1.0f, 1.0f, 1.0f);
		lightingShader.setFloat("pointLights[0].constant", 1.0f);
		lightingShader.setFloat("pointLights[0].linear", 0.09);
		lightingShader.setFloat("pointLights[0].quadratic", quadratic);
		// point light 2
		lightingShader.setVec3("pointLights[1].position", point_light_positions[1]);
		lightingShader.setVec3("pointLights[1].ambient", 0.05f, 0.05f, 0.05f);
		lightingShader.setVec3("pointLights[1].diffuse", 0.8f, 0.8f, 0.8f);
		lightingShader.setVec3("pointLights[1].specular", 1.0f, 1.0f, 1.0f);
		lightingShader.setFloat("pointLights[1].constant", 1.0f);
		lightingShader.setFloat("pointLights[1].linear", 0.09);
		lightingShader.setFloat("pointLights[1].quadratic", quadratic);
		// point light 3
		lightingShader.setVec3("pointLights[2].position", point_light_positions[2]);
		lightingShader.setVec3("pointLights[2].ambient", 0.05f, 0.05f, 0.05f);
		lightingShader.setVec3("pointLights[2].diffuse", 0.8f, 0.8f, 0.8f);
		lightingShader.setVec3("pointLights[2].specular", 1.0f, 1.0f, 1.0f);
		lightingShader.setFloat("pointLights[2].constant", 1.0f);
		lightingShader.setFloat("pointLights[2].linear", 0.09);
		lightingShader.setFloat("pointLights[2].quadratic", quadratic);
		// point light 4
		lightingShader.setVec3("pointLights[3].position", point_light_positions[3]);
		lightingShader.setVec3("pointLights[3].ambient", 0.05f, 0.05f, 0.05f);
		lightingShader.setVec3("pointLights[3].diffuse", 0.8f, 0.8f, 0.8f);
		lightingShader.setVec3("pointLights[3].specular", 1.0f, 1.0f, 1.0f);
		lightingShader.setFloat("pointLights[3].constant", 1.0f);
		lightingShader.setFloat("pointLights[3].linear", 0.09);
		lightingShader.setFloat("pointLights[3].quadratic", quadratic);

		// spotLight
		//lightingShader.setVec3("spotLight.position", camera_position);
		//lightingShader.setVec3("spotLight.direction", glm::vec3(1,0,0) );
		//lightingShader.setVec3("spotLight.ambient", 0.0f, 0.0f, 0.0f);
		//lightingShader.setVec3("spotLight.diffuse", 1.0f, 1.0f, 1.0f);
		//lightingShader.setVec3("spotLight.specular", 1.0f, 1.0f, 1.0f);
		//lightingShader.setFloat("spotLight.constant", 1.0f);
		//lightingShader.setFloat("spotLight.linear", 0.09);
		//lightingShader.setFloat("spotLight.quadratic", 0.032);
		//lightingShader.setFloat("spotLight.cutOff", glm::cos(glm::radians(12.5f)));
		//lightingShader.setFloat("spotLight.outerCutOff", glm::cos(glm::radians(15.0f)));

		View = keyboard_cam_control(window, &Player, fps, h_origin(mat4_Identity));
		MVP = CameraProjection * View * Model;

		lightingShader.setMat4("projection", CameraProjection);
		lightingShader.setMat4("view", View);

		lightingShader.setVec3("objectColor", 0.8f, 0.8f, 0.8f);

		// world transformation
		glm::mat4 model = glm::mat4(1.0f);
		lightingShader.setMat4("model", model);

		// bind diffuse map
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, stone_diffuse_map);
		// bind specular map
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, stone_specular_map);

		glBindVertexArray(cubeVAO);

		const float bcd = 10;
		float scf = bcd*2;
		const float zoff = 10.f;
		mat4 hw_cube[6] =
		{
			{
				{
					{1, 0, 0, -bcd},
					{0, scf, 0, 0.f},
					{0, 0, scf, 0.f},
					{0, 0, 0, 1.f}
				}
			},
			{
				{
					{1, 0, 0, bcd},
					{0, scf, 0, 0.f},
					{0, 0, scf, 0.f},
					{0, 0, 0, 1.f}
				}
			},
			{
				{
					{scf, 0, 0, 0.f},
					{0, 1, 0, -bcd},
					{0, 0, scf, 0.f},
					{0, 0, 0, 1.f}
				}
			},
			{
				{
					{scf, 0, 0, 0.f},
					{0, 1, 0, bcd},
					{0, 0, scf, 0.f},
					{0, 0, 0, 1.f}
				}
			},
			{
				{
					{scf, 0, 0, 0.f},
					{0, scf, 0, 0.f},
					{0, 0, 1, -bcd},
					{0, 0, 0, 1.f}
				}
			},
			{
				{
					{scf, 0, 0, 0.f},
					{0, scf, 0, 0.f},
					{0, 0, 1, bcd},
					{0, 0, 0, 1.f}
				}
			}
		};
		for (int i = 0; i < 6; i++)
		{
			hw_cube[i].m[2][3] += zoff;
			glm::mat4 model = ht_matrix_to_mat4(hw_cube[i]);
			lightingShader.setMat4("model", model);

			glBindVertexArray(cubeVAO);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}

		// don't forget to enable shader before setting uniforms
		model_shader.use();

		// view/projection transformations
		model_shader.setMat4("projection", CameraProjection);
		model_shader.setMat4("view", View);
		// render the loaded model
		//model = glm::mat4(1.0f);
		//model = glm::translate(model, glm::vec3(0.0f, 0.0f, 5.f)); // translate it down so it's at the center of the scene
		//model = glm::scale(model, glm::vec3(1.0f, 1.0f, 1.0f));	// it's a bit too big for our scene, so scale it down
		scf = .333f;
		mat4 backpack_htm = {
			{
				{scf, 0, 0, 0},
				{0, scf, 0, 0},
				{0, 0, scf, 0},
				{0, 0, 0, 1}
			}
		};
		backpack_htm = mat4_mult(backpack_htm, Hx(PI/2.f));
		backpack_htm.m[2][3] = 20.5f;
		backpack_htm.m[2][3] = 1.5f;
		backpack_htm = mat4_mult(backpack_htm, Hy(time));
		model = ht_matrix_to_mat4(backpack_htm);
		model_shader.setMat4("model", model);
		ourModel.Draw(model_shader);


		{
			scf = 1.f;
			mat4 tf = {
				{
					{scf, 0, 0, 0},
					{0, scf, 0, 0},
					{0, 0, scf, 0},
					{0, 0, 0, 1}
				}
			};
			tf.m[2][3] = 5.f;
			model = ht_matrix_to_mat4(tf);
		}
		model_shader.setMat4("model", model);
		psyhand_thumbcap.Draw(model_shader);



		//for (int xc = -5; xc < 5; xc+= 1)
		//{
		//	for (int yc = -5; yc < 5; yc+= 1)
		//	{
		//		for (int zc = -5; zc < 5; zc+= 1)
		//		{
		//			mat4 hw_cube = { 
		//				{
		//					{.33, 0, 0, (float)xc},
		//					{0, .33, 0, (float)yc},
		//					{0, 0, .33, (float)zc},
		//					{0, 0, 0, 1.f}
		//				}
		//			};
		//			glm::mat4 model = ht_matrix_to_mat4(hw_cube);
		//			//lightingShader.setVec3("objectColor", .4f, .4f, .4f);
		//			
		//			lightingShader.setMat4("model", model);
		//			glBindVertexArray(cubeVAO);
		//			glDrawArrays(GL_TRIANGLES, 0, 36);
		//		}
		//	}
		//}





		// also draw the lamp object(s)
		lightCubeShader.use();
		lightCubeShader.setMat4("projection", CameraProjection);
		lightCubeShader.setMat4("view", View);

		// we now draw as many light bulbs as we have point lights.
		glBindVertexArray(lightCubeVAO);
		for (unsigned int i = 0; i < 4; i++)
		{
			glm::mat4 model = glm::mat4(1.0f);
			model = glm::translate(model, point_light_positions[i]);
			model = glm::scale(model, glm::vec3(0.2f)); // Make it a smaller cube
			lightCubeShader.setMat4("model", model);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(window, true);
	}
	glfwTerminate();
	return 0;
}