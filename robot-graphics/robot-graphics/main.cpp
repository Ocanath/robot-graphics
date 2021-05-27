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
#include "finger-kinematics.h"

#include "utils.h"


#include <iostream>

#define NUM_LIGHTS 5

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

	//AssetModel psyhand_thumbcap("misc_models/psyonic-hand/thumb-F2.STL");
	vector<AssetModel> psy_thumb_modellist;		//vector of thumb stl's
	psy_thumb_modellist.push_back(AssetModel("misc_models/psyonic-hand/thumb-C0.STL"));
	psy_thumb_modellist.push_back(AssetModel("misc_models/psyonic-hand/thumb-F1.STL"));
	psy_thumb_modellist.push_back(AssetModel("misc_models/psyonic-hand/thumb-F2.STL"));
	vector<AssetModel> psy_finger_modellist;	//vector of finger stl's
	psy_finger_modellist.push_back(AssetModel("misc_models/psyonic-hand/idx-F0.STL"));
	psy_finger_modellist.push_back(AssetModel("misc_models/psyonic-hand/idx-F1.STL"));
	psy_finger_modellist.push_back(AssetModel("misc_models/psyonic-hand/idx-F2.STL"));

	kinematic_hand_t psy_hand_bones;
	init_finger_kinematics(&psy_hand_bones);


	double start_time = glfwGetTime();
	double prev_time = start_time;

	glm::vec3 point_light_positions[NUM_LIGHTS] = {
		{-9,-9,9},
		{-9,9,9},
		{9,-9,9},
		{9,9,9},
		{0,0,6}
	};

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
		const float quadratic = 0.028f;

		lightingShader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
		lightingShader.setVec3("dirLight.ambient", 0.05f, 0.05f, 0.05f);
		lightingShader.setVec3("dirLight.diffuse", 0.4f, 0.4f, 0.4f);
		lightingShader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);
		
		for (int i = 0; i < NUM_LIGHTS; i++)
		{
			char buf[32] = { 0 };
			sprintf_s(buf, "pointLights[%d].position", i);
			lightingShader.setVec3(buf, point_light_positions[i]);

			sprintf_s(buf, "pointLights[%d].ambient", i);
			lightingShader.setVec3(buf, 0.05f, 0.05f, 0.05f);

			sprintf_s(buf, "pointLights[%d].diffuse", i);
			lightingShader.setVec3(buf, 0.8f, 0.8f, 0.8f);

			sprintf_s(buf, "pointLights[%d].specular", i);
			lightingShader.setVec3(buf, 1.0f, 1.0f, 1.0f);

			sprintf_s(buf, "pointLights[%d].constant", i);
			lightingShader.setFloat(buf, 1.0f);

			sprintf_s(buf, "pointLights[%d].linear", i);
			lightingShader.setFloat(buf, 0.09);

			sprintf_s(buf, "pointLights[%d].quadratic", i);
			lightingShader.setFloat(buf, quadratic);			
		}

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
		
		vect3 player_pos;
		for(int r = 0; r < 3; r++)
			player_pos.v[r] = Player.CamRobot.hw_b.m[r][3];
		point_light_positions[4] = glm::vec3(player_pos.v[0], player_pos.v[1], player_pos.v[2] + 4.f);

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
		//model_shader.use();// view/projection transformations
		//model_shader.setMat4("projection", CameraProjection);
		//model_shader.setMat4("view", View);

		scf = .333f;
		mat4 backpack_htm = {
			{
				{scf, 0, 0, 0},
				{0, scf, 0, 0},
				{0, 0, scf, 0},
				{0, 0, 0, 1}
			}
		};
		mat4 res;
		
		mat4 rot = Hx(PI / 2.f);
		mat4_mult_pbr(&backpack_htm, &rot, &res);
		copy_mat4(&backpack_htm, &res);
		backpack_htm.m[1][3] = 7.5f;
		backpack_htm.m[2][3] = 1.5f;
		rot = Hy(time);
		mat4_mult_pbr(&backpack_htm, &rot, &res);
		copy_mat4(&backpack_htm, &res);
		model = ht_matrix_to_mat4(backpack_htm);
		lightingShader.setMat4("model", model);
		ourModel.Draw(lightingShader);

		//do the math for the psyonic hand
		float tr_angle = (-15.f - 5.f * sin(time));	//in degrees
		float tf_angle = (15.f + 5.f * sin(time));	//in degrees
		psy_hand_bones.finger[0].chain[1].q = 15.f;
		psy_hand_bones.finger[1].chain[1].q = 15.f;
		psy_hand_bones.finger[2].chain[1].q = 15.f;
		psy_hand_bones.finger[3].chain[1].q = 15.f;
		psy_hand_bones.finger[4].chain[1].q = ((180 + 10.82) + tr_angle) * PI / 180.f;
		psy_hand_bones.finger[4].chain[2].q = (-19.7f - tf_angle) * PI / 180.f;
		finger_kinematics(&psy_hand_bones);
		//render the psyonic hand
		scf = 0.05f;
		mat4 tf = {
			{
				{scf, 0, 0, 0},
				{0, scf, 0, 0},
				{0, 0, scf, 0},
				{0, 0, 0, 1}
			}
		};
		mat4 hw_b = Hx(PI / 2);
		hw_b = mat4_mult(tf, hw_b);
		hw_b.m[2][3] = 1.5f;
		{
			{//THUMB RENDER
				mat4 hb_0 = mat4_I();	//for fingers, this is NOT the identity. Load it into the 0th entry of the joint Him1_i matrix
				mat4 hw_0 = mat4_mult(hw_b, hb_0);

				model = ht_matrix_to_mat4(hw_0);
				lightingShader.setMat4("model", model);
				psy_thumb_modellist[0].Draw(lightingShader);

				for (int i = 1; i <= 2; i++)
				{
					mat4 h0_i = psy_hand_bones.finger[4].chain[i].h0_i;
					mat4 hw_i = mat4_mult(hw_0, h0_i);
					model = ht_matrix_to_mat4(hw_i);
					lightingShader.setMat4("model", model);
					psy_thumb_modellist[i].Draw(lightingShader);
				}
			}
			for(int fidx = 0; fidx < 1; fidx++)
			{//FINGER RENDER
				mat4 hb_0 = psy_hand_bones.finger[fidx].chain[0].him1_i;	//store it here
				mat4 hw_0 = mat4_mult(hw_b, hb_0);

				model = ht_matrix_to_mat4(hw_0);
				lightingShader.setMat4("model", model);
				psy_finger_modellist[0].Draw(lightingShader);

				for (int i = 1; i <= 2; i++)
				{
					mat4 h0_i = psy_hand_bones.finger[fidx].chain[i].h0_i;
					mat4 hw_i = mat4_mult(hw_0, h0_i);
					model = ht_matrix_to_mat4(hw_i);
					lightingShader.setMat4("model", model);
					psy_finger_modellist[i].Draw(lightingShader);
				}
			}
		}

		// also draw the lamp object(s)
		lightCubeShader.use();
		lightCubeShader.setMat4("projection", CameraProjection);
		lightCubeShader.setMat4("view", View);

		// we now draw as many light bulbs as we have point lights.
		glBindVertexArray(lightCubeVAO);
		for (unsigned int i = 0; i < NUM_LIGHTS; i++)
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