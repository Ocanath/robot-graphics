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

/**/
typedef struct light_params_t
{
	glm::vec3 position;
	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
	float constant;
	float linear;
	float quadratic;
}light_params_t;


void transform_mpos_to_kpos(float qin[6], kinematic_hand_t * hand)
{
	enum { THR = 5, THF = 4 };	//thumb mapping enum
	for (int finger = 0; finger < 4; finger++)
	{
		float fangle = (qin[finger] + 4.84f) * PI / 180.f;
		hand->finger[finger].chain[1].q = fangle;
	}
	hand->finger[4].chain[1].q = ((180 + 10.82) + qin[THR]) * PI / 180.f;
	hand->finger[4].chain[2].q = (-19.7f - qin[THF]) * PI / 180.f;
}

int main(void)
{	
	glfwInit();
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 3);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	int winx = 1200;
	int winy = 800;
	//GLFWmonitor* m = glfwGetPrimaryMonitor();
	//int xpos, ypos;
	//glfwGetMonitorWorkarea(m, &xpos, &ypos, &winx, &winy);
	GLFWmonitor* m = NULL;
	GLFWwindow* window = glfwCreateWindow(winx, winy, "Jesse's Crappy World", m, NULL);
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
	Player.CamRobot.hw_b.m[0][3] = -2.18f;
	Player.CamRobot.hw_b.m[1][3] = 0.73f;
	Player.CamRobot.hw_b.m[2][3] = 2.08f;
	Player.CamRobot.j[1].q = fmod(-3.08f + PI, 2*PI)-PI;
	Player.CamRobot.j[2].q = -1.73f;
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
	AssetModel psy_palm("misc_models/psyonic-hand/PALM_BASE_FRAME.STL");

	kinematic_hand_t psy_hand_bones;
	init_finger_kinematics(&psy_hand_bones);
	float q[6] = { 15,15,15,15, 0,-90 };
	transform_mpos_to_kpos(q, &psy_hand_bones);

	double start_time = glfwGetTime();
	double prev_time = start_time;

	float xw, yw, zh;
	xw = 9.f;
	yw = 9.f;
	zh = 3.f;
	light_params_t light[NUM_LIGHTS];
	for (uint8_t i = 0; i < 4; i++)
	{
		float xsign = (i & 1) ? -1.f : 1.f;
		float ysign = (i & 2) ? -1.f : 1.f;
		light[i].position = glm::vec3(xw * xsign, yw * ysign, zh);
	}
	light[4].position = glm::vec3(0, 0, 6);
	for (int i = 0; i < NUM_LIGHTS; i++)
	{
		light[i].ambient = glm::vec3(0.07f, 0.07f, 0.07f);
		light[i].diffuse = glm::vec3(0.7f, 0.7f, 0.7f);
		light[i].specular = glm::vec3(0.9f, 0.9f, 0.9f);
		light[i].constant = 1.f;
		light[i].linear = .09f;
		light[i].quadratic = .028f;
	}
	float shininess = 32.f;

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
		lightingShader.setFloat("material.shininess", shininess);
		

		lightingShader.setVec3("dirLight.direction", -0.2f, -1.0f, -0.3f);
		lightingShader.setVec3("dirLight.ambient", 0.05f, 0.05f, 0.05f);
		lightingShader.setVec3("dirLight.diffuse", 0.4f, 0.4f, 0.4f);
		lightingShader.setVec3("dirLight.specular", 0.5f, 0.5f, 0.5f);
		
		for (int i = 0; i < NUM_LIGHTS; i++)
		{
			char buf[32] = { 0 };
			sprintf_s(buf, "pointLights[%d].position", i);
			lightingShader.setVec3(buf, light[i].position);

			sprintf_s(buf, "pointLights[%d].ambient", i);
			lightingShader.setVec3(buf, light[i].ambient);

			sprintf_s(buf, "pointLights[%d].diffuse", i);
			lightingShader.setVec3(buf, light[i].diffuse);

			sprintf_s(buf, "pointLights[%d].specular", i);
			lightingShader.setVec3(buf, light[i].specular);

			sprintf_s(buf, "pointLights[%d].constant", i);
			lightingShader.setFloat(buf, light[i].constant);

			sprintf_s(buf, "pointLights[%d].linear", i);
			lightingShader.setFloat(buf, light[i].linear);

			sprintf_s(buf, "pointLights[%d].quadratic", i);
			lightingShader.setFloat(buf, light[i].quadratic);			
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
		
		//vect3 player_pos;
		//for(int r = 0; r < 3; r++)
		//	player_pos.v[r] = Player.CamRobot.hw_b.m[r][3];
		//point_light_positions[4] = glm::vec3(player_pos.v[0], player_pos.v[1], player_pos.v[2] + 4.f);


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
		backpack_htm.m[0][3] = 0;
		backpack_htm.m[1][3] = 8.f;
		backpack_htm.m[2][3] = 1.3f;
		model = ht_matrix_to_mat4(backpack_htm);
		lightingShader.setMat4("model", model);
		ourModel.Draw(lightingShader);

		/*Make the light follow the backpack*/
		//light[4].position = glm::vec3(backpack_htm.m[0][3], backpack_htm.m[1][3], backpack_htm.m[2][3] + 1.f);
		if (glfwGetKey(window, GLFW_KEY_R))
		{
			q[0] = 60.f;
			q[1] = 60.f;
			q[2] = 60.f;
			q[3] = 60.f;
			q[5] = -50.f;
			q[4] = 10.f;
		}
		//do the math for the psyonic hand
		transform_mpos_to_kpos(q, &psy_hand_bones);
		finger_kinematics(&psy_hand_bones);
		
		vect3 o_idx_b;
		htmatrix_vect3_mult(&psy_hand_bones.finger[0].chain[0].him1_i, &psy_hand_bones.finger[0].ef_pos_0, &o_idx_b);	//wow. wordy
		vect3 o_thumb_b = psy_hand_bones.finger[4].ef_pos_0;

		float tau[3] = { 0,0,0 };	//num joints + 1

		vect6 f = { 0,0,0, 0, 0, 0 };

		vect3 thumb_force = vect3_add(o_idx_b, vect3_scale(o_thumb_b, -1.f));	//Thumb force IN THE THUMB 0 FRAME/BASE FRAME. NO CHANGE OF FRAME NECESSARY

		for (int i = 0; i < 4; i++)
		{
			vect3 o_f_b;
			htmatrix_vect3_mult(&psy_hand_bones.finger[i].chain[0].him1_i, &psy_hand_bones.finger[i].ef_pos_0, &o_f_b);	//wow. wordy
			vect3 finger_force_b = vect3_add(o_thumb_b, vect3_scale(o_f_b, -1.f));	//idx force IN THE BASE FRAME. FRAME CHANGE NECESSARY

			//float xsq = finger_force_b.v[0] * finger_force_b.v[0];
			float xf = finger_force_b.v[0] - 10.f;
			if (xf < 3.16227766f)
				xf = 3.16227766f;
			xf = 1.f / (xf*xf);

			float shirk = 0.f;
			float zc = finger_force_b.v[2] - 2.f;
			if (zc < 0.f)
				shirk = -zc * zc * .1f * xf;
			
			//float box_xdim = 15.f;
			//if (finger_force_b.v[0] < -box_xdim || finger_force_b.v[0] > box_xdim)
			//	shirk = 0.f;

			//float shirk = -.07f * (5.f - dist_exp);
			//if (shirk > 0.f)
			//	shirk = 0.f;
			//float shirk = 0;
			//shirk += over_component;
			q[i] += shirk;
		}


		//mat4 hidx0_b = ht_inverse(psy_hand_bones.finger[0].chain[0].him1_i);	//consider loading in the other unoccupied 0 frame transform...?
		//for(int r = 0; r <3; r++)
		//	hidx0_b.m[r][3] = 0;
		//vect3 idx_force_0;
		//htmatrix_vect3_mult(&hidx0_b, &idx_force_b, &idx_force_0);
		///*Apply index finger force*/
		//for (int r = 0; r < 3; r++)
		//	f.v[r + 3] = .003f*idx_force_0.v[r];
		//calc_tau(psy_hand_bones.finger[0].chain, 2, f, tau);

		/*Apply Thumb force*/
		for (int r = 0; r < 3; r++)
			f.v[r + 3] = .0003f*thumb_force.v[r];
		calc_tau(psy_hand_bones.finger[4].chain, 2, f, tau);
		q[5] += tau[1];
		q[4] -= tau[2];
		for ( int i = 0; i < 5; i++)
		{
			if (q[i] < 0.f)
				q[i] = 0.f;
			if (q[i] > 90.f)
				q[i] = 90.f;
		}
		if (q[5] > 0.f)
			q[5]= 0.f;
		if (q[5] < -110.f)
			q[5] = -110.f;

		if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
		{
		}

		//render the psyonic hand
		lightingShader.setVec3("objectColor", 2.0f, 2.0f, 2.0f);
		//lightingShader.setFloat("material.shininess", 32.0f);
		scf = 0.01f;
		mat4 tf = {
			{
				{scf, 0, 0, 0},
				{0, scf, 0, 0},
				{0, 0, scf, 0},
				{0, 0, 0, 1}
			}
		};

		//mat4 hw_b = Hx(PI / 2 + .2*sin(time) );
		//hw_b = mat4_mult(hw_b, Hy(.2*sin(time + 1)));
		//hw_b = mat4_mult(hw_b, Hz(.2*sin(time + 2)));
		mat4 hw_b = Hx(PI / 2);
		hw_b = mat4_mult(tf, hw_b);
		//hw_b.m[2][3] = 1.5f+.1*(.5f*sin(time+3)+.5f);
		hw_b.m[2][3] = 1.5f;
		{
			{//THUMB RENDER
				mat4 hb_0 = mat4_I();	//for fingers, this is NOT the identity. Load it into the 0th entry of the joint Him1_i matrix
				mat4 hw_0 = mat4_mult(hw_b, hb_0);

				model = ht_matrix_to_mat4(hw_0);
				lightingShader.setMat4("model", model);
				psy_thumb_modellist[0].Draw(lightingShader);
				psy_palm.Draw(lightingShader);	//render the palm too
				for (int i = 1; i <= 2; i++)
				{
					mat4 h0_i = psy_hand_bones.finger[4].chain[i].h0_i;
					mat4 hw_i = mat4_mult(hw_0, h0_i);
					model = ht_matrix_to_mat4(hw_i);
					lightingShader.setMat4("model", model);
					psy_thumb_modellist[i].Draw(lightingShader);
				}
			}
			for(int fidx = 0; fidx < 4; fidx++)
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
			glm::mat4 model = glm::mat4(2.0f);
			model = glm::translate(model, light[i].position);
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