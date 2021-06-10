#include <glad/glad.h>
#include <GLFW/glfw3.h>

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/type_ptr.hpp>

#include "stb_header.h"
#include "stb_image.h"

#include "glfw_interface.h"
#include "kinematics.h"
#include "finger-kinematics.h"
#include "Grips.h"

#include "utils.h"

#include "physics_thread.h"

#include "model.h"
#include "dynahex.h"

#include <iostream>
#include <thread>

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
	//hand->finger[4].chain[2].q = (-19.7f - qin[THF]) * PI / 180.f;
	hand->finger[4].chain[2].q = (-19.7f - qin[THF]) * PI / 180.f;
}

#define NUM_GRIPKEYS 12

struct key_lookup_entry
{
	int key_val;
	int grip_cfg_idx;
};
const struct key_lookup_entry key_lookup[NUM_GRIPKEYS] = {
	{GLFW_KEY_1, CHUCK_GRASP_CFG_IDX},
	{GLFW_KEY_2, CHUCK_OK_GRASP_CFG_IDX},
	{GLFW_KEY_3, PINCH_GRASP_CFG_IDX},
	{GLFW_KEY_4, POWER_GRASP_CFG_IDX},
	{GLFW_KEY_5, KEY_GRASP_CFG_IDX},
	{GLFW_KEY_6, HANDSHAKE_CFG_IDX},
	{GLFW_KEY_7, TRIGGER_CFG_IDX},
	{GLFW_KEY_8, POINT_GRASP_CFG_IDX},
	{GLFW_KEY_T, SIGN_OF_THE_HORNS_CFG_IDX},
	{GLFW_KEY_Y, RUDE_POINT_GRASP_CFG_IDX},
	{GLFW_KEY_U, MODE_SWITCH_CLOSE_CFG_IDX},
	{GLFW_KEY_I, UTILITY_GRASP_CFG_IDX},
};

dynahex_t * dynahex_bones = NULL;
kinematic_hand_t* psy_hand_bones = NULL;
int main_render_thread(void);

int main(void)
{
	std::thread t1(main_render_thread);
	std::thread t2(physics_thread);
	t1.join();
	t2.join();

}


int main_render_thread(void)
{	


	gl_start_dynamics_flag = 1;


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
	Player.CamRobot.hw_b.m[0][3] = -8.f;
	Player.CamRobot.hw_b.m[1][3] = -10.f;
	Player.CamRobot.hw_b.m[2][3] = 8.f;
	Player.CamRobot.j[1].q = fmod(209.5962f + PI, 2*PI)-PI;
	Player.CamRobot.j[2].q = -2.067593;
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
	unsigned int white_map = loadTexture("img/white.png");

	AssetModel * cube = new AssetModel("misc_models/primitive_shapes/cube.obj");
	cube->hb_model = new mat4;
	*cube->hb_model = mat4_I();
	cube->hb_model->m[0][0] = .1f;
	cube->hb_model->m[1][1] = .1f;
	cube->hb_model->m[2][2] = .1f;
	cube->hb_model->m[0][3] = .1f;
	cube->hb_model->m[1][3] = .1f;

	lightingShader.use();
	lightingShader.setInt("material.diffuse", 0);
	lightingShader.setInt("material.specular", 1);

	// tell stb_image.h to flip loaded texture's on the y-axis (before loading model).
	stbi_set_flip_vertically_on_load(true);
	//Shader model_shader("1.model_loading.vs", "1.model_loading.fs");
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

	dynahex_bones = new dynahex_t;
	init_dh_kinematics(dynahex_bones);
	forward_kinematics_dynahexleg(dynahex_bones);
	vector<AssetModel> dynahex_modellist;
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F0-stripped.STL"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F1-stripped.STL"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F2-stripped.STL"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F3-stripped.STL"));
	//dynahex_modellist.push_back(AssetModel("misc_models/dynahex/FB.obj"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/FB-stripped.STL"));
	mat4 dynahex_hw_b = mat4_I();
	for (int l = 0; l < NUM_LEGS; l++)
	{
		joint* j = dynahex_bones->leg[l].chain;
		j[1].q = 0.f;
		j[2].q = 0.f;
		j[3].q = 1.f;
	}
	forward_kinematics_dynahexleg(dynahex_bones);

	
	psy_hand_bones = new kinematic_hand_t;
	init_finger_kinematics(psy_hand_bones);
	float q[6] = { 15,15,15,15, 0,-90 };
	float qd[6] = { 15,15,15,15,0,-90 };
	transform_mpos_to_kpos(q, psy_hand_bones);

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
	int cfg_idx = 32;


	double ambient_press_time = 0.f;
	double movement_press_time = 0.f;
	run_dynamics_thread = 1;
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
		

		//manual control of the overhead light position
		int mnljki = (glfwGetKey(window, GLFW_KEY_I) == GLFW_PRESS) << 0;
		mnljki |= (glfwGetKey(window, GLFW_KEY_K) == GLFW_PRESS) << 1;
		mnljki |= (glfwGetKey(window, GLFW_KEY_J) == GLFW_PRESS) << 2;
		mnljki |= (glfwGetKey(window, GLFW_KEY_L) == GLFW_PRESS) << 3;
		mnljki |= (glfwGetKey(window, GLFW_KEY_N) == GLFW_PRESS) << 4;
		mnljki |= (glfwGetKey(window, GLFW_KEY_M) == GLFW_PRESS) << 5;
		if (mnljki == 0)
			movement_press_time = time;
		else
		{
			float displacement_per_sec = .01f*(time-movement_press_time) + 1.f/fps;
			if ( (mnljki & (1 << 0)) != 0)
				light[4].position += glm::vec3(displacement_per_sec, 0, 0);
			if ((mnljki & (1 << 1)) != 0)
				light[4].position += glm::vec3(-displacement_per_sec, 0, 0);
			if ((mnljki & (1 << 2)) != 0)
				light[4].position += glm::vec3(0, displacement_per_sec, 0);
			if ((mnljki & (1 << 3)) != 0)
				light[4].position += glm::vec3(0, -displacement_per_sec, 0);
			if ((mnljki & (1 << 4)) != 0)
				light[4].position += glm::vec3(0, 0, displacement_per_sec);
			if ((mnljki & (1 << 5)) != 0)
				light[4].position += glm::vec3(0, 0, -displacement_per_sec);
		}

		if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		{
			float v = .01f * (time - ambient_press_time);
			light[4].ambient += glm::vec3(v,v,v);
			float mag = 0;
			for (int r = 0; r < 3; r++)
				mag += light[4].ambient[r] * light[4].ambient[r];
			printf("%f\r\n", mag);
		}
		else if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		{
			float v = .01f * (time - ambient_press_time);
			light[4].ambient += glm::vec3(-v,-v,-v);
			float mag = 0;
			for (int r = 0; r < 3; r++)
				mag += light[4].ambient[r] * light[4].ambient[r];
			printf("%f\r\n", mag);
		}
		else
			ambient_press_time = time;
		
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
		//const float zoff = 10.f;
		//mat4 hw_cube[6] =
		//{
		//	{
		//		{
		//			{1, 0, 0, -bcd},
		//			{0, scf, 0, 0.f},
		//			{0, 0, scf, 0.f},
		//			{0, 0, 0, 1.f}
		//		}
		//	},
		//	{
		//		{
		//			{1, 0, 0, bcd},
		//			{0, scf, 0, 0.f},
		//			{0, 0, scf, 0.f},
		//			{0, 0, 0, 1.f}
		//		}
		//	},
		//	{
		//		{
		//			{scf, 0, 0, 0.f},
		//			{0, 1, 0, -bcd},
		//			{0, 0, scf, 0.f},
		//			{0, 0, 0, 1.f}
		//		}
		//	},
		//	{
		//		{
		//			{scf, 0, 0, 0.f},
		//			{0, 1, 0, bcd},
		//			{0, 0, scf, 0.f},
		//			{0, 0, 0, 1.f}
		//		}
		//	},
		//	{
		//		{
		//			{scf, 0, 0, 0.f},
		//			{0, scf, 0, 0.f},
		//			{0, 0, 1, -bcd},
		//			{0, 0, 0, 1.f}
		//		}
		//	},
		//	{
		//		{
		//			{scf, 0, 0, 0.f},
		//			{0, scf, 0, 0.f},
		//			{0, 0, 1, bcd},
		//			{0, 0, 0, 1.f}
		//		}
		//	}
		//};
		//for (int i = 0; i < 6; i++)
		//{
		//	hw_cube[i].m[2][3] += zoff;
		//	glm::mat4 model = ht_matrix_to_mat4(hw_cube[i]);
		//	lightingShader.setMat4("model", model);
		//	glBindVertexArray(cubeVAO);
		//	glDrawArrays(GL_TRIANGLES, 0, 36);
		//}



		mat4 cube_hw_b = mat4_I();
		//mat4 cube_hw_b = Hz(time);
		//cube_hw_b.m[0][3] = cos(time);
		//cube_hw_b.m[1][3] = sin(time);
		//cube_hw_b.m[2][3] = 2.f;
		//cube->Draw(lightingShader, &cube_hw_b);
		for (int i = 0; i < cube->meshes.size(); i++)
		{
			for (int j = 0; j < cube->meshes[i].vertices.size(); j++)
			{
				for (int r = 0; r < 3; r++)
					cube_hw_b.m[r][3] = cube->meshes[i].vertices[j].Position[r];
				cube_hw_b.m[2][3] += 2.f;
				cube->Draw(lightingShader, &cube_hw_b);
			}
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
		ourModel.Draw(lightingShader, NULL);


		int prev_cfg = cfg_idx;
		for (int i = 0; i < NUM_GRIPKEYS; i++)
		{
			if (glfwGetKey(window, key_lookup[i].key_val))
			{
				cfg_idx = key_lookup[i].grip_cfg_idx;
			}
		}
		if (glfwGetKey(window, GLFW_KEY_SPACE))
		{
			if (cfg_idx < 32)
				cfg_idx += 32;
		}
		for (int ch = 0; ch < NUM_CHANNELS; ch++)
			qd[ch] = system_griplist_default[cfg_idx]->wp[ch].qd;

		//do the math for the psyonic hand
		transform_mpos_to_kpos(q, psy_hand_bones);
		finger_kinematics(psy_hand_bones);
		
		float tau[3] = { 0,0,0 };	//num joints + 1
		vect3 f = { 0,0,0 };
		vect3 thumb_force = { 0,0,0 };
		vect3 o_thumb_b = psy_hand_bones->finger[4].ef_pos_b;



		/*Shirk Implementation/Test*/
		float k = .02f;
		float tau4 = k * (qd[4] - q[4]);
		float tau5 = k * (qd[5] - q[5]);
		float sferr = 0.f;
		float finger_error[4];
		for (int i = 0; i < 4; i++)
		{
			float error = qd[i] - q[i];
			finger_error[i] = error;
			q[i] += k * error;

			if (error < 0.f)
				error = -error;	//take absolute value of error
			sferr += error;
		}
		
		#define NUM_THUMB_REFPOINTS 4
		vect3 o_thumb_mid, o_thumb_low, o_thumb_side;
		vect3 thumb_midref_2 = { -21.39, -9.25, -2.81 };
		vect3 thumb_lowref_2 = { -46.09, -5.32, -2.58 };
		vect3 thumb_sideref_2 = { -34.52f, 0, 11.f };
		htmatrix_vect3_mult(&psy_hand_bones->finger[4].chain[2].hb_i, &thumb_midref_2, &o_thumb_mid);
		htmatrix_vect3_mult(&psy_hand_bones->finger[4].chain[2].hb_i, &thumb_lowref_2, &o_thumb_low);
		htmatrix_vect3_mult(&psy_hand_bones->finger[4].chain[2].hb_i, &thumb_sideref_2, &o_thumb_side);
		vect3* thumb_pos_b[NUM_THUMB_REFPOINTS] = { &o_thumb_b, &o_thumb_mid, &o_thumb_low, &o_thumb_side };
		float weight[NUM_THUMB_REFPOINTS] = { 44.f, 44.f, 44.f, 44.f };
				
		float shirk = 0;
		for (int i = 0; i < 4; i++)
		{
			joint* j = psy_hand_bones->finger[i].chain;
			vect3 o_f_b = psy_hand_bones->finger[i].ef_pos_b;
			//htmatrix_vect3_mult(&j[0].him1_i, &psy_hand_bones->finger[i].ef_pos_0, &o_f_b);	//wow. wordy

			vect3 knuckle_1 = { -15.44f, -6.91f, 0.f, };
			vect3 knuckle_b, knuckle_force_b;
			htmatrix_vect3_mult(&j[1].hb_i, &knuckle_1, &knuckle_b);

			/*As the figner approaches its setpoint, reduce the repulsive field between the fingertip and the thumb*/
			float abserr = finger_error[i];
			if (abserr < 0)
				abserr = -abserr;
			float tip_scalef = 1.f - (1.f / abserr);
			if (tip_scalef < 0.f)
				tip_scalef = 0.f;	//

			abserr = qd[5] - q[5];
			if (abserr < 0)
				abserr = -abserr;
			float knuckle_scalef = 1.f - (1.f / abserr);
			if (knuckle_scalef < 0.f)
				knuckle_scalef = 0.f;

			for (int thumbref_idx = 0; thumbref_idx < NUM_THUMB_REFPOINTS; thumbref_idx++)
			{
				vect3 thumb_force_b = vect3_add(o_f_b, vect3_scale(*thumb_pos_b[thumbref_idx], -1.f));	//idx force IN THE BASE FRAME. FRAME CHANGE NECESSARY
				float m1 = 0;
				for (int r = 0; r < 3; r++)
					m1 += thumb_force_b.v[r] * thumb_force_b.v[r];

				for (int r = 0; r < 3; r++)
					knuckle_force_b.v[r] = knuckle_b.v[r] - thumb_pos_b[thumbref_idx]->v[r];
				float m2 = 0;
				for (int r = 0; r < 3; r++)
					m2 += knuckle_force_b.v[r] * knuckle_force_b.v[r];

				float weight_m1 = weight[thumbref_idx] * tip_scalef;
				float weight_m2 = weight[thumbref_idx] * knuckle_scalef;

				float shirk_v = -weight_m1 / m1 - weight_m2 / m2;
				shirk += shirk_v;	//accumulate thumb rotator torque
			}
		}
		
		q[4] += tau4+shirk;
		q[5] += tau5;
		/*END SHIRK TEST*/


		//for (int i = 0; i < 4; i++)
		//{
		//	/*Get finger position in the base frame*/
		//	vect3 o_f_b = psy_hand_bones->finger[i].ef_pos_b;
		//	//htmatrix_vect3_mult(&psy_hand_bones->finger[i].chain[0].him1_i, &psy_hand_bones->finger[i].ef_pos_0, &o_f_b);	//wow. wordy
		//	
		//	/*Set some force to act on the fingertip*/
		//	vect3 finger_force_b = { 0, 0, 0 };
		//	if (i == 0)
		//		finger_force_b = vect3_add(o_thumb_b, vect3_scale(o_f_b, -1.f));	//idx force IN THE BASE FRAME. FRAME CHANGE NECESSARY
		//	else if (i == 1)
		//		finger_force_b = vect3_add(o_thumb_b, vect3_scale(o_f_b, -1.f));	//idx force IN THE BASE FRAME. FRAME CHANGE NECESSARY

		//	/*Transform the force from the base frame to the 0 frame*/
		//	mat4 hidx0_b = ht_inverse(psy_hand_bones->finger[i].chain[0].him1_i);	//consider loading in the other unoccupied 0 frame transform...?
		//	for (int r = 0; r < 3; r++)
		//		hidx0_b.m[r][3] = 0;
		//	vect3 force_0;
		//	htmatrix_vect3_mult(&hidx0_b, &finger_force_b, &force_0);

		//	/*Apply index finger force*/
		//	for (int r = 0; r < 3; r++)
		//		f.v[r] = .0003f * force_0.v[r];
		//	calc_tau3(psy_hand_bones->finger[i].chain, 2, &f, tau);
		//	q[i] += tau[1];
		//}
		///*Create an attraction force between the thumb tip and index + middle finger tip*/
		//for (int i = 0; i < 2; i++)
		//{
		//	vect3 * o_anchor_b = &psy_hand_bones->finger[i].ef_pos_b;
		//	for (int r = 0; r < 3; r++)
		//		thumb_force.v[r] += .0003f * (o_anchor_b->v[r] - o_thumb_b.v[r]);
		//}
		///*Apply Thumb force*/
		//calc_tau3(psy_hand_bones->finger[4].chain, 2, &thumb_force, tau);
		//q[5] += tau[1];
		//q[4] -= tau[2];

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



		//render the psyonic hand
		// bind diffuse map
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, white_map);
		// bind specular map
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, white_map);

		
		
		for (int i = 0; i < gl_shared_cubes.size(); i++)
		{
			if(gl_shared_cubes[i]->modelref != NULL)
				gl_shared_cubes[i]->render(lightingShader);
			else
				gl_shared_cubes[i]->modelref = cube;
		}



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
		mat4 hw_b = Hx(PI);
		hw_b = mat4_mult(hw_b, Hz(PI/2));
		hw_b = mat4_mult(tf, hw_b);
		//hw_b.m[2][3] = 1.5f+.1*(.5f*sin(time+3)+.5f);
		hw_b.m[0][3] = 7.f;
		hw_b.m[2][3] = 1.f;
		{
			{//THUMB RENDER
				//mat4 hb_0 = mat4_I();	//for fingers, this is NOT the identity. Load it into the 0th entry of the joint Him1_i matrix
				mat4 hb_0 = psy_hand_bones->finger[4].chain[0].hb_i;
				mat4 hw_0 = mat4_mult(hw_b, hb_0);

				model = ht_matrix_to_mat4(hw_0);
				lightingShader.setMat4("model", model);
				psy_thumb_modellist[0].Draw(lightingShader, NULL);
				psy_palm.Draw(lightingShader, NULL);	//render the palm too
				for (int i = 1; i <= 2; i++)
				{
					mat4 hw_i = mat4_mult(hw_b, psy_hand_bones->finger[4].chain[i].hb_i);
					model = ht_matrix_to_mat4(hw_i);
					lightingShader.setMat4("model", model);
					psy_thumb_modellist[i].Draw(lightingShader, NULL);
				}
			}
			for(int fidx = 0; fidx < 4; fidx++)
			{//FINGER RENDER
				mat4 hb_0 = psy_hand_bones->finger[fidx].chain[0].him1_i;	//store it here
				mat4 hw_0 = mat4_mult(hw_b, hb_0);

				model = ht_matrix_to_mat4(hw_0);
				lightingShader.setMat4("model", model);
				psy_finger_modellist[0].Draw(lightingShader, NULL);

				for (int i = 1; i <= 2; i++)
				{
//					mat4 h0_i = psy_hand_bones->finger[fidx].chain[i].h0_i;
//					mat4 hw_i = mat4_mult(hw_0, h0_i);
					mat4 hw_i = mat4_mult(hw_b, psy_hand_bones->finger[fidx].chain[i].hb_i);
					model = ht_matrix_to_mat4(hw_i);
					lightingShader.setMat4("model", model);
					psy_finger_modellist[i].Draw(lightingShader, NULL);
				}
			}
		}

		//float v = .1f * sin(time);
		//for (int leg = 0; leg < 6; leg++)
		//{
		//	joint* j = dynahex_bones.leg[leg].chain;
		//	for (int jidx = 1; jidx <= NUM_JOINTS_HEXLEG; jidx++)
		//	{
		//		j[jidx].q = v;
		//	}
		//}


		scf = .001f;
		mat4 H_scf = {
			{
				{scf, 0, 0, 0},
				{0, scf, 0, 0},
				{0, 0, scf, 0},
				{0, 0, 0, 1}
			}
		};
		//mat4 id = mat4_I();
		hw_b = mat4_I();
		//hw_b.m[0][3] = 100.f*cos(time);
		//hw_b.m[1][3] = 100.f*sin(time);
		//hw_b.m[2][3] = 400.f;
		mat4_mult_pbr(&H_scf, &hw_b, &dynahex_hw_b);
		dynahex_hw_b.m[1][3] = -6.f;
		dynahex_hw_b.m[2][3] = 1.f;

		model = ht_matrix_to_mat4(dynahex_hw_b);
		lightingShader.setMat4("model", model);
		dynahex_modellist[4].Draw(lightingShader, NULL);
		forward_kinematics_dynahexleg(dynahex_bones);
		for (int l = 0; l < 6; l++)
		{
			joint* j = dynahex_bones->leg[l].chain;
			for (int i = 0; i < 4; i++)
			{
				//dynahex_modellist[i].hb_model = &j[i].hb_i;
				mat4 hw_i;
				mat4_mult_pbr(&dynahex_hw_b, &j[i].hb_i, &hw_i);

				model = ht_matrix_to_mat4(hw_i);
				lightingShader.setMat4("model", model);
				dynahex_modellist[i].Draw(lightingShader, NULL);
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
	run_dynamics_thread = 0;
	return 0;
}