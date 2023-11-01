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

#include "model.h"
#include "dynahex.h"

#include <iostream>
#include <thread>
#include "physics_thread.h"

#include "kinematics_fixed.h"
#include "dh_hex_fixed.h"
#include "m_mcpy.h"

#include "hexapod_footpath.h"
#include "WinUdpBkstServer.h"
#include "WinUdpClient.h"

#include "external/tinyxml2/tinyxml2.h"
#include "IIRsos.h"
#include "z1.h"
#include "psyhand_urdf.h"

#define NUM_LIGHTS 5

//#define GET_HEXAPOD_OFFSET_VALS

/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
*/
uint32_t get_checksum32(uint32_t* arr, int size)
{
	int32_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int32_t)arr[i];
	return -checksum;
}

// glfw: whenever the window size changed (by OS or user resize) this callback function executes
// ---------------------------------------------------------------------------------------------
void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
	// make sure the viewport matches the new window dimensions; note that width and 
	// height will be significantly larger than specified on retina displays.
	glViewport(0, 0, width, height);
}
uint8_t parse_abh_htmat(WinUdpBkstServer* soc, mat4_t * m)
{
	int rc = soc->read();
	if (rc != WSAEWOULDBLOCK && soc->recv_len == 64)
	{
		int bidx = 0;
		u32_fmt_t pfmt;
		for (int r = 0; r < 4; r++)
		{
			for (int c = 0; c < 4; c++)
			{
				for (int i = 0; i < sizeof(float); i++)
				{
					pfmt.ui8[i] = soc->r_buf[bidx++];
				}
				m->m[r][c] = pfmt.f32;
			}
		}
		return 1;
	}
	return 0;
}

void parse_abh_fpos_udp_cmd(WinUdpBkstServer * soc, float * q)
{
	int rc = soc->read();
	if (rc != WSAEWOULDBLOCK && soc->recv_len == 15)
	{
		//u32_fmt_t* pfmt = (u32_fmt_t*)((uint8_t*)udp_server.r_buf);
		u32_fmt_t pfmt;
		int bidx = 2;
		for (int ch = 0; ch < 6; ch++)
		{
			for (int i = 0; i < sizeof(int16_t); i++)
			{
				pfmt.ui8[i] = soc->r_buf[bidx];
				bidx++;
			}
			q[ch] = ((float)pfmt.i16[0]) * 150.f / 32767.f;
		}
	}
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
	glm::vec4 base_color;
	glm::vec3 position;
	glm::vec3 ambient;
	glm::vec3 diffuse;
	glm::vec3 specular;
	float constant;
	float linear;
	float quadratic;
}light_params_t;

/*Core function wrapping my linear spring vector controller*/
float get_vect_err(float targ, float ref)
{
	float thr = fmod_2pi(ref + PI) - PI;
	float vr1 = cos_fast(thr);
	float vr2 = sin_fast(thr);

	float th_targ = fmod_2pi(targ + PI) - PI;
	float vt1 = cos_fast(th_targ);
	float vt2 = sin_fast(th_targ);

	float vd1 = vt1 - vr1;
	float vd2 = vt2 - vr2;

	return (vr1 * vd2 - vr2 * vd1);	//control error
}

void transform_mpos_to_kpos(float qin[6], kinematic_hand_t * hand)
{
	enum { THR = 5, THF = 4 };	//thumb mapping enum
	for (int finger = 0; finger < 4; finger++)
	{
		float fangle = qin[finger] * PI / 180.f;
		hand->finger[finger].chain[1].q = fangle;
	}
	hand->finger[4].chain[1].q = qin[THR] * PI / 180.f;
	hand->finger[4].chain[2].q = qin[THF] * PI / 180.f;
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

/*
Generic hex checksum calculation.
TODO: use this in the psyonic API
 */
int8_t get_checksum(uint8_t* arr, int size)
{

	int8_t checksum = 0;
	for (int i = 0; i < size; i++)
		checksum += (int8_t)arr[i];
	return -checksum;
}

dynahex_t * dynahex_bones = NULL;
kinematic_hand_t* psy_hand_bones = NULL;
int main_render_thread(void);


int main(void)
{
	//tinyxml2::XMLDocument doc;
	//doc.LoadFile("URDF/blah.xml");

	//tinyxml2::XMLElement* titleElement = doc.FirstChildElement("robot")->FirstChildElement("link");
	//const char* title = titleElement->GetText();
	//printf("Name of play (1): %s\n", title);

	std::thread t2(physics_thread);
	t2.join();
	std::thread t1(main_render_thread);
	t1.join();
}


int main_render_thread(void)
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
	joint cam_joints[CAM_NUM_FRAMES];
	CamControlStruct Player;				//specialized camera structure. carries around movement parameters
	init_cam(&Player, cam_joints);
	Player.CamRobot.hb_0 = mat4_t_mult(Hx(PI), mat4_t_I());
	Player.CamRobot.hw_b = mat4_t_I();		//END initializing camera
	Player.CamRobot.j[1].q = fmod(-91.145767 + PI, 2 * PI) - PI;
	Player.CamRobot.j[2].q = fmod(-1.805995 + PI, 2 * PI) - PI;
	Player.CamRobot.hw_b.m[0][3] = -9.480493;
	Player.CamRobot.hw_b.m[1][3] = 0.555928;
	Player.CamRobot.hw_b.m[2][3] = 9.907876;
	//Player.CamRobot.j[1].q = 0;
	//Player.CamRobot.j[2].q = -PI/2;
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

	AssetModel cube("misc_models/primitive_shapes/cube.obj");
	cube.hb_model = new mat4_t;
	*cube.hb_model = mat4_t_I();

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
	psy_finger_modellist.push_back(AssetModel("misc_models/psyonic-hand/split-finger-F0.STL"));
	psy_finger_modellist.push_back(AssetModel("misc_models/psyonic-hand/split-finger-F1.STL"));
	psy_finger_modellist.push_back(AssetModel("misc_models/psyonic-hand/split-finger-F2.STL"));
	AssetModel psy_palm("misc_models/psyonic-hand/PALM_BASE_FRAME.STL");
	AssetModel psy_crosslink("misc_models/psyonic-hand/crosslink.STL");






	/**/
	vector<AssetModel> simpletree_modellist;
	simpletree_modellist.push_back(AssetModel("misc_models/simple-tree/base.STL"));
	simpletree_modellist.push_back(AssetModel("misc_models/simple-tree/link.STL"));
	link_t base, link1, link2, link3, link4, link5, link6;
	/*special loading condition: base or root of the kinematic tree. hbase_us should be identity, or things break!*/
	base.name = "base";
	base.h_base_us = mat4_t_Identity;//load identity for base_base
	base.num_children = 2;//number of childrens
	base.joints = new joint2[2];	//create memory for joint references
	base.joints[0].child = &link1;	//and child
	base.joints[1].child = &link5;	//joint
	base.joints[0].q = 0;
	base.joints[1].q = 0;
	base.model_ref = (void*)(&simpletree_modellist[0]);	//create model reference for rendering
	base.joints[0].h_link = get_rpy_xyz_mat4(0, 0, 0, 7e-1, 12e-1, -0.92e-1);	//create link frames!
	base.joints[1].h_link = get_rpy_xyz_mat4(0, 0, 0, 7e-1, -12e-1, -0.92e-1);//(for both joints)
	link1.name = "link1";
	link1.joints = new joint2[1];
	link1.joints[0].child = &link2;
	link1.joints[0].q = 0;
	link1.num_children = 1;
	link1.model_ref = (void*)(&simpletree_modellist[1]);
	link1.joints[0].h_link = get_rpy_xyz_mat4(0, 0, 0, 20e-1, 0, 0);
	link2.name = "link2";
	link2.joints = new joint2[2];
	link2.joints[0].child = &link3;
	link2.joints[1].child = &link4;
	link2.num_children = 2;
	link2.joints[0].q = 0;
	link2.joints[1].q = 0;
	link2.model_ref = (void*)(&simpletree_modellist[1]);
	link2.joints[0].h_link = get_rpy_xyz_mat4(0, 0, 1.57079633, 20e-1, 0, 0);
	link2.joints[1].h_link = get_rpy_xyz_mat4(0, 0, -1.57079633, 20e-1, 0, 0);
	link4.name = "link4";
	link4.joints = NULL;
	link4.num_children = 0;
	link4.model_ref = (void*)(&simpletree_modellist[1]);
	link3.name = "link3";
	link3.joints = NULL;
	link3.num_children = 0;
	link3.model_ref = (void*)(&simpletree_modellist[1]);
	link5.name = "link5";
	link5.joints = new joint2[1];
	link5.joints[0].child = &link6;
	link5.joints[0].q = 0;
	link5.num_children = 1;
	link5.model_ref = (void*)(&simpletree_modellist[1]);
	link5.joints[0].h_link = get_rpy_xyz_mat4(0, 0, -1.57079633, 20e-1, 0, 0);
	link6.name = "link6";
	link6.joints = NULL;
	link6.num_children = 0;
	link6.model_ref = (void*)(&simpletree_modellist[1]);
	joint2* simpletree_jointlist[6] = { 
		&base.joints[0],
		&base.joints[1],
		&link1.joints[0],
		&link2.joints[0],
		&link2.joints[1],
		&link5.joints[0]
	};
	tree_assign_parent(&base);
	tree_dfs(&base);
	vect3_t simpletree_anchor = h_origin(link6.h_base_us);
	tree_jacobian(&link6, &simpletree_anchor);



	dynahex_bones = new dynahex_t;
	init_dh_kinematics(dynahex_bones);
	vector<AssetModel> dynahex_modellist;
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F0-stripped.STL"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F1-stripped.STL"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F2-stripped.STL"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/F3-stripped.STL"));
	dynahex_modellist.push_back(AssetModel("misc_models/dynahex/FB-stripped.STL"));
	


	vector<AssetModel> hexapod_modellist;
	hexapod_modellist.push_back(AssetModel("URDF/render/merged_link1_base_mesh_meters.STL"));
	hexapod_modellist.push_back(AssetModel("URDF/render/FB_gbx_leg1_meters.STL"));
	hexapod_modellist.push_back(AssetModel("URDF/render/J1_link1_meters.STL"));
	hexapod_modellist.push_back(AssetModel("URDF/render/J2_link2_meters.STL"));
	hexapod_modellist.push_back(AssetModel("URDF/render/J3_link3_meters.STL"));
	/*another hexapod structure. this */
	joint2 hexjoints[18];
	int jointlist_idx = 0;
	link_t hexbase, hexlink1[6], hexlink2[6], hexlink3[6];
	hexbase.name = "hexbase";
	hexbase.h_base_us = mat4_t_Identity;
	hexbase.model_ref = (void*)&hexapod_modellist[0];
	hexbase.joints = &hexjoints[jointlist_idx];
	hexbase.num_children = 6; // but they gotta be assigned and im just gonna POC before I use an xml parser
	jointlist_idx += hexbase.num_children;
	mat4_t basetozero = get_rpy_xyz_mat4(0.0, -0.0, 1.4824826666439834f, 9.68560403467281e-3f, 109.357754632563e-3f, -46.52e-3f);
	for (int leg = 0; leg < 6; leg++)
	{
		hexbase.joints[leg].child = &hexlink1[leg];
		hexbase.joints[leg].h_link = mat4_t_mult(Hz((float)leg * TWO_PI / 6.f), basetozero);
	}

	for (int leg = 0; leg < 6; leg++)
	{
		hexlink1[leg].name = "link 1";
		hexlink1[leg].model_ref = (void*)&hexapod_modellist[2];
		hexlink1[leg].joints = &hexjoints[jointlist_idx];
		hexlink1[leg].num_children = 1;
		jointlist_idx += hexlink1[leg].num_children;
		hexlink1[leg].joints[0].h_link = get_rpy_xyz_mat4(1.5707963267948966f, 0.f, -3.141592653589793f, 53.19977678085327e-3f, -9.341271351859646e-3f, -32.1600000000312e-3f);
		hexlink1[leg].joints[0].child = &hexlink2[leg];

		hexlink2[leg].name = "link 2";
		hexlink2[leg].model_ref = (void*)&hexapod_modellist[3];
		hexlink2[leg].joints = &hexjoints[jointlist_idx];
		hexlink2[leg].num_children = 1;
		jointlist_idx += hexlink2[leg].num_children;
		hexlink2[leg].joints[0].h_link = get_rpy_xyz_mat4(3.141592653589793f, 0, 3.141592653589793f, -17.75562321538834e-3f, 98.8847208563049e-3f, 33.359575692035264e-3f);
		hexlink2[leg].joints[0].child = &hexlink3[leg];

		hexlink3[leg].name = "link 3";
		hexlink3[leg].model_ref = (void*)&hexapod_modellist[4];
		hexlink3[leg].joints = &hexjoints[jointlist_idx];
		hexlink3[leg].num_children = 0;
		hexlink3[leg].joints = NULL;
	}
	tree_assign_parent(&hexbase);
	tree_dfs(&hexbase);

	vect3_t target = { {-5.0, 0.f, 3.f} };


	
	
	
	/*Notes on hexapod calibration:
	* the configuration below is a 'on the ground' position used for leg calibration.
	* Basically, eyeball it.
	* By taking the raw encoder pos when the robot has its legs splayed on the ground, you can use those positions to set the leg offsets.
	* qtrue = qenc - qoffset
	*/
	mat4_t dynahex_hw_b = mat4_t_I();
	const float q1_calib = 0;
	const float q2_calib = (-17.46668627f * DEG_TO_RAD);
	const float q3_calib = (-10.74216371 * DEG_TO_RAD);
	for (int l = 0; l < NUM_LEGS; l++)
	{
		joint* j = dynahex_bones->leg[l].chain;
		j[1].q = q1_calib;
		j[2].q = q2_calib;
		j[3].q = q3_calib;
	}
	forward_kinematics_dynahexleg(dynahex_bones);
	print_mat4_t(dynahex_bones->leg[0].chain[3].hb_i);
	{
		vect3_t targ_b = h_origin(dynahex_bones->leg[0].chain[3].hb_i);
		mat4_t* hb_0 = &dynahex_bones->leg[0].chain[0].him1_i;
		joint* start = &dynahex_bones->leg[0].chain[1];
		ik_closedform_hexapod(hb_0, start, &targ_b);
		printf("q1=%f\r\n", wrap_2pi(start->q));
		printf("q2=%f\r\n", wrap_2pi(start->child->q));
		printf("q3=%f\r\n", wrap_2pi(start->child->child->q));
	}

	psy_hand_bones = new kinematic_hand_t;
	init_finger_kinematics(psy_hand_bones);
	float qleft[6] = { 15,15,15,15, 0,-90 };
	float qright[6] = { 15,15,15,15, 0,-90 };
	float qd[6] = { 15,15,15,15,0,-90 };
	transform_mpos_to_kpos(qleft, psy_hand_bones);

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
	enum { KEYBOARD_CONTROLLED_LIGHT_IDX = 4, ROBOT_CONNECTED_LIGHT_IDX = 3};
	light[KEYBOARD_CONTROLLED_LIGHT_IDX].position = glm::vec3(-2, 1, 7.9);
	for (int i = 0; i < NUM_LIGHTS; i++)
	{
		light[i].ambient = glm::vec3(0.07f, 0.07f, 0.07f);
		light[i].diffuse = glm::vec3(0.7f, 0.7f, 0.7f);
		light[i].specular = glm::vec3(0.9f, 0.9f, 0.9f);
		light[i].constant = 1.f;
		light[i].linear = .09f;
		light[i].quadratic = .028f;
		light[i].base_color = glm::vec4(1.f);
	}

	float shininess = 32.f;
	int cfg_idx = 32;


	double ambient_press_time = 0.f;
	double movement_press_time = 0.f;


	uint8_t debug_msg_queued = 0;



	WinUdpBkstServer udp_server(50134);
	if (udp_server.set_nonblocking() != NO_ERROR)
		printf("socket at port %d set to non-blocking ok\r\n", udp_server.port);


	WinUdpBkstServer abh_lh_pos_soc(7242);
	if(abh_lh_pos_soc.set_nonblocking() != NO_ERROR)
		printf("socket at port %d set to non-blocking ok\r\n", abh_lh_pos_soc.port);
	WinUdpBkstServer abh_rh_pos_soc(7240);
	if (abh_rh_pos_soc.set_nonblocking() != NO_ERROR)
		printf("socket at port %d set to non-blocking ok\r\n", abh_rh_pos_soc.port);
	WinUdpBkstServer abh_lh_finger_soc(23234);
	abh_lh_finger_soc.set_nonblocking();
	WinUdpBkstServer abh_rh_finger_soc(34345);
	abh_rh_finger_soc.set_nonblocking(); 


	/*UDP client for ESP32 udp server that points the hose at us if we ping it*/
	uint8_t udp_rx_buf[BUFLEN];	//large udp recive buffer
	WinUdpClient robot_client(3145);
	robot_client.set_nonblocking();
	robot_client.si_other.sin_addr.S_un.S_addr = robot_client.get_bkst_ip();
	uint64_t udpsend_ts = 0;
	uint64_t udp_connected_ts = 0;

	mat4_t lh_htmat = mat4_t_Identity;
	mat4_t rh_htmat = mat4_t_Identity;
	iirSOS lpfs[6] = { 0 };
	for (int i = 0; i < 6; i++)
	{
		m_mcpy(&lpfs[i], (iirSOS*)(&lpf_template), sizeof(iirSOS));
	}
	
	AbilityHandLeftUrdf left_abh_2;
	
	Z1_arm z1;
	z1.hw_b = Hz(PI);
	z1.hw_b.m[0][3] = 0;
	z1.hw_b.m[1][3] = 1;
	z1.hw_b.m[2][3] = 2.0f;
	
	z1.joints[1].q = 0.3;
	z1.joints[2].q = 0.3;
	z1.joints[3].q = -0.3;
	z1.joints[4].q = 0.3;
	z1.joints[5].q = 0.3;
	z1.joints[6].q = 0.3;
	z1.fk();
	z1.joints[1].q = 0;
	z1.joints[2].q = 1.5;
	z1.joints[3].q = -1.17;
	z1.joints[4].q = -1.9;
	z1.joints[5].q = 0;
	z1.joints[6].q = 0.01;
	z1.fk();


	{
		mat4_t hf6_targ = mat4_t_Identity;
		hf6_targ.m[0][3] = 0.051e1f;
		mat4_t htarg = mat4_t_mult(z1.joints[6].hb_i, hf6_targ);
		print_mat4_t(htarg);
		printf("\r\n");
	}
	//print_mat4_t()
	while (!glfwWindowShouldClose(window))
	{
		double time = glfwGetTime();
		double fps = 1.0 / (time - prev_time);
		prev_time = time;
		uint64_t tick = GetTickCount64();

		// render
		// ------
		glClearColor(0.2f, 0.2f, 0.2f, 1.0f);
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

		lightingShader.use();
		vect3_t cam_origin = h_origin(Player.CamRobot.hw_b);
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



		if ((tick - udp_connected_ts) < 300)	//connected
		{
			//for (int i = 0; i < NUM_LIGHTS; i++)
			{
				int i = ROBOT_CONNECTED_LIGHT_IDX;
				light[i].ambient = glm::vec3(0.07f, 0.07f, 0.07f);
				light[i].diffuse = glm::vec3(0.7f, 0.7f, 0.7f);
				light[i].specular = glm::vec3(0.9f, 0.9f, 0.9f);
				light[i].base_color = glm::vec4(1.f);
			}
		}
		else
		{
			//for (int i = 0; i < NUM_LIGHTS; i++)
			{
				int i = ROBOT_CONNECTED_LIGHT_IDX;
				light[i].ambient = glm::vec3(0.07f, 0, 0);
				light[i].diffuse = glm::vec3(0.7f, 0, 0);
				light[i].specular = glm::vec3(0.9f, 0.0f, 0.0f);
				light[i].base_color = glm::vec4(1.f, 0.f, 0.f, 1.f);
			}
			for (int l = 0; l < NUM_LEGS; l++)
			{
				joint* j = dynahex_bones->leg[l].chain;
				j[1].q = q1_calib;
				j[2].q = q2_calib;
				j[3].q = q3_calib;
			}
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
		View = keyboard_cam_control(window, &Player, fps, target);
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
			float displacement_per_sec = (.01f*(float)((time-movement_press_time))) + 1.f/(float)fps;
			if ( (mnljki & (1 << 0)) != 0)
				light[KEYBOARD_CONTROLLED_LIGHT_IDX].position += glm::vec3(displacement_per_sec, 0, 0);
			if ((mnljki & (1 << 1)) != 0)
				light[KEYBOARD_CONTROLLED_LIGHT_IDX].position += glm::vec3(-displacement_per_sec, 0, 0);
			if ((mnljki & (1 << 2)) != 0)
				light[KEYBOARD_CONTROLLED_LIGHT_IDX].position += glm::vec3(0, displacement_per_sec, 0);
			if ((mnljki & (1 << 3)) != 0)
				light[KEYBOARD_CONTROLLED_LIGHT_IDX].position += glm::vec3(0, -displacement_per_sec, 0);
			if ((mnljki & (1 << 4)) != 0)
				light[KEYBOARD_CONTROLLED_LIGHT_IDX].position += glm::vec3(0, 0, displacement_per_sec);
			if ((mnljki & (1 << 5)) != 0)
				light[KEYBOARD_CONTROLLED_LIGHT_IDX].position += glm::vec3(0, 0, -displacement_per_sec);
		}

		if (glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		{
			if (glfwGetKey(window, GLFW_KEY_LEFT_BRACKET) == GLFW_PRESS)
			{
				Player.xyV *= 0.95f;
				if (Player.xyV <= 1e-9f)
					Player.xyV = 1e-9f;
				printf("xyV = %f\r\n", Player.xyV);
			}
			else if (glfwGetKey(window, GLFW_KEY_RIGHT_BRACKET) == GLFW_PRESS)
			{
				Player.xyV *= 1.05f;
				printf("xyV = %f\r\n", Player.xyV);
			}
		}
		if (glfwGetKey(window, GLFW_KEY_PERIOD) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		{
			float v =(float)( .01f * (time - ambient_press_time));
			light[KEYBOARD_CONTROLLED_LIGHT_IDX].ambient += glm::vec3(v,v,v);
			float mag = 0;
			for (int r = 0; r < 3; r++)
				mag += light[KEYBOARD_CONTROLLED_LIGHT_IDX].ambient[r] * light[KEYBOARD_CONTROLLED_LIGHT_IDX].ambient[r];
			printf("%f\r\n", mag);
		}
		else if (glfwGetKey(window, GLFW_KEY_COMMA) == GLFW_PRESS && glfwGetKey(window, GLFW_KEY_LEFT_SHIFT) == GLFW_PRESS)
		{
			float v = (float)(.01f * (time - ambient_press_time));
			light[KEYBOARD_CONTROLLED_LIGHT_IDX].ambient += glm::vec3(-v,-v,-v);
			float mag = 0;
			for (int r = 0; r < 3; r++)
				mag += light[KEYBOARD_CONTROLLED_LIGHT_IDX].ambient[r] * light[KEYBOARD_CONTROLLED_LIGHT_IDX].ambient[r];
			printf("%f\r\n", mag);
		}
		else
			ambient_press_time = time;
		
		//vect3_t player_pos;
		//for(int r = 0; r < 3; r++)
		//	player_pos.v[r] = Player.CamRobot.hw_b.m[r][3];
		//point_light_positions[4] = glm::vec3(player_pos.v[0], player_pos.v[1], er_pos.v[2] + 4.f);


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
		mat4_t hw_cube[6] =
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
			glm::mat4 model = ht_matrix_to_mat4_t(hw_cube[i]);
			lightingShader.setMat4("model", model);

			glBindVertexArray(cubeVAO);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}

		// don't forget to enable shader before setting uniforms
		//model_shader.use();// view/projection transformations
		//model_shader.setMat4("projection", CameraProjection);
		//model_shader.setMat4("view", View);

		scf = .333f;
		mat4_t backpack_htm = {
			{
				{scf, 0, 0, 0},
				{0, scf, 0, 0},
				{0, 0, scf, 0},
				{0, 0, 0, 1}
			}
		};
		mat4_t res;		
		mat4_t rot = Hx(PI / 2.f);
		mat4_t_mult_pbr(&backpack_htm, &rot, &res);
		copy_mat4_t(&backpack_htm, &res);
		backpack_htm.m[0][3] = 0;
		backpack_htm.m[1][3] = 8.f;
		backpack_htm.m[2][3] = 1.3f;
		model = ht_matrix_to_mat4_t(backpack_htm);
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

		if (glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_PRESS)
		{
			debug_msg_queued = 1;
		}
		if (debug_msg_queued && glfwGetMouseButton(window, GLFW_MOUSE_BUTTON_RIGHT) == GLFW_RELEASE)
		{
			debug_msg_queued = 0;


			/*prepare message*/
			{
				printf("index q1 = %f\r\n", psy_hand_bones->finger[0].chain[1].q*RAD_TO_DEG);
				printf("index q2 = %f\r\n", psy_hand_bones->finger[0].chain[2].q*RAD_TO_DEG);

				for (int link = 1; link <= 2; link++)
				{
					printf("index h_link_%d:\r\n", link);
					print_mat4_t(psy_hand_bones->finger[0].chain[link].h_link);
					printf("\r\n\r\n");
				}
				for (int link = 1; link <= 2; link++)
				{
					vect3_t xyz, rpy;
					get_xyz_rpy(&psy_hand_bones->finger[0].chain[link].h_link, &xyz, &rpy);
					printf("link %d xyz = ", link);
					print_vect3(xyz);
					printf("rpy = ");
					print_vect3(rpy);
					printf("\r\n");
				}
			}
		}
		


		//printf("link transformed = \n");
		//print_mat4_t(psy_hand_bones->finger[0].chain[1].him1_i);
		//printf("\nlink dh-original= \n");
		//print_mat4_t(psy_hand_bones->finger[0].chain[1].h_link);
		//printf("\nfinger hb_0\n");
		//print_mat4_t(psy_hand_bones->finger[0].chain[0].hb_i);

		float tau[3] = { 0,0,0 };	//num joints + 1
		vect3_t f = { 0,0,0 };
		vect3_t thumb_force = { 0,0,0 };
		vect3_t o_thumb_b = psy_hand_bones->finger[4].ef_pos_b;



		///*Shirk Implementation/Test*/
		//float k = .02f;
		//
		////float tau4 = k * get_vect_err(DEG_TO_RAD * qd[4], DEG_TO_RAD * q[4]) * RAD_TO_DEG;
		////float tau5 = k * get_vect_err(DEG_TO_RAD * qd[5], DEG_TO_RAD * q[5]) * RAD_TO_DEG;
		//float finger_error[6];
		//for (int i = 0; i < 6; i++)
		//	finger_error[i] = get_vect_err(DEG_TO_RAD * qd[i], DEG_TO_RAD * q[i]) * RAD_TO_DEG;
		//float tau4 = k * finger_error[4];
		//float tau5 = k * finger_error[5];

		//#define NUM_THUMB_REFPOINTS 4
		//vect3_t o_thumb_mid, o_thumb_low, o_thumb_side;
		//vect3_t thumb_midref_2 = { -21.39, -9.25, -2.81 };
		//vect3_t thumb_lowref_2 = { -46.09, -5.32, -2.58 };
		//vect3_t thumb_sideref_2 = { -34.52f, 0, 11.f };
		//htmatrix_vect3_mult(&psy_hand_bones->finger[4].chain[2].hb_i, &thumb_midref_2, &o_thumb_mid);
		//htmatrix_vect3_mult(&psy_hand_bones->finger[4].chain[2].hb_i, &thumb_lowref_2, &o_thumb_low);
		//htmatrix_vect3_mult(&psy_hand_bones->finger[4].chain[2].hb_i, &thumb_sideref_2, &o_thumb_side);
		//vect3_t* thumb_pos_b[NUM_THUMB_REFPOINTS] = { &o_thumb_b, &o_thumb_mid, &o_thumb_low, &o_thumb_side };
		//float weight[NUM_THUMB_REFPOINTS] = { 44.f, 44.f, 44.f, 44.f };
		//		
		//float shirk = 0;
		//for (int i = 0; i < 4; i++)
		//{
		//	joint* j = psy_hand_bones->finger[i].chain;
		//	vect3_t o_f_b = psy_hand_bones->finger[i].ef_pos_b;
		//	//htmatrix_vect3_mult(&j[0].him1_i, &psy_hand_bones->finger[i].ef_pos_0, &o_f_b);	//wow. wordy

		//	vect3_t knuckle_1 = { -15.44f, -6.91f, 0.f, };
		//	vect3_t knuckle_b, knuckle_force_b;
		//	htmatrix_vect3_mult(&j[1].hb_i, &knuckle_1, &knuckle_b);

		//	/*As the figner approaches its setpoint, reduce the repulsive field between the fingertip and the thumb*/
		//	float abserr = finger_error[i];
		//	if (abserr < 0)
		//		abserr = -abserr;
		//	float tip_scalef = 1.f - (1.f / abserr);
		//	if (tip_scalef < 0.f)
		//		tip_scalef = 0.f;	//

		//	abserr = finger_error[5];
		//	if (abserr < 0)
		//		abserr = -abserr;
		//	float knuckle_scalef = 1.f - (1.f / abserr);
		//	if (knuckle_scalef < 0.f)
		//		knuckle_scalef = 0.f;

		//	for (int thumbref_idx = 0; thumbref_idx < NUM_THUMB_REFPOINTS; thumbref_idx++)
		//	{
		//		vect3_t thumb_force_b = vect3_add(o_f_b, vect3_scale(*thumb_pos_b[thumbref_idx], -1.f));	//idx force IN THE BASE FRAME. FRAME CHANGE NECESSARY
		//		float m1 = 0;
		//		for (int r = 0; r < 3; r++)
		//			m1 += thumb_force_b.v[r] * thumb_force_b.v[r];

		//		for (int r = 0; r < 3; r++)
		//			knuckle_force_b.v[r] = knuckle_b.v[r] - thumb_pos_b[thumbref_idx]->v[r];
		//		float m2 = 0;
		//		for (int r = 0; r < 3; r++)
		//			m2 += knuckle_force_b.v[r] * knuckle_force_b.v[r];

		//		float weight_m1 = weight[thumbref_idx] * tip_scalef;
		//		float weight_m2 = weight[thumbref_idx] * knuckle_scalef;

		//		float shirk_v = -weight_m1 / m1 - weight_m2 / m2;
		//		shirk += shirk_v;	//accumulate thumb rotator torque
		//	}
		//}
		//for(int i = 0; i < 4; i++)
		//	q[i] += k * finger_error[i];
		//q[4] += tau4+shirk;
		//q[5] += tau5;
		///*END SHIRK TEST*/


		//for (int i = 0; i < 4; i++)
		//{
		//	/*Get finger position in the base frame*/
		//	vect3_t o_f_b = psy_hand_bones->finger[i].ef_pos_b;
		//	//htmatrix_vect3_mult(&psy_hand_bones->finger[i].chain[0].him1_i, &psy_hand_bones->finger[i].ef_pos_0, &o_f_b);	//wow. wordy
		//	
		//	/*Set some force to act on the fingertip*/
		//	vect3_t finger_force_b = { 0, 0, 0 };
		//	if (i == 0)
		//		finger_force_b = vect3_add(o_thumb_b, vect3_scale(o_f_b, -1.f));	//idx force IN THE BASE FRAME. FRAME CHANGE NECESSARY
		//	else if (i == 1)
		//		finger_force_b = vect3_add(o_thumb_b, vect3_scale(o_f_b, -1.f));	//idx force IN THE BASE FRAME. FRAME CHANGE NECESSARY

		//	/*Transform the force from the base frame to the 0 frame*/
		//	mat4_t hidx0_b = ht_inverse(psy_hand_bones->finger[i].chain[0].him1_i);	//consider loading in the other unoccupied 0 frame transform...?
		//	for (int r = 0; r < 3; r++)
		//		hidx0_b.m[r][3] = 0;
		//	vect3_t force_0;
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
		//	vect3_t * o_anchor_b = &psy_hand_bones->finger[i].ef_pos_b;
		//	for (int r = 0; r < 3; r++)
		//		thumb_force.v[r] += .0003f * (o_anchor_b->v[r] - o_thumb_b.v[r]);
		//}
		///*Apply Thumb force*/
		//calc_tau3(psy_hand_bones->finger[4].chain, 2, &thumb_force, tau);
		//q[5] += tau[1];
		//q[4] -= tau[2];




		//render the psyonic hand
		// bind diffuse map
		glActiveTexture(GL_TEXTURE0);
		glBindTexture(GL_TEXTURE_2D, white_map);
		// bind specular map
		glActiveTexture(GL_TEXTURE1);
		glBindTexture(GL_TEXTURE_2D, white_map);

		//lightingShader.setFloat("material.shininess", 32.0f);
		scf = 0.01f;
		mat4_t tf = {
			{
				{scf, 0, 0, 0},
				{0, scf, 0, 0},
				{0, 0, scf, 0},
				{0, 0, 0, 1}
			}
		};

		//mat4_t hw_b = Hx(PI / 2 + .2*sin(time) );
		//hw_b = mat4_t_mult(hw_b, Hy(.2*sin(time + 1)));
		//hw_b = mat4_t_mult(hw_b, Hz(.2*sin(time + 2)));

		/*create a model meters to mm scale matrix*/
		mat4_t scale_matrix = {
			{
				{1000.f,0,0,0},
				{0,1000.f,0,0},
				{0,0,1000.f,0},
				{0,0,0,1.f},
			}
		};

		mat4_t hw_b = Hx(PI);
		hw_b = mat4_t_mult(hw_b, Hz(PI/2));
		hw_b = mat4_t_mult(tf, hw_b);
		//hw_b.m[2][3] = 1.5f+.1*(.5f*sin(time+3)+.5f);
		hw_b.m[0][3] = 0;
		hw_b.m[1][3] = 2;
		hw_b.m[2][3] = 10.f;


		
		{
			int rc = parse_abh_htmat(&abh_lh_pos_soc, &lh_htmat);
			vect3_t lh_rpy; vect3_t lh_xyz; get_xyz_rpy(&lh_htmat, &lh_xyz, &lh_rpy);

			//filter
			lh_rpy.v[0] = sos_f(&lpfs[0], lh_rpy.v[0]);
			lh_rpy.v[1] = sos_f(&lpfs[1], lh_rpy.v[1]);
			lh_rpy.v[2] = sos_f(&lpfs[2], lh_rpy.v[2]);

			vect3_t shuffle;
			shuffle.v[0] = lh_rpy.v[1]+PI;
			shuffle.v[1] = (lh_rpy.v[2] + PI/2)*0;
			shuffle.v[2] = lh_rpy.v[0]+0.2+PI/2;


			mat4_t m = get_rpy_xyz_htmatrix(&lh_xyz, &shuffle);
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					hw_b.m[r][c] = m.m[r][c] * .01;
				}
			}
		}
		parse_abh_fpos_udp_cmd(&abh_lh_finger_soc, qleft);

		//do the math for the psyonic hand
		transform_mpos_to_kpos(qleft, psy_hand_bones);
		finger_kinematics(psy_hand_bones);
		{
			{//THUMB RENDER
				//mat4_t hb_0 = mat4_t_I();	//for fingers, this is NOT the identity. Load it into the 0th entry of the joint Him1_i matrix
				mat4_t hb_0 = psy_hand_bones->finger[4].chain[0].hb_i;
				mat4_t hw_0 = mat4_t_mult(hw_b, hb_0);

				model = ht_matrix_to_mat4_t(hw_0);
				lightingShader.setMat4("model", model);
				//psy_thumb_modellist[0].Draw(lightingShader, NULL);
				//psy_palm.Draw(lightingShader, NULL);	//render the palm too
				for (int i = 1; i <= 2; i++)
				{
					mat4_t hw_i = mat4_t_mult(hw_b, psy_hand_bones->finger[4].chain[i].hb_i);
					hw_i = mat4_t_mult(hw_i, scale_matrix);
					model = ht_matrix_to_mat4_t(hw_i);
					lightingShader.setMat4("model", model);
					psy_thumb_modellist[i].Draw(lightingShader, NULL);
				}
			}
			for(int fidx = 0; fidx <= 3; fidx++)
			{//FINGER RENDER

				/*Establish hw_0 frame*/
				mat4_t hb_0 = psy_hand_bones->finger[fidx].chain[0].him1_i;	//store it here
				mat4_t hw_0 = mat4_t_mult(hw_b, hb_0);
				
				{
					mat4_t h0_crosslink = mat4_t_Identity;
					vect3_t crosslink_origin = { {9.47966f, -0.62133f, -0.04458f} };

					vect3_t o2, v_crx, v_cry, v_crz;
					mat4_t h0_2 = mat4_t_mult(psy_hand_bones->finger[fidx].chain[1].him1_i, psy_hand_bones->finger[fidx].chain[2].him1_i);
					h_origin_pbr(&h0_2, &o2);

					//get direction vector pointing from crosslink location to the origin of frame 2 (where the crosslink x intersects)
					for (int i = 0; i < 3; i++)
						v_crx.v[i] = o2.v[i] - crosslink_origin.v[i];
					v_crx.v[2] = 0;
					v_crx = vect3_normalize(v_crx);
					v_crz = { {0,0,-1} };
					v_cry = cross(v_crz, v_crx);
					v_cry = vect3_normalize(v_cry);
					float dp = vect_dot(v_crx.v, v_crz.v, 3);
					if (abs_f(dp) > 0.00001f)
					{
						v_crz = cross(v_crx, v_cry);
						v_crz = vect3_normalize(v_crz);
					}
					for (int r = 0; r < 3; r++)
					{
						h0_crosslink.m[r][0] = v_crx.v[r];
						h0_crosslink.m[r][1] = v_cry.v[r];
						h0_crosslink.m[r][2] = v_crz.v[r];
						h0_crosslink.m[r][3] = crosslink_origin.v[r];
					}

					mat4_t hw_crosslink = mat4_t_mult(hw_0, h0_crosslink);
					hw_crosslink = mat4_t_mult(hw_crosslink, scale_matrix);
					model = ht_matrix_to_mat4_t(hw_crosslink);
					lightingShader.setMat4("model", model);
					psy_crosslink.Draw(lightingShader, NULL);
				}

				mat4_t f0_mesh = mat4_t_mult(hw_0, scale_matrix);
				model = ht_matrix_to_mat4_t(f0_mesh);
				lightingShader.setMat4("model", model);
				psy_finger_modellist[0].Draw(lightingShader, NULL);

				for (int i = 1; i <= 2; i++)
				{
					mat4_t hw_i = mat4_t_mult(hw_b, psy_hand_bones->finger[fidx].chain[i].hb_i);

					hw_i = mat4_t_mult(hw_i, scale_matrix);
					model = ht_matrix_to_mat4_t(hw_i);
					
					lightingShader.setMat4("model", model);
					psy_finger_modellist[i].Draw(lightingShader, NULL);
				}

			}
		}
		

		{
			//float roll = sin(time);
			//float pitch = (-cos(time)*0.5+0.5)*- PI/2;
			//float yaw = 0;
			vect3_t lightboxpos_w = {
				{
					light[KEYBOARD_CONTROLLED_LIGHT_IDX].position.x,
					light[KEYBOARD_CONTROLLED_LIGHT_IDX].position.y,
					light[KEYBOARD_CONTROLLED_LIGHT_IDX].position.z
				}
			};
			mat4_t hb_w = ht_inverse(z1.hw_b);
			vect3_t otarg_b = mat4_t_vect3_mult(hb_w, lightboxpos_w);

			if (glfwGetKey(window, GLFW_KEY_P) == GLFW_PRESS)
			{
				printf("targW=");
				print_vect3(lightboxpos_w);
				printf("\r\ntargB=");
				print_vect3(otarg_b);
				printf("\r\nq = {\r\n");
				for(int i = 0; i < 6; i++)
					printf("    %f\r\n", z1.joints[i+1].q);	
				printf("};\r\n");
			}

			float roll = 0;
			float pitch = -PI/2+PI/6;
			float yaw = 0;
			float x = otarg_b.v[0];
			float y = otarg_b.v[1];
			float z = otarg_b.v[2];
			mat4_t z1_ik_targ = get_rpy_xyz_mat4(roll,pitch,yaw,x,y,z);
			z1_ik_targ = mat4_t_mult(z1_ik_targ, Hz(0.3*sin(time)));
			//z1_ik_targ = mat4_t_mult(z1_ik_targ, Hx(0.3 * sin(5*time)));
			z1_ik_targ = mat4_t_mult(z1_ik_targ, Hy(0.3 * cos(time)));

			//mat4_t z1_ik_targ = {
			//	{
			//		{0.784573, -0.443946, 0.432849, 0.285926, },
			//		{0.552033, 0.818008, -0.161624, 0.398402, },
			//		{-0.282321, 0.365753, 0.886859, 1.573436, },
			//		{0.000000, 0.000000, 0.000000, 1.000000, }
			//	}
			//};
			//z1_ik_targ = mat4_t_mult(z1_ik_targ, Hx(time));

			double err = 1000.;
			int iters = 0;
			int iters_per_iter = 100;
			while (err > .0001)
			{
				err = z1.num_ik(&z1_ik_targ, iters_per_iter);
				iters += iters_per_iter;
				if (iters > 20000)
					break;
			}
			//printf("err %f, fps = %f, iters = %d\r\n", err, fps, iters);

			/*Example of how to computeulate the origin of the target frame in the base frame. construct the frame6-to-target homogeneous transformation*/
			//mat4_t hf6_targ = mat4_t_Identity;
			//hf6_targ.m[0][3] = 0.051e1f;
			//mat4_t htarg = mat4_t_mult(z1.joints[6].hb_i, hf6_targ);
			//print_mat4_t(htarg);
			//printf("\r\n\r\n");
		}
		z1.render_arm(lightingShader);
		/*connect abh to z1*/
		left_abh_2.fk();
		
		float lh_q[6] = { 0 };
		for (int i = 0; i < 5; i++)
		{
			lh_q[i] = (-0.5 * cos((float)i + time) + 0.5) * 50.f;
		}
		lh_q[5] = ((-0.5 * cos(5.f + time) + 0.5) * -100.f);
		left_abh_2.load_q(lh_q);

		{
			mat4_t hw_z16 = mat4_t_mult(z1.hw_b, z1.joints[6].hb_i);
			mat4_t hz16_abhw = get_rpy_xyz_mat4(0, 1.57079632679, 0, 60e-2, 0, 0);
			left_abh_2.hw_b = mat4_t_mult(hw_z16, hz16_abhw);
			left_abh_2.hw_b = mat4_t_mult(left_abh_2.hw_b, Hz(-PI/2));

			left_abh_2.hw_b = mat4_t_mult(left_abh_2.hw_b, Hscale(10.f));
			left_abh_2.render(&lightingShader);	//then render
		}


		
		{
			int rc = parse_abh_htmat(&abh_rh_pos_soc, &rh_htmat);
			vect3_t rh_rpy; vect3_t rh_xyz; get_xyz_rpy(&rh_htmat, &rh_xyz, &rh_rpy);
			
			rh_rpy.v[0] = sos_f(&lpfs[3], rh_rpy.v[0]);
			rh_rpy.v[1] = sos_f(&lpfs[4], rh_rpy.v[1]);
			rh_rpy.v[2] = sos_f(&lpfs[5], rh_rpy.v[2]);
			
			vect3_t shuffle;
			shuffle.v[0] = rh_rpy.v[1] + PI;	//good
			shuffle.v[1] = 0*(rh_rpy.v[2] + PI/2);
			shuffle.v[2] = rh_rpy.v[0]+2.83+PI+PI/2+PI;	//good
			mat4_t m = get_rpy_xyz_htmatrix(&rh_xyz, &shuffle);
			for (int r = 0; r < 3; r++)
			{
				for (int c = 0; c < 3; c++)
				{
					hw_b.m[r][c] = m.m[r][c]* .01;
				}
			}
		}
		parse_abh_fpos_udp_cmd(&abh_rh_finger_soc, qright);

		hw_b.m[1][3] = 0;
		mat4_t hmir = mat4_t_Identity;
		hmir.m[0][0] = -1;
		hw_b = mat4_t_mult(hw_b, hmir);

		//do the math for the psyonic hand
		transform_mpos_to_kpos(qright, psy_hand_bones);
		finger_kinematics(psy_hand_bones);
		{
			{//THUMB RENDER
				//mat4_t hb_0 = mat4_t_I();	//for fingers, this is NOT the identity. Load it into the 0th entry of the joint Him1_i matrix
				mat4_t hb_0 = psy_hand_bones->finger[4].chain[0].hb_i;
				mat4_t hw_0 = mat4_t_mult(hw_b, hb_0);

				model = ht_matrix_to_mat4_t(hw_0);
				lightingShader.setMat4("model", model);
				//psy_thumb_modellist[0].Draw(lightingShader, NULL);
				//psy_palm.Draw(lightingShader, NULL);	//render the palm too
				for (int i = 1; i <= 2; i++)
				{
					mat4_t hw_i = mat4_t_mult(hw_b, psy_hand_bones->finger[4].chain[i].hb_i);
					hw_i = mat4_t_mult(hw_i, scale_matrix);
					model = ht_matrix_to_mat4_t(hw_i);
					lightingShader.setMat4("model", model);
					psy_thumb_modellist[i].Draw(lightingShader, NULL);
				}
			}
			for (int fidx = 0; fidx <= 3; fidx++)
			{//FINGER RENDER

				/*Establish hw_0 frame*/
				mat4_t hb_0 = psy_hand_bones->finger[fidx].chain[0].him1_i;	//store it here
				mat4_t hw_0 = mat4_t_mult(hw_b, hb_0);

				{
					mat4_t h0_crosslink = mat4_t_Identity;
					vect3_t crosslink_origin = { {9.47966f, -0.62133f, -0.04458f} };

					vect3_t o2, v_crx, v_cry, v_crz;
					mat4_t h0_2 = mat4_t_mult(psy_hand_bones->finger[fidx].chain[1].him1_i, psy_hand_bones->finger[fidx].chain[2].him1_i);
					h_origin_pbr(&h0_2, &o2);

					//get direction vector pointing from crosslink location to the origin of frame 2 (where the crosslink x intersects)
					for (int i = 0; i < 3; i++)
						v_crx.v[i] = o2.v[i] - crosslink_origin.v[i];
					v_crx.v[2] = 0;
					v_crx = vect3_normalize(v_crx);
					v_crz = { {0,0,-1} };
					v_cry = cross(v_crz, v_crx);
					v_cry = vect3_normalize(v_cry);
					float dp = vect_dot(v_crx.v, v_crz.v, 3);
					if (abs_f(dp) > 0.00001f)
					{
						v_crz = cross(v_crx, v_cry);
						v_crz = vect3_normalize(v_crz);
					}
					for (int r = 0; r < 3; r++)
					{
						h0_crosslink.m[r][0] = v_crx.v[r];
						h0_crosslink.m[r][1] = v_cry.v[r];
						h0_crosslink.m[r][2] = v_crz.v[r];
						h0_crosslink.m[r][3] = crosslink_origin.v[r];
					}

					mat4_t hw_crosslink = mat4_t_mult(hw_0, h0_crosslink);
					hw_crosslink = mat4_t_mult(hw_crosslink, scale_matrix);
					model = ht_matrix_to_mat4_t(hw_crosslink);
					lightingShader.setMat4("model", model);
					psy_crosslink.Draw(lightingShader, NULL);
				}

				mat4_t f0_mesh = mat4_t_mult(hw_0, scale_matrix);
				model = ht_matrix_to_mat4_t(f0_mesh);
				lightingShader.setMat4("model", model);
				psy_finger_modellist[0].Draw(lightingShader, NULL);

				for (int i = 1; i <= 2; i++)
				{
					mat4_t hw_i = mat4_t_mult(hw_b, psy_hand_bones->finger[fidx].chain[i].hb_i);

					hw_i = mat4_t_mult(hw_i, scale_matrix);
					model = ht_matrix_to_mat4_t(hw_i);

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

		dynahex_hw_b = Hscale(.005f);
		dynahex_hw_b.m[1][3] = -6.f;
		dynahex_hw_b.m[2][3] = 2.f;

		///*Kill render of all legs 1-5, leaving only leg 0*/
		//mat4_t zeros = { { {0,0,0,0},{0,0,0,0},{0,0,0,0},{0,0,0,0} } };
		//for (int leg = 1; leg < 6; leg++)
		//	dynahex_bones->leg[leg].chain[0].hb_i = zeros;

		{//do stuff to hexleg here

			/*Generate base for first half of the legs*/
			vect3_t foot_xy_1;
			float h = 40; float w = 100;
			float period = 2.f;
			//foot_path(time, h, w, period, &foot_xy_1);
			float one_16 = (float)(1 << 16);
			vect3_32b_t foot_xy_1_32b;
			foot_path((float)time, h, w, period, &foot_xy_1);
			//foot_path_fixed((uint32_t)(time * 1000.f), (int32_t)(h * one_16), (int32_t)(w * one_16), (int32_t)(period * 4096.f), &foot_xy_1_32b);
			
			/*Generate base for second half of the legs*/
			vect3_t foot_xy_2;
			vect3_32b_t foot_xy_2_32b;
			foot_path((float)time+period/2.f, h, w, period, &foot_xy_2);
			//foot_path_fixed((uint32_t)((time + period/2) * 1000.f), (int32_t)(h* one_16), (int32_t)(w* one_16), (int32_t)(period * 4096.f), &foot_xy_2_32b);
			//for (int i = 0; i < 3; i++)
			//	foot_xy_2.v[i] = (float)foot_xy_2_32b.v[i] / one_16;

			/*Rotate and translate*/
			float forward_direction_angle = 0.f;	//y axis!
			mat4_t xrot = Hx(HALF_PI);
			mat4_t zrot = Hz(forward_direction_angle - HALF_PI);
			vect3_t tmp;
			htmatrix_vect3_mult(&xrot, &foot_xy_1, &tmp);
			htmatrix_vect3_mult(&zrot, &tmp, &foot_xy_1);	//done 1
			htmatrix_vect3_mult(&xrot, &foot_xy_2, &tmp);
			htmatrix_vect3_mult(&zrot, &tmp, &foot_xy_2);	//done 2
			//Note: can cut down on 2 rotations (the x rotations) by just doing xz generation for foot_path instead of xy

			//float fw_motion = (w*2.f / period)*scf*time;
			//dynahex_hw_b.m[1][3] = fw_motion - 6.f;

			for (int leg = 0; leg < 6; leg++)
			{
				
				mat4_t lrot = Hz((TWO_PI / 6.f) * (float)leg);
				vect3_t o_motion_b = { 270.f, 0.f,-270.f };
				htmatrix_vect3_mult(&lrot, &o_motion_b, &tmp);
				o_motion_b = tmp;
				 				
				vect3_t targ_b;
				for (int i = 0; i < 3; i++)
				{
					if (leg % 2 == 0)
					{
						targ_b.v[i] = foot_xy_1.v[i] + o_motion_b.v[i];
					}
					else
					{
						targ_b.v[i] = foot_xy_2.v[i] + o_motion_b.v[i];
					}
				}

				/*do gd IK. result is loaded into q*/
				mat4_t* hb_0 = &dynahex_bones->leg[leg].chain[0].him1_i;
				joint* start = &dynahex_bones->leg[leg].chain[1];
				joint* end = start->child->child;
				vect3_t zero = { {0,0,0} };
				vect3_t anchor_b;

				/*Either one of these does IK for the hexapod. 
				* the top one is numerical and expensive.
				* for some reason, it seems to do better on my microcontroller
				* 
				* the bottom one is closed form, and should be faster than the gd one.
				*/
				//gd_ik_single(hb_0, start, end, &zero, &targ_b, &anchor_b, 20000.f);				//UNCOMMENT TO DO IK AND LOAD OUT NEW POSITIONS TO THE LEG!!!!!!
				//ik_closedform_hexapod(hb_0, start, &targ_b);


				/*This code checks the distance in mm between generated setpoint (by ik) and true setpoint (targ)
				*/
				//forward_kinematics(hb_0, start);
				//vect3_t ep = h_origin(end->hb_i);
				//vect3_t dif;
				//for (int i = 0; i < 3; i++)
				//	dif.v[i] = ep.v[i] - targ_b.v[i];
				//float mdif = vect_mag(dif.v,3);
				//printf("mdif = %f\r\n",mdif);
			}
		}



		/*
		* UDP stuff to control a hexapod leg (scale to whole robot later)
		*/
		if (tick > udpsend_ts)	//periodically send a ping pessage so the server knows to point the hose at us
		{
			udpsend_ts = tick + 750;
			sendto(robot_client.s, "hello", 5, 0, (struct sockaddr*)&robot_client.si_other, robot_client.slen);
		}
		int recieved_length = recvfrom(robot_client.s, (char*)udp_rx_buf, BUFLEN, 0, (struct sockaddr*)&(robot_client.si_other), &robot_client.slen);
		if (recieved_length > 0)
		{
			if ((recieved_length % 4) == 0 && (recieved_length/4) == 18)	//checksum not sent over udp. check size of packet to confirm load
			{
				int i = 0;
				for (int leg = 0; leg < 6; leg++)
				{
					for (int joint = 1; joint <= 3; joint++)
					{
						int32_t val = ((int32_t*)udp_rx_buf)[i];
						float fval = (float)val;
						dynahex_bones->leg[leg].chain[joint].q = fval / 4096.f;
						i++;

						udp_connected_ts = tick;
					}
				}
			}
		}
#ifdef GET_HEXAPOD_OFFSET_VALS
		/*offset capture:
		* 1. match the orientation of the robot pre-udp connection
		* 2. connect via udp/power on the robot
		* 3. record the full configuration. each value is the offset, for each leg
		*/
		float qref[3] = { q1_calib, q2_calib, q3_calib };
		for(int leg = 0; leg < 6; leg++)
		{
			printf("l%d:", leg);
			for (int joint = 1; joint <= 3; joint++)
			{
				if (joint < 3)
					printf("%f, ", dynahex_bones->leg[leg].chain[joint].q - qref[joint - 1]);
				else
					printf("%f", dynahex_bones->leg[leg].chain[joint].q - qref[joint - 1]);
			}
			printf(" ");
		}
		printf(")\r\n");
#endif



		model = ht_matrix_to_mat4_t(dynahex_hw_b);
		lightingShader.setMat4("model", model);
		dynahex_modellist[4].Draw(lightingShader, NULL);
		forward_kinematics_dynahexleg(dynahex_bones);
		for (int l = 0; l < 6; l++)
		{
			joint* j = dynahex_bones->leg[l].chain;
			vect3_t o3 = h_origin(dynahex_bones->leg[l].chain[3].hb_i);
			calc_J_point(&j->him1_i, j->child, &o3);
			for (int i = 0; i < 4; i++)
			{
				//dynahex_modellist[i].hb_model = &j[i].hb_i;
				mat4_t hw_i;
				mat4_t_mult_pbr(&dynahex_hw_b, &j[i].hb_i, &hw_i);

				model = ht_matrix_to_mat4_t(hw_i);
				lightingShader.setMat4("model", model);
				dynahex_modellist[i].Draw(lightingShader, NULL);
			}
		}
		
		simpletree_jointlist[0]->q = 0.5f* (float)sin(time);
		simpletree_jointlist[2]->q = 0.5f * (float)sin(time);

		simpletree_jointlist[1]->q = 0.5f * (float)sin(time);
		simpletree_jointlist[5]->q = 0.5f * (float)sin(time);

		simpletree_jointlist[3]->q = .3f*(float)sin(time*5)-2;
		simpletree_jointlist[4]->q = -.3f*(float)sin(time*5)+2;
		
		for (int joint = 0; joint < 18; joint++)
		{
			hexjoints[joint].q = 0;// 1.f + .1f * (float)sin(time * 10.f + (float)joint);
		}
		tree_dfs(&hexbase);
		{
			mat4_t hw_b = Hscale(10.f);
			hw_b.m[0][3] = -5.f;
			hw_b.m[1][3] = 0;
			hw_b.m[2][3] = 3.f;
			hw_b = mat4_t_mult(hw_b, Hz(0));
			for (int i = 0; i < 3; i++)		//assign target to this robot
				target.v[i] = hw_b.m[i][3];
			model = ht_matrix_to_mat4_t(hw_b);
			lightingShader.setMat4("model", model);
			render_robot(&hw_b, &lightingShader, &hexbase);
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
			lightCubeShader.setVec4("obj_color", light[i].base_color);
			glDrawArrays(GL_TRIANGLES, 0, 36);
		}

		glfwSwapBuffers(window);
		glfwPollEvents();
		if (glfwGetKey(window, GLFW_KEY_ESCAPE) == GLFW_PRESS)
			glfwSetWindowShouldClose(window, true);
	}
	delete[] base.joints;
	delete[] link1.joints;
	delete[] link2.joints;
	
	
	delete[] link5.joints;

	glfwTerminate();
	return 0;
}