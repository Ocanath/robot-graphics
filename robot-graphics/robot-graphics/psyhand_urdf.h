#ifndef PSYHAND_URDF_H
#define PSYHAND_URDF_H
#include "kinematics.h"
#include "model.h"

#define NUM_MODELS 6

class AbilityHandLeftUrdf
{
	private:
		joint2 abhjoints[11] = { 0 };
		link_t wrist = { 0 };
		link_t palm = { 0 };
		link_t fl1[4] = { 0 };
		link_t fl2[4] = { 0 };
		link_t tl1 = { 0 };
		link_t tl2 = { 0 };

		const char* model_names[NUM_MODELS] = {
			"abh_models/wristmesh.STL",
			"abh_models/FB_palm_ref.STL",
			"abh_models/idx-F1.STL",
			"abh_models/idx-F2.STL",
			"abh_models/thumb-F1.STL",
			"abh_models/thumb-F2-left.STL"
		};

	public:
		vector<AssetModel> modellist;
		mat4_t hw_b = { 0 };
		AbilityHandLeftUrdf()
		{
			hw_b = mat4_t_Identity;//identity to start

			for (int i = 0; i < NUM_MODELS; i++)
			{
				modellist.push_back((AssetModel)model_names[i]);
			}
			int jointlist_idx = 0;	//out of the pre-allocated joint array, grab chunks of it and label it as used for children. Order is UNIMPORTANT, since the tree is defined by pointer references not joint order in memory.
			
			/*first link*/
			wrist.name = "wrist-adapter";
			wrist.h_base_us = mat4_t_Identity;
			wrist.model_ref = (void*)&modellist[0];
			wrist.joints = &abhjoints[jointlist_idx];
			wrist.num_children = 1;
			jointlist_idx += wrist.num_children;
			wrist.joints[0].child = &palm;
			mat4_t link = get_rpy_xyz_mat4(3.14148426, 0.08848813, 3.14036612, 24.0476665e-3, 3.78124745e-3, 32.32964923e-3);
			wrist.joints[0].h_link = link;


			/*second link: palm*/
			palm.name = "palm";
			palm.model_ref = (void*)&modellist[1];
			palm.joints = &abhjoints[jointlist_idx];
			palm.num_children = 5;
			jointlist_idx += palm.num_children;
			palm.joints[0].h_link = get_rpy_xyz_mat4(-1.982050, 1.284473, -2.090591, -9.49e-3, -13.04e-3, -62.95e-3);	//index q1
			palm.joints[0].child = &fl1[0];
			palm.joints[1].h_link = get_rpy_xyz_mat4(-1.860531, 1.308458, -1.896217, 9.653191e-3, -15.310271e-3, -67.853949e-3);	//middle q1
			palm.joints[1].child = &fl1[1];
			palm.joints[2].h_link = get_rpy_xyz_mat4(-1.716598, 1.321452, -1.675862, 29.954260e-3, -14.212492e-3, -67.286105e-3);	//ring q1
			palm.joints[2].child = &fl1[2];
			palm.joints[3].h_link = get_rpy_xyz_mat4(-1.765110, 1.322220, -1.658383, 49.521293e-3, -11.004583e-3, -63.029065e-3);	//pinky q1
			palm.joints[3].child = &fl1[3];
			palm.joints[4].h_link = get_rpy_xyz_mat4(0, 0, 3.330437, 0, 0, 0);	//thumb_q1
			palm.joints[4].child = &tl1;

			/**/
			fl1[0].name = "index-link-1";
			fl1[1].name = "middle-link-1";
			fl1[2].name = "ring-link-1";
			fl1[3].name = "pinky-link-1";
			for (int i = 0; i < 4; i++)
			{
				fl1[i].model_ref = (void*)&modellist[2];
				fl1[i].joints = &abhjoints[jointlist_idx];
				fl1[i].num_children = 1;
				jointlist_idx += fl1[i].num_children;
				fl1[i].joints[0].h_link = get_rpy_xyz_mat4(0, 0, 0.084474, 38.472723e-3, 3.257695e-3, 0.000000e-3);
				fl1[i].joints[0].child = &fl2[i];
			}

			/**/
			fl2[0].name = "index-link-2";
			fl2[1].name = "middle-link-2";
			fl2[2].name = "ring-link-2";
			fl2[3].name = "pinky-link-2";
			for (int i = 0; i < 4; i++)
			{
				fl2[i].model_ref = (void*)&modellist[3];
				fl2[i].joints = NULL;
				fl2[i].num_children = 0;
			}

			tl1.name = "thumb-link-1";
			tl1.model_ref = (void*)&modellist[4];
			tl1.joints = &abhjoints[jointlist_idx];
			tl1.num_children = 1;
			jointlist_idx += tl1.num_children;
			tl1.joints[0].h_link = get_rpy_xyz_mat4(4.450589592585541, 0, 0, 27.8283501e-3, 0, -14.7507000e-3);
			tl1.joints[0].child = &tl2;

			tl2.name = "thumb-link-2";
			tl2.model_ref = (void*)&modellist[5];
			tl2.joints = NULL;
			tl2.num_children = 0;

			tree_assign_parent(&wrist);
			tree_dfs(&wrist);
		}

		void fk(void)
		{
			tree_dfs(&wrist);
		}
		void load_q(float q[6])
		{
			for (int i = 0; i < 4; i++)
			{
				palm.joints[i].q = q[i] * DEG_TO_RAD;
				palm.joints[i].child->joints[0].q = palm.joints[i].q * 1.05851325f + 0.72349796f;
			}
			palm.joints[4].q = q[5] * DEG_TO_RAD;
			palm.joints[4].child->joints[0].q = q[4] * DEG_TO_RAD;

		}
		void render(Shader * sh)
		{
			render_robot(&hw_b, sh, &wrist);
		}
};


#endif
