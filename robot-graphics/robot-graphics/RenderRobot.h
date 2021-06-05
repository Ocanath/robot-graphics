#ifndef RENDER_ROBOT
#define RENDER_ROBOT
#include "kinematics.h"
#include "model.h"

typedef struct RenderRobot_t
{
	mat4 hw_b;
	vector<AssetModel>;
}RenderRobot_t;

#endif // !RENDER_ROBOT
