#ifndef ABILITY_HAND_RENDERING_H
#define ABILITY_HAND_RENDERING_H
#include "finger-kinematics.h"
#include "Grips.h"

struct key_lookup_entry
{
	int key_val;
	int grip_cfg_idx;
};

extern const struct key_lookup_entry key_lookup[NUM_GRIPKEYS];

#endif // !ABILITY_HAND_RENDERING_H

