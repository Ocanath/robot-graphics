#ifndef GRIPS_H_
#define GRIPS_H_
#include "stdint.h"

#define NUM_GRASPS 32
#define NUM_GRIP_CONFIGURATIONS 64	//Twice the number of grips

/*Enumeration of all configuration indices, which
index into the griplist to provide a given grip
configuration*/
enum {
	GENERAL_OPEN_CMD = -1,
	CHUCK_GRASP_CFG_IDX = 0,
	CHUCK_OK_GRASP_CFG_IDX = 1,
	PINCH_GRASP_CFG_IDX = 2,
	POWER_GRASP_CFG_IDX = 3,
	KEY_GRASP_CFG_IDX = 4,
	HANDSHAKE_CFG_IDX = 5,
	TRIGGER_CFG_IDX = 6,
	POINT_GRASP_CFG_IDX = 7,
	RELAX_CFG_IDX = 8,
	SIGN_OF_THE_HORNS_CFG_IDX = 9,
	RUDE_POINT_GRASP_CFG_IDX = 10,
	MODE_SWITCH_CLOSE_CFG_IDX = 11,
	UTILITY_GRASP_CFG_IDX = 12,
	UGRIP_1_CFG_IDX = 13,
	UGRIP_2_CFG_IDX = 14,
	UGRIP_3_CFG_IDX = 15,
	UGRIP_4_CFG_IDX = 16,
	UGRIP_5_CFG_IDX = 17,
	UGRIP_6_CFG_IDX = 18,
	UGRIP_7_CFG_IDX = 19,
	UGRIP_8_CFG_IDX = 20,
	UGRIP_9_CFG_IDX = 21,
	UGRIP_10_CFG_IDX = 22,
	UGRIP_11_CFG_IDX = 23,
	UGRIP_12_CFG_IDX = 24,
	UGRIP_13_CFG_IDX = 25,
	UGRIP_14_CFG_IDX = 26,
	UGRIP_15_CFG_IDX = 27,
	UGRIP_16_CFG_IDX = 28,
	UGRIP_17_CFG_IDX = 29,
	UGRIP_18_CFG_IDX = 30,
	UGRIP_19_CFG_IDX = 31
};

#define SKIP_GRIP 255
#define PREV_GRIP 254
#define IS_OPEN 255
#define RESERVED_NON_GRIP 255

#define NUM_CHANNELS 6

typedef struct waypoint_p
{
	int8_t pidx;
	float qd;
}waypoint_p;

/*
Grip configuration type.
Finger index aligned array of waypoints, which
are setpoints and order/priority assignments.
*/
typedef struct grip_t
{
	waypoint_p wp[NUM_CHANNELS];
}grip_t;


extern grip_t* system_griplist_default[NUM_GRIP_CONFIGURATIONS];	//system default grip definitions

#endif
