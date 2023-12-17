#include "Grips.h"
#define HAND_CLOSED_POS 	                90.0f
#define HAND_OPEN_POS 		                10.0f

/*
This file contains system default grips. To change the grips for a particular hand, use the API or
Bluetooth grip update capability.
*/
/******************************************************************************/

/*This configuration is special. It is used as a placeholder. If a grip in the systemdefault list
does not contain this config, but the grip in the filesystem DOES contain this config, then that
grip will be overwritten by the system default configuration.
*/
grip_t unused_noset = {
	{
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f}
	}
};
/*******************************/

grip_t chuck_grasp = {
	{
		{1, 64.0f},
		{1, 64.0f},
		{1, 100.0f},
		{1, 100.0f},
		{1, 50.0f},
		{0, -80.0f}
	}
};
grip_t chuck_ok_grasp = {
	{
		{1, 64.0f},
		{1, 64.0f},
		{1, HAND_OPEN_POS},
		{1, HAND_OPEN_POS},
		{1, 50.0f},
		{0, -80.0f}
	}
};
grip_t open_chuck_ok_grasp = {
	{
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, -80.0f}
	}
};
grip_t pinch_grasp = {
	{
		//wp1					 wp2
		{1, 59.6f},					//INDEX
		{1, 30.f},			//MIDDLE
		{1, 22.5f},			//RING
		{1, 15.f},			//PINKY
		{1, 44.38},					//FLEXOR
		{0, -80.f}					//ROTATOR
	}
};
grip_t open_pinch_grasp = {
	{
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{0, -80.f}
	}
};
grip_t power_grasp = {
	{
		{1, HAND_CLOSED_POS},	//INDEX
		{1, HAND_CLOSED_POS},	//Middle
		{1, HAND_CLOSED_POS},	//Ring
		{1, HAND_CLOSED_POS},	//Pinky
		{2, 55.0f},			//Flexor
		{0, -100.0f}			//Rotator
	}
};
grip_t open_power_grasp = {
	{
		{1, HAND_OPEN_POS},
		{1, HAND_OPEN_POS},
		{1, HAND_OPEN_POS},
		{1, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{1, -100.0f}
	}
};
grip_t key_grasp = {
	{
		{0, 92.f},		//INDEX
		{0, 92.f},		//MIDDLE
		{0, 92.f},		//RING
		{0, 92.f},		//PINKY
		{0, 75.0f},				//FLEXOR
		{0, -30.0f}				//ROTATOR
	}
};
grip_t handshake_grasp = {
	{
		{0, 84.0f},	//INDEX
		{0, 84.0f},	//MIDDLE
		{0, 84.0f},	//RING
		{0, 84.0f},	//PINKY
		{1, 55.0f},	//FLEXOR
		{0, -45.0f}	//ROTATOR
	}
};
grip_t three_finger_trigger_grasp = {
	{
		{5, HAND_CLOSED_POS},
		{5, HAND_CLOSED_POS},
		{2, HAND_CLOSED_POS},
		{1, HAND_CLOSED_POS},
		{3, 30.0f},
		{0, -115.0f}
	}
};
grip_t point_grasp = {
	{
		{1, HAND_OPEN_POS},
		{1, HAND_CLOSED_POS},
		{1, HAND_CLOSED_POS},
		{1, HAND_CLOSED_POS},
		{2, 42.0f},
		{0, -110.0f}
	}
};
grip_t relax_grasp = {
	{
		{0,	15.0f},
		{0,	15.0f},
		{0,	15.0f},
		{0,	15.0f},
		{0,	40.0f},
		{-1,-15.0f}
	}
};
grip_t open_relax = {
	{
		{-1,	15.0f},
		{-1,	15.0f},
		{-1,	15.0f},
		{-1,	15.0f},
		{-1,	40.0f},
		{-1,	-15.0f}
	}
};
grip_t sign_of_the_horns_grasp = {
	{
		{0, HAND_OPEN_POS},	//INDEX
		{0, HAND_CLOSED_POS},	//MIDDLE
		{0, HAND_CLOSED_POS},	//RING
		{0, HAND_OPEN_POS},	//PINKY
		{2, 45.0f},			//FLEXOR
		{1, -100.0f}			//ROTATOR
	}
};
grip_t rude_point_grasp = {
	{
		{0, HAND_CLOSED_POS},	//INDEX
		{0, HAND_OPEN_POS},	//MIDDLE
		{0, HAND_CLOSED_POS},	//RING
		{0, HAND_CLOSED_POS},	//PINKY
		{2, 40.0f},			//FLEXOR
		{1, -50.0f}			//ROTATOR
	}
};
grip_t mode_switch_close_grasp = {
	{
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f},
		{-1, 0.f}		//prio is negative 
	}
};
grip_t key_open_grasp = {
	{
		{1, 92.f},	//index
		{1, 92.f},	//middle
		{1, 92.f},	//ring
		{1, 92.f},	//pinky
		{0, HAND_OPEN_POS},	//flexor
		{1, -30.0f}	//rotator		
	}
};
grip_t open_thumb_down_grasp = {
	{
		{1, HAND_OPEN_POS},	//index
		{1, HAND_OPEN_POS},	//middle
		{1, HAND_OPEN_POS},	//ring
		{1, HAND_OPEN_POS},	//pinky
		{0, HAND_OPEN_POS},	//flexor
		{1, -HAND_OPEN_POS}						//rotator
	}
};
grip_t three_finger_trigger_open_grasp = {
	{
		{0, HAND_OPEN_POS},
		{0, HAND_OPEN_POS},
		{4, HAND_OPEN_POS},
		{5, HAND_OPEN_POS},
		{2, 15.0f},
		{0, -115.0f}
	}
};
grip_t open_point_grasp = {
	{
		{1, 15.0f},
		{1, 15.0f},
		{1, 15.0f},
		{1, 15.0f},
		{0, 15.0f},
		{2, -15.0f},
	}
};
grip_t mode_switch_thumb_no_open = {
	{
		{-1, 0},	//index
		{-1, 0},	//middle
		{-1, 0},	//ring
		{-1, 0},	//pinky
		{-1, 0},	//flexor
		{-1, 0}						//rotator
	}
};
grip_t open_handshake_grasp = {
	{
		{1, HAND_OPEN_POS},	//INDEX
		{1, HAND_OPEN_POS},	//MIDDLE
		{1, HAND_OPEN_POS},	//RING
		{1, HAND_OPEN_POS},	//PINKY
		{0, HAND_OPEN_POS},	//FLEXOR
		{1, -45.0f}			//ROTATOR	
	}
};

grip_t utility_grasp = {
	{
		{0, 65},	//INDEX
		{0, 65},	//MIDDLE
		{0, 65},	//RING
		{0, 65},	//PINKY
		{0, 45},	//FLEXOR
		{0, -110.0f}			//ROTATOR	
	}
};
grip_t utility_open = {
	{
		{0, 15},	//INDEX
		{0, 15},	//MIDDLE
		{0, 15},	//RING
		{0, 15},	//PINKY
		{0, 10},	//FLEXOR
		{0, -110.0f}			//ROTATOR	
	}
};

grip_t mouse_grasp = {
	{
		{1, 36.f},
		{0, 1.f},
		{0, 1.f},
		{0, 30.f},
		{0, 70.f},
		{0, -55.f}
	}
};

grip_t mouse_open = {
	{
		{0, 10.f},
		{0, 1.f},
		{0, 1.f},
		{0, 30.f},
		{0, 70.f},
		{0, -55.f}
	}
};

/*
NOTE (CRITICAL):
THIS ARRAY IS FORMATTED
close grips are the first N/2 grip_t configurations
open grips is the last N/2 grip_t configurations

To add a new grip:
1. create a new close and open configuration above
2. Change the entry &unused_noset in the FIRST half of the array to point to the new CLOSE config you created
3. Change the entry &unused_noset in the SECOND half of the array to point to the new OPEN config you created
4. Compile, create an OTA package, and OTA update the hand.


If the total blank entries are exceeded, the NUM_GRASPS and NUM_GRIP_CONFIGURATIONS
defines must be modified, and the new entries placed accordingly. This will create
bad references in the filesystem, so the whole thing will need to be overwritten.
*/
grip_t* system_griplist_default[NUM_GRIP_CONFIGURATIONS] = {
	&chuck_grasp,
	&chuck_ok_grasp,
	&pinch_grasp,
	&power_grasp,
	&key_grasp,
	&handshake_grasp,
	&three_finger_trigger_grasp,
	&point_grasp,
	&relax_grasp,
	&sign_of_the_horns_grasp,
	&rude_point_grasp,
	&mode_switch_close_grasp,
	&utility_grasp,
	&mouse_grasp,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,			//END CLOSE GRIPS (32)
	&open_chuck_ok_grasp,	//BEGIN OPEN GRIPS	chuck
	&open_chuck_ok_grasp,
	&open_pinch_grasp,	//pinch
	&open_power_grasp,
	&key_open_grasp,
	&open_handshake_grasp,
	&three_finger_trigger_open_grasp,
	&open_point_grasp,
	&open_relax,			//relax
	&open_thumb_down_grasp,	//horns
	&open_thumb_down_grasp,	//rude point
	&mode_switch_thumb_no_open,
	&utility_open,
	&mouse_open,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset,
	&unused_noset
};





