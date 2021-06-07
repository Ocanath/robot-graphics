#ifndef PHYSICS_THREAD_H
#define PHYSICS_THREAD_H

#include "RenderBullet.h"
#include "btBulletDynamicsCommon.h"
#include <stdio.h>

extern int gl_start_dynamics_flag;
extern int run_dynamics_thread;
extern vect3 keyboard_ctl_vect;
extern std::vector<RenderBulletObject*> gl_shared_cubes;
extern RenderBulletObject* gl_player_cube;
extern RenderBulletObject ground_cube;

void physics_thread(void);


#endif