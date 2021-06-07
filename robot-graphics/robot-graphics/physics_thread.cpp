#include "physics_thread.h"

int gl_start_dynamics_flag = 0;
int run_dynamics_thread = 0;
vect3 keyboard_ctl_vect = { 0,0,0 };
std::vector<RenderBulletObject*> gl_shared_cubes;
RenderBulletObject * gl_player_cube = NULL;
RenderBulletObject ground_cube;


btRigidBody* load_cube(btScalar mass, vect3 dim, vect3 initpos, btDiscreteDynamicsWorld* dynamics_world)
{
	btCollisionShape* col_shape = new btBoxShape(btVector3(btScalar(dim.v[0] / 2), btScalar(dim.v[1] / 2), btScalar(dim.v[2] / 2)));
	btTransform start_transform;
	start_transform.setIdentity();
	start_transform.setOrigin(btVector3(btScalar(initpos.v[0]), btScalar(initpos.v[1]), btScalar(initpos.v[2])));

	bool is_dynamic = (mass != 0.f);
	btVector3 local_inertia(0, 0, 0);
	if (is_dynamic)
		col_shape->calculateLocalInertia(mass, local_inertia);

	btDefaultMotionState* motion_state = new btDefaultMotionState(start_transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motion_state, col_shape, local_inertia);
	btRigidBody* body = new btRigidBody(rbInfo);
	dynamics_world->addRigidBody(body);
	return body;
}


void physics_thread(void)
{
	btDefaultCollisionConfiguration* collision_config = new btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new btCollisionDispatcher(collision_config);
	btBroadphaseInterface* overlappingPairCache = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;
	btDiscreteDynamicsWorld* dynamics_world = new btDiscreteDynamicsWorld(dispatcher, overlappingPairCache, solver, collision_config);

	dynamics_world->setGravity(btVector3(0, 0, 0));

	printf("dynamics thread prepared, waiting for signal from render thread\n");
	while (gl_start_dynamics_flag == 0);
	printf("signal received. starting dynamics now\n");
	double prev_time = glfwGetTime();

	std::vector<RenderBulletObject> cubelist;
	for (int x = -200; x < 200; x += 50)
	{
		for (int y = -200; y < 200; y += 50)
		{
			for (int z = 1; z < 200; z += 50)
			{
				RenderBulletObject cube;
				cube.hw_cube = mat4_Identity;
				cube.hw_cube.m[0][3] = x;
				cube.hw_cube.m[1][3] = y;
				cube.hw_cube.m[2][3] = z;
				//cube[i].cube_dim = { {i, i, i*i/10+1} };
				cube.cube_dim = { { 3, 1, 1 } };
				float cubevol = cube.cube_dim.v[0] * cube.cube_dim.v[1] * cube.cube_dim.v[2];
				cube.body = load_cube(1 * cubevol, cube.cube_dim, h_origin(cube.hw_cube), dynamics_world);
				cubelist.push_back(cube);
				btVector3 av = btVector3(x, y, z);
				av.normalize();
				av = av * (btScalar(3));
				cube.body->setAngularVelocity(av);
			}
		}
	}
	for (int i = 0; i < cubelist.size(); i++)
		gl_shared_cubes.push_back(&(cubelist[i]));

	//ground_cube.cube_color = { { 1 * .1, .5 * .1, .31 * .1 } };
	ground_cube.cube_dim = { { 10000, 10000, 1000 } };
	ground_cube.hw_cube = { {
		{ 1, 0, 0, 0 },
		{ 0, 1, 0, 0 },
		{ 0, 0, 1, -510 },
		{ 0, 0, 0, 1 }
	} };
	ground_cube.body = load_cube(0, ground_cube.cube_dim, h_origin(ground_cube.hw_cube), dynamics_world);
	gl_shared_cubes.push_back(&ground_cube);


	RenderBulletObject player_cube;
	player_cube.cube_dim = { { 5, 5, 5 } };
	//player_cube.body = load_cube(1, player_cube.cube_dim, h_origin(player_cube.hw_cube), dynamics_world);
	btCollisionShape* col_shape = new btSphereShape(10.0f);//btBoxShape(btVector3(btScalar(dim.v[0] / 2), btScalar(dim.v[1] / 2), btScalar(dim.v[2] / 2)));
	btTransform start_transform;
	start_transform.setIdentity();
	start_transform.setOrigin(btVector3(0, 0, 20));
	btScalar mass = 10.0f;
	bool is_dynamic = (mass != 0.f);
	btVector3 local_inertia(0, 0, 0);
	if (is_dynamic)
		col_shape->calculateLocalInertia(mass, local_inertia);
	btDefaultMotionState* motion_state = new btDefaultMotionState(start_transform);
	btRigidBody::btRigidBodyConstructionInfo rbInfo(mass, motion_state, col_shape, local_inertia);
	player_cube.body = new btRigidBody(rbInfo);
	dynamics_world->addRigidBody(player_cube.body);
	gl_player_cube = &player_cube;


	while (run_dynamics_thread == 1)
	{
		double time = glfwGetTime();
		dynamics_world->stepSimulation(time - prev_time, 10);
		prev_time = time;
		for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
		{
			btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
			btRigidBody* body = btRigidBody::upcast(obj);
			body->getMotionState();
		}
		cubelist[0].body->applyCentralForce(btVector3(0, 0, -9.8));


		gl_player_cube->body->applyCentralForce(btVector3(0, 0, keyboard_ctl_vect.v[2]));
		btVector3 refV = btVector3(keyboard_ctl_vect.v[0], keyboard_ctl_vect.v[1], keyboard_ctl_vect.v[2]);
		btVector3 ctl = btScalar(300.0) * (refV - gl_player_cube->body->getLinearVelocity());
		ctl.setZ(0.);

		gl_player_cube->body->applyCentralForce(ctl);
		//gl_player_cube->body->setLinearVelocity((btVector3(keyboard_ctl_vect.v[0], keyboard_ctl_vect.v[1], 0)));
		btScalar m = gl_player_cube->body->getMass();
		gl_player_cube->body->applyCentralForce(btVector3(0, 0, -9.8 * m));

		//btVector3 o_targ = btVector3(20 * sin(time), 20 * cos(time), 10);
		//if (gl_shared_cubes.size() >= 10)
		//{
		//	for (int i = 1; i < 10; i++)
		//	{
		//		btRigidBody * body = gl_shared_cubes[i]->body;

		//		btScalar m = body->getMass();
		//		btScalar k = 30.0f*m;
		//		btScalar d = 30.0f;


		//		btVector3 o_obj = btVector3(gl_shared_cubes[i]->hw_cube.m[0][3], gl_shared_cubes[i]->hw_cube.m[1][3], gl_shared_cubes[i]->hw_cube.m[2][3]);

		//		btVector3 velocity = body->getLinearVelocity();
		//		btVector3 vtarg = (o_targ - o_obj)*k - velocity*d;

		//		body->applyCentralForce(vtarg);
		//		body->applyCentralForce(btVector3(0, 0, -9.8*m));
		//	}
		//}

	}
	for (int i = dynamics_world->getNumCollisionObjects() - 1; i >= 0; i--)
	{
		btCollisionObject* obj = dynamics_world->getCollisionObjectArray()[i];
		btRigidBody* body = btRigidBody::upcast(obj);
		delete body->getCollisionShape();
	}

	delete dynamics_world;
	delete solver;
	delete overlappingPairCache;
	delete dispatcher;
	delete collision_config;
}