#include "bulletWorldManager.h"



BulletWorldManager::BulletWorldManager() {
	btSoftBodyRigidBodyCollisionConfiguration * collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();// btDefaultCollisionConfiguration();
	btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

	btBroadphaseInterface* broadphase = new btDbvtBroadphase();
	btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

	btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);
	btDefaultSoftBodySolver * softbodySolver = new btDefaultSoftBodySolver();
	world = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration, softbodySolver);
	//world = new btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	//world->setGravity(btVector3(0, -10, 0));

	rigidManager = new BulletRigidManager();
	softManager = new BulletSoftManager();
}


BulletWorldManager::~BulletWorldManager() {
	delete debugDrawer;
	delete rigidManager;
	delete softManager;
	delete world;
}

void BulletWorldManager::update() {
	if (world) {
		world->stepSimulation(timeStep);
		rigidManager->update();
		softManager->update();
		if (debugDrawer) {
			debugDrawer->clear();
			world->debugDrawWorld();
		}
	}
}

void BulletWorldManager::setGravity(float x, float y, float z) {
	world->setGravity(btVector3(x, y, z));
}

void BulletWorldManager::addRigid(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform, nau::physics::IPhysics::BoundingVolume shape, float mass, bool isStatic) {
	rigidManager->createInfo(scene, nbVertices, vertices, nbIndices, indices, transform);
	world->addRigidBody(
		rigidManager->addRigid(
			scene,
			rigidManager->createCollisionShape(scene, shape, isStatic),
			mass,
			isStatic
		)
	);
}

void BulletWorldManager::setRigidProperty(std::string scene, std::string propName, float value) {
	if (propName.compare("MASS") == 0) {
		rigidManager->setMass(scene, value);
	}
	else if (propName.compare("FRICTION") == 0 || propName.compare("DYNAMIC_FRICTION") == 0 || propName.compare("STATIC_FRICTION") == 0) {
		rigidManager->setFriction(scene, value);
	}
	else if (propName.compare("ROLLING_FRICTION") == 0) {
		rigidManager->setRollingFriction(scene, value);
	}
	else if (propName.compare("RESTITUTION") == 0) {
		rigidManager->setRestitution(scene, value);
	}
}

void BulletWorldManager::setRigidProperty(std::string scene, std::string propName, float * value) {
	if (propName.compare("IMPULSE") == 0) {
		rigidManager->addImpulse(scene, value);
	}
	else if (propName.compare("INERTIA") == 0) {
		rigidManager->setLocalInertia(scene, value);
	}
}

void BulletWorldManager::moveRigid(std::string scene, float * transform) {
	rigidManager->move(scene, transform); 
}

void BulletWorldManager::addCloth(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	world->addSoftBody(softManager->addSoftBody(world->getWorldInfo(), scene, nbVertices, vertices, nbIndices, indices, transform));
}

void BulletWorldManager::setSoftProperty(std::string scene, std::string propName, float value) {
	//TODO: Check properties for cloths
}

void BulletWorldManager::moveSoft(std::string scene, float * transform) {
	softManager->move(scene, transform);
}

void BulletWorldManager::setDebug() {
	if (!debugDrawer) {
		debugDrawer = new BulletDebugger();
		debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		//debugDrawer->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
		world->setDebugDrawer(debugDrawer);
	}
}
