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
	delete world;
}

void BulletWorldManager::update() {
	//TODO: XML defined step tine
	if (world) {
		world->stepSimulation(1 / 60.0f);
		rigidManager->update();
		softManager->update();
		if (debugDrawer) {
			world->debugDrawWorld();
		}
	}
}

void BulletWorldManager::setGravity(float x, float y, float z) {
	world->setGravity(btVector3(x, y, z));
}

void BulletWorldManager::addRigid(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform, bool isStatic) {
	if (isStatic) {
		world->addRigidBody(rigidManager->addStaticBody(scene, nbVertices, vertices, nbIndices, indices, transform));
	}
	else {
		world->addRigidBody(rigidManager->addDynamicBody(scene, nbVertices, vertices, nbIndices, indices, transform));
	}
}

void BulletWorldManager::setRigidProperty(std::string scene, std::string propName, float value) {
	if (propName.compare("MASS") == 0) {
		rigidManager->setMass(scene, value);
	}
	if (propName.compare("FRICTION") == 0) {
		rigidManager->setFriction(scene, value);
	}
	if (propName.compare("RESTITUTION") == 0) {
		rigidManager->setRestitution(scene, value);
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

void BulletWorldManager::setDebug(std::vector<float>* debugPoints) {
	if (debugDrawer) {
		debugDrawer->setPoints(debugPoints);
	} else {
		debugDrawer = new BulletDebugger(debugPoints);
		debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		//debugDrawer->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
		world->setDebugDrawer(debugDrawer);
	}
	
}
