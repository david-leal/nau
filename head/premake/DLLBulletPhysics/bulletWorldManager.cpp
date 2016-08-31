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
	if (propName.compare("MASS") == 0)
		rigidManager->setMass(scene, value);
	else if (propName.compare("FRICTION") == 0 || propName.compare("DYNAMIC_FRICTION") == 0 || propName.compare("STATIC_FRICTION") == 0) 
		rigidManager->setFriction(scene, value);
	else if (propName.compare("ROLLING_FRICTION") == 0) 
		rigidManager->setRollingFriction(scene, value);
	else if (propName.compare("RESTITUTION") == 0) 
		rigidManager->setRestitution(scene, value);
}

void BulletWorldManager::setRigidProperty(std::string scene, std::string propName, float * value) {
	if (propName.compare("IMPULSE") == 0)
		rigidManager->addImpulse(scene, value);
	else if (propName.compare("INERTIA") == 0)
		rigidManager->setLocalInertia(scene, value);
}

void BulletWorldManager::moveRigid(std::string scene, float * transform) {
	rigidManager->move(scene, transform); 
}

void BulletWorldManager::addCloth(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform, nau::physics::IPhysics::SceneCondition condition, float * conditionValue) {
	switch (condition)
	{
	case nau::physics::IPhysics::GT:
		softManager->createInfo(scene, nbVertices, vertices, nbIndices, indices, transform, CLOTH_CONDITION_GT, conditionValue);
		break;
	case nau::physics::IPhysics::LT:
		softManager->createInfo(scene, nbVertices, vertices, nbIndices, indices, transform, CLOTH_CONDITION_LT, conditionValue);
		break;
	case nau::physics::IPhysics::EGT:
		softManager->createInfo(scene, nbVertices, vertices, nbIndices, indices, transform, CLOTH_CONDITION_EGT, conditionValue);
		break;
	case nau::physics::IPhysics::ELT:
		softManager->createInfo(scene, nbVertices, vertices, nbIndices, indices, transform, CLOTH_CONDITION_ELT, conditionValue);
		break;
	case nau::physics::IPhysics::EQ:
		softManager->createInfo(scene, nbVertices, vertices, nbIndices, indices, transform, CLOTH_CONDITION_EQ, conditionValue);
		break;
	default:
		softManager->createInfo(scene, nbVertices, vertices, nbIndices, indices, transform, CLOTH_CONDITION_NONE, conditionValue);
		break;
	}
	world->addSoftBody(softManager->addSoftBody(world->getWorldInfo(), scene));
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

void BulletWorldManager::addCamera(const std::string & scene, float * position, float * up, float pace, float minPace, float hitMagnitude, float timeStep, float stepOffset, float mass, float radius, float height) {
}

std::map<std::string, float*>* BulletWorldManager::getCameraPositions() {
	//TODO: Change This When Character manager exists
	return new std::map<std::string, float*>;
}
