#include "physXWorldManager.h"

using namespace physx;

PhysXWorldManager::PhysXWorldManager() {
	PxTolerancesScale scale = PxTolerancesScale();
	PxFoundation* gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = &PxProfileZoneManager::createProfileZoneManager(gFoundation);
	PxPhysics*	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, scale, true, profileZoneManager);
	if (gPhysics->getPvdConnectionManager()) {
		gPhysics->getVisualDebugger()->setVisualizeConstraints(true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_SCENEQUERIES, true);
		//gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONSTRAINTS, true);
		gConnection = PxVisualDebuggerExt::createConnection(gPhysics->getPvdConnectionManager(), "127.0.0.1", 5425, 100);
	}
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	//sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	PxDefaultCpuDispatcher* gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVETRANSFORMS;
	world = gPhysics->createScene(sceneDesc);
	//m_pDynamicsWorld->setFlag(PxSceneFlag::eENABLE_ACTIVETRANSFORMS, true);

	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(scale));

	rigidManager = new PhysXRigidManager();
	softManager = new PhysXSoftManager();
	particleManager = new PhysXParticleManager();
	characterManager = new PhysXCharacterManager(world);
}

PhysXWorldManager::~PhysXWorldManager() {
	if (gConnection)
		gConnection->release();
	world->release();
	delete &world;
	delete rigidManager;
}

void PhysXWorldManager::update() {
	//TODO: XML defined step tine
	if (world) {
		float time = 1 / 60.0f;
		world->simulate(time);
		world->fetchResults(true);

		//RIGID BODIES UPDATE
		PxU32 nbActiveTransforms;
		const PxActiveTransform* activeTransforms = world->getActiveTransforms(nbActiveTransforms);
		rigidManager->update(activeTransforms, nbActiveTransforms);
		
		//SOFT BODIES UPDATE
		softManager->update();

		//PARTICLE UPDATE
		particleManager->update();

		//CHARACTER UPDATE
		characterManager->update(time, world->getGravity());
	}
}

void PhysXWorldManager::setGravity(float x, float y, float z) {
		world->setGravity(PxVec3(x, y, z));
}

void PhysXWorldManager::addRigid(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform, float dynamicFrction, float staticFriction, float restitution, bool isStatic) {
	if (isStatic) {
		rigidManager->addStaticBody(world, mCooking, scene, nbVertices, vertices, nbIndices, indices, transform, staticFriction, dynamicFrction, restitution);
	}
	else {
		   rigidManager->addDynamicBody(world, mCooking, scene, nbVertices, vertices, nbIndices, indices, transform, staticFriction, dynamicFrction, restitution);
	}
}

void PhysXWorldManager::setRigidProperty(std::string scene, std::string propName, float value) {
	if (propName.compare("MASS") == 0) {
		rigidManager->setMass(scene, value);
	}
	if (propName.compare("STATIC_FRICTION") == 0) {
		rigidManager->setStaticFriction(scene, value);
	}
	if (propName.compare("DYNAMIC_FRICTION") == 0) {
		rigidManager->setDynamicFriction(scene, value);
	}
	if (propName.compare("RESTITUTION") == 0) {
		rigidManager->setRestitution(scene, value);
	}
}

void PhysXWorldManager::moveRigid(std::string scene, float * transform) {
	rigidManager->move(scene, transform);
}

void PhysXWorldManager::addCloth(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	softManager->addSoftBody(world, scene, nbVertices, vertices, nbIndices, indices, transform);
}

void PhysXWorldManager::setSoftProperty(std::string scene, std::string propName, float value) {
	// TODO: Check properties for cloths
}

void PhysXWorldManager::moveSoft(std::string scene, float * transform) {
	softManager->move(scene, transform);
}

void PhysXWorldManager::addParticles(const std::string &scene, const std::string &material, float maxParticles, float * positions, float *transform) {
	particleManager->addParticleSystem(world, scene, material, maxParticles, positions, transform);
}

std::map<std::string, int>* PhysXWorldManager::getMaterialParticleNb() {
	return particleManager->getParticleSystemsParticleNb();
}

void PhysXWorldManager::addCharacter(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	characterManager->addCharacter(world, scene, nbVertices, vertices, nbIndices, indices, transform);
}

void PhysXWorldManager::setCharacterProperty(std::string scene, std::string propName, float value) {
	if (propName.compare("PACE") == 0) {
		characterManager->setPace(scene, value);
	}
	else if (propName.compare("HIT_MAGNITUDE") == 0) {
		characterManager->setHitMagnitude(scene, value);
	}
	else if (propName.compare("HEIGHT") == 0) {
		characterManager->setHeight(scene, value);
	}
	else if (propName.compare("RADIUS") == 0) {
		characterManager->setRadius(scene, value);
	}
	else if (propName.compare("STEP_OFFSET") == 0) {
		characterManager->setStepOffset(scene, value);
	}
	else if (propName.compare("MASS") == 0) {
		characterManager->setMass(scene, value);
	}
	else if (propName.compare("FRICTION") == 0) {
		characterManager->setFriction(scene, value);
	}
	else if (propName.compare("RESTITUTION") == 0) {
		characterManager->setRestitution(scene, value);
	}
}

void PhysXWorldManager::setCharacterProperty(std::string scene, std::string propName, float * value) {
	if (propName.compare("DIRECTION") == 0) {
		characterManager->setDirection(scene, PxVec3(value[0], value[1], value[2]));
	}
}

void PhysXWorldManager::moveCharacter(std::string scene, float * transform) {
	characterManager->move(scene, 1 / 60.0f, world->getGravity());
}
