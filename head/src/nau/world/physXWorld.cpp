#include "nau/world/physXWorld.h"

#include "nau/geometry/iBoundingVolume.h"
#include "nau/geometry/vertexData.h"
#include "nau/material/materialGroup.h"

using namespace nau::world;
using namespace nau::geometry;
using namespace nau::scene;
using namespace nau::render;
using namespace nau::material;
using namespace physx;

PxDefaultAllocator		gAllocator;
PxDefaultErrorCallback	gErrorCallback;

PhsXWorld::PhsXWorld(void) : m_pScene(0), m_pDynamicsWorld(0) {
}


PhsXWorld::~PhsXWorld(void) {
	//delete m_pDynamicsWorld;
}


void
PhsXWorld::update(void) {
	if (0 != m_pDynamicsWorld) {
		
		m_pDynamicsWorld->simulate(1 / 60.0f);
		m_pDynamicsWorld->fetchResults(true);

		PxU32 nbActiveTransforms;
		const PxActiveTransform* activeTransforms = m_pDynamicsWorld->getActiveTransforms(nbActiveTransforms);
		for (PxU32 i = 0; i < nbActiveTransforms; ++i) {
			if (activeTransforms[i].userData != NULL) {
				nau::scene::IScene *m_IScene = static_cast<nau::scene::IScene*>(activeTransforms[i].userData);
				PxMat44 mat = PxMat44(activeTransforms[i].actor2World);
				float m[16];
				for (int i = 0; i < 16; ++i) m[i] = *(mat.front() + i);
				m_IScene->setTransform(mat4(m));
			}
		}
		
	}
}


void
PhsXWorld::build(void) {
	PxFoundation* gFoundation = PxCreateFoundation(PX_PHYSICS_VERSION, gAllocator, gErrorCallback);
	PxProfileZoneManager* profileZoneManager = &PxProfileZoneManager::createProfileZoneManager(gFoundation);
	PxPhysics*	gPhysics = PxCreatePhysics(PX_PHYSICS_VERSION, *gFoundation, PxTolerancesScale(), true, profileZoneManager);
	PxVisualDebuggerConnection* gConnection = NULL;
	if (gPhysics->getPvdConnectionManager()) {
		gPhysics->getVisualDebugger()->setVisualizeConstraints(true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_CONTACTS, true);
		gPhysics->getVisualDebugger()->setVisualDebuggerFlag(PxVisualDebuggerFlag::eTRANSMIT_SCENEQUERIES, true);
		//gConnection = PxVisualDebuggerExt::createConnection(gPhysics->getPvdConnectionManager(), PVD_HOST, 5425, 10);
	}
	PxSceneDesc sceneDesc(gPhysics->getTolerancesScale());
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	PxDefaultCpuDispatcher* gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.flags = PxSceneFlag::eENABLE_ACTIVETRANSFORMS;
	m_pDynamicsWorld = gPhysics->createScene(sceneDesc);
	//m_pDynamicsWorld->setFlag(PxSceneFlag::eENABLE_ACTIVETRANSFORMS, true);

}


void
PhsXWorld::setScene(nau::scene::IScene *aScene) {
	m_pScene = aScene;
}

void
PhsXWorld::_add(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {
	//PxActor* body;
	PxPhysics *gPhysics = &(m_pDynamicsWorld->getPhysics());
	if (name.compare("plane") == 0) {
		PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics,
			PxPlane(0, 1, 0, 0),
			*(gPhysics->createMaterial(0.5f, 0.5f, 0.6f))
			);
		m_pDynamicsWorld->addActor(*groundPlane);
	}
	else {
		PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics,
			PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix()))),
			PxSphereGeometry(1),
			*(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)),
			10.0f);
		//dynamic->setLinearVelocity(PxVec3(0, -50, -100));
		dynamic->userData = aScene.get();
		//dynamic->setAngularDamping(0.5f);
		//dynamic->setLinearVelocity(velocity);
		m_pDynamicsWorld->addActor(*dynamic);
	}
	//m_RigidBodies[name] = body;
	//m_pDynamicsWorld->addActor(*body);
}

void
PhsXWorld::setKinematic(std::string name) {
	//m_RigidBodies[name]->setCollisionFlags(m_RigidBodies[name]->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	//m_RigidBodies[name]->setActivationState(DISABLE_DEACTIVATION);
}

void
PhsXWorld::setDynamic(std::string name) {
	//m_RigidBodies[name]->setCollisionFlags(m_RigidBodies[name]->getCollisionFlags() ^ btCollisionObject::CF_KINEMATIC_OBJECT);
	//m_RigidBodies[name]->activate();
}

void
PhsXWorld::setVelocity(std::string name, nau::math::vec3 vel) {
	/*if (m_RigidBodies.count(name)) {
		if (m_RigidBodies[name]->getVelocityInLocalPoint(m_RigidBodies[name]->getCenterOfMassPosition()).length() < 10.0f) {
			m_RigidBodies[name]->applyCentralImpulse(btVector3(3 * vel.x, 3 * vel.y, 3 * vel.z));
		}
	}*/
}

void
PhsXWorld::disableObject(std::string name) {
	//m_pDynamicsWorld->removeCollisionObject(m_RigidBodies[name]);
}

void
PhsXWorld::enableObject(std::string name) {
	//m_pDynamicsWorld->addCollisionObject(m_RigidBodies[name]);
}
