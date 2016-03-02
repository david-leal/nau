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
PxVisualDebuggerConnection* gConnection = NULL;
PxCooking* mCooking;
PxControllerManager* manager;
PxController* controller;

PhsXWorld::PhsXWorld(void) : m_pScene(0), m_pDynamicsWorld(0) {
}


PhsXWorld::~PhsXWorld(void) {
	//delete m_pDynamicsWorld;
	if(gConnection)
		gConnection->release();
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
	
		controller->move(PxVec3(0, -9.81f, -0.5f), 0.2f, 1 / 60.0f, NULL);
	}
}


void
PhsXWorld::build(void) {
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
	sceneDesc.gravity = PxVec3(0.0f, -9.81f, 0.0f);
	PxDefaultCpuDispatcher* gDispatcher = PxDefaultCpuDispatcherCreate(2);
	sceneDesc.cpuDispatcher = gDispatcher;
	sceneDesc.filterShader = PxDefaultSimulationFilterShader;
	sceneDesc.flags |= PxSceneFlag::eENABLE_ACTIVETRANSFORMS;
	m_pDynamicsWorld = gPhysics->createScene(sceneDesc);
	//m_pDynamicsWorld->setFlag(PxSceneFlag::eENABLE_ACTIVETRANSFORMS, true);

	mCooking = PxCreateCooking(PX_PHYSICS_VERSION, *gFoundation, PxCookingParams(scale));
	manager = PxCreateControllerManager(*m_pDynamicsWorld);

}


void
PhsXWorld::setScene(nau::scene::IScene *aScene) {
	m_pScene = aScene;
}

//PxTriangleMesh*
PxDefaultMemoryInputData
getTriangleMeshGeo(PxScene *m_pDynamicsWorld, std::shared_ptr<nau::scene::IScene> &aScene, bool isStatic=true) {
	PxPhysics *gPhysics = &(m_pDynamicsWorld->getPhysics());

	std::shared_ptr<nau::scene::SceneObject> &aObject = aScene->getSceneObject(0);
	std::shared_ptr<VertexData> &vd = aObject->getRenderable()->getVertexData();
	std::vector<std::shared_ptr<MaterialGroup>> &matGroups = aObject->getRenderable()->getMaterialGroups();
	std::vector<std::shared_ptr<MaterialGroup>>::iterator matGroupsIter;
	
	PxDefaultMemoryOutputStream writeBuffer;
	
	PxTriangleMeshDesc meshDesc;
	matGroupsIter = matGroups.begin();
	for (; matGroupsIter != matGroups.end(); matGroupsIter++) {
		if ((*matGroupsIter)->getIndexData()->getIndexSize()) {
			std::shared_ptr<std::vector<unsigned int>> &indexes = (*matGroupsIter)->getIndexData()->getIndexData();

			meshDesc.points.count = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());
			meshDesc.points.stride = 4 * sizeof(float);
			meshDesc.points.data = reinterpret_cast<const unsigned char *>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0)));

			meshDesc.triangles.count = static_cast<int> (indexes->size() / 3);
			meshDesc.triangles.stride = 3 * sizeof(unsigned int);
			meshDesc.triangles.data = reinterpret_cast<const unsigned char *>(&((*indexes)[0]));

		}
	}
	bool status;
	if (isStatic) {
		//PxToolkit::MemoryOutputStream writeBuffer;
		status = mCooking->cookTriangleMesh(meshDesc, writeBuffer);
		//if (!status)
		//	return NULL;
	}
	else {
		PxConvexMeshDesc meshDesc2;
		
		/*PxU32 nbVerts = 0;
		PxVec3 * vertices;
		PxU32 nbIndices = 0 ;
		PxU32 *	indices;
		PxU32 nbPolygons = 0;
		PxHullPolygon * hullPolygons;
		
		if (mCooking->computeHullPolygons(meshDesc, gAllocator, nbVerts, vertices, nbIndices, indices, nbPolygons, hullPolygons)) {
			meshDesc2.points.count = nbVerts;
			meshDesc2.points.data = vertices;
			meshDesc2.points.stride = sizeof(PxVec3);
			meshDesc2.indices.count = nbIndices;
			meshDesc2.indices.data = indices;
			meshDesc2.indices.stride = sizeof(PxU32);
			meshDesc2.polygons.count = nbPolygons;
			meshDesc2.polygons.data = hullPolygons;
			meshDesc2.polygons.stride = sizeof(PxHullPolygon);
			meshDesc2.flags = PxConvexFlag::eINFLATE_CONVEX;
		}
		else {*/
			meshDesc2.points.count = meshDesc.points.count;
			meshDesc2.points.stride = meshDesc.points.stride;
			meshDesc2.points.data = meshDesc.points.data;
			meshDesc2.flags |= PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX;
			meshDesc2.vertexLimit = 256;
		//}
		/*PxConvexMeshDesc meshDesc;
		matGroupsIter = matGroups.begin();
		for (; matGroupsIter != matGroups.end(); matGroupsIter++) {
			if ((*matGroupsIter)->getIndexData()->getIndexSize()) {
				std::shared_ptr<std::vector<unsigned int>> &indexes = (*matGroupsIter)->getIndexData()->getIndexData();

				meshDesc.points.count = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());
				meshDesc.points.stride = 4 * sizeof(float);
				meshDesc.points.data = reinterpret_cast<const unsigned char *>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0)));
				meshDesc.flags = PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX;
				meshDesc.vertexLimit = 256;
			}
		}*/
		status = mCooking->cookConvexMesh(meshDesc2, writeBuffer);

	}

	//PxToolkit::PxInputStream readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	PxDefaultMemoryInputData readBuffer(writeBuffer.getData(), writeBuffer.getSize());
	return readBuffer;
	//return gPhysics->createTriangleMesh(readBuffer);
}



void
PhsXWorld::_add(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {

	PxPhysics *gPhysics = &(m_pDynamicsWorld->getPhysics());
	if (name.compare("plane") == 0) {
		PxRigidStatic* groundPlane = PxCreatePlane(*gPhysics,
			PxPlane(0, 1, 0, 0),
			*(gPhysics->createMaterial(0.5f, 0.5f, 0.6f))
			);

		/*PxRigidStatic* groundPlane = gPhysics->createRigidStatic(PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix()))));
		PxTriangleMeshGeometry triGeom;
		triGeom.triangleMesh = gPhysics->createTriangleMesh(getTriangleMeshGeo(m_pDynamicsWorld, aScene));
		groundPlane->createShape(triGeom, *(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)));*/

		m_pDynamicsWorld->addActor(*groundPlane);
	}
	if (name.compare("ball") == 0) {
		/*PxRigidDynamic* dynamic = PxCreateDynamic(*gPhysics,
			PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix()))),
			PxSphereGeometry(1),
			*(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)),
			10.0f);*/
		//dynamic->setLinearVelocity(PxVec3(0, -50, -100));

		PxTransform trans = PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix())));
		PxRigidDynamic* dynamic = gPhysics->createRigidDynamic(trans);
		PxConvexMesh * convexMesh = gPhysics->createConvexMesh(getTriangleMeshGeo(m_pDynamicsWorld, aScene, false));
		PxConvexMeshGeometry convGeom(convexMesh);
		//PxConvexMeshGeometry convGeom(convexMesh, PxMeshScale(0.5f));
		//convGeom.convexMesh = gPhysics->createConvexMesh(getTriangleMeshGeo(m_pDynamicsWorld, aScene, false));
		//PxShape *shape = dynamic->createShape(convGeom, *(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)), PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE);
		PxShape *shape = dynamic->createShape(convGeom, *(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)));
		//shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
		//dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
		
		
		dynamic->userData = aScene.get();
		//dynamic->setAngularDamping(0.5f);
		//dynamic->setLinearVelocity(velocity);
		m_pDynamicsWorld->addActor(*dynamic);
	}
	if (name.compare("man") == 0) {
		PxCapsuleControllerDesc desc;
		desc.height = 1;
		desc.radius = 0.5;
		PxVec3 pos = PxMat44(const_cast<float*> (aScene->getTransform().getMatrix())).getPosition();
		desc.position = PxExtendedVec3( pos.x, pos.y, pos.z);
		desc.material = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
		desc.userData = aScene.get();
		desc.reportCallback = this;
		desc.climbingMode = PxCapsuleClimbingMode::eCONSTRAINED;
		desc.stepOffset = 0.30f;
		desc.slopeLimit = cosf(DegToRad(45.0f));
		controller = manager->createController(desc);
		
	}




	//m_RigidBodies[name] = body;
	//m_pDynamicsWorld->addActor(*body);
}

void
PhsXWorld::onShapeHit(const PxControllerShapeHit &hit) {
	if(hit.actor->isRigidDynamic())
		hit.actor->isRigidDynamic()->addForce(PxVec3(hit.dir)*10);
}

void 
PhsXWorld::onControllerHit(const PxControllersHit & hit) {
}

void nau::world::PhsXWorld::onObstacleHit(const PxControllerObstacleHit & hit) {
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
