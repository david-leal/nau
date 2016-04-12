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
PxCloth* cloth;
PxParticleSystem* particleSystem;
std::vector<PxVec3> particlePositions;
PxParticleExt::IndexPool* particleIndexPool;

PhsXWorld::PhsXWorld(void) : m_pScene(0), m_pDynamicsWorld(0) {
}


PhsXWorld::~PhsXWorld(void) {
	//delete m_pDynamicsWorld;
	if(gConnection)
		gConnection->release();
}

mat4
getMatFromPhysXTransform(PxTransform transform) {
	PxMat44 mat = PxMat44(transform);
	float m[16];
	for (int i = 0; i < 16; ++i) m[i] = *(mat.front() + i);
	return mat4(m);
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
				m_IScene->setTransform(getMatFromPhysXTransform(activeTransforms[i].actor2World));
			}
		}
	
		if (cloth) {
			nau::scene::IScene *m_IScene = static_cast<nau::scene::IScene*>(cloth->userData);
			//m_IScene->setTransform(getMatFromPhysXTransform(cloth->getGlobalPose()));

			std::shared_ptr<VertexData> &vd = m_IScene->getSceneObject(0)->getRenderable()->getVertexData();

			int count = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());

			PxClothParticleData* pData = cloth->lockParticleData();
			PxClothParticle* pParticles = pData->particles;

			std::shared_ptr<std::vector<VertexData::Attr>> points = vd->getDataOf(VertexData::GetAttribIndex(std::string("position")));
			for (int i = 0; i < count; i++) {
				points->at(i).set(pParticles[i].pos.x, pParticles[i].pos.y, pParticles[i].pos.z);
			}
			vd->resetCompilationFlag();
			vd->compile();
			pData->unlock();
		}

		if (controller) {
			controller->move(PxVec3(0.3f, -9.81f, 0.0f), 0.2f, 1 / 60.0f, NULL);
			nau::scene::IScene *m_ISceneCharacter = static_cast<nau::scene::IScene*>(controller->getUserData());
			mat4 mat = getMatFromPhysXTransform(controller->getActor()->getGlobalPose());

			//For Pusher character
			mat.rotate(-90, vec3(0, 0, 1));
			//mat.rotate(-90, vec3(0, 1, 0));

			m_ISceneCharacter->setTransform(mat);
		}

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
PhsXWorld::_addRigid(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {

	PxPhysics *gPhysics = &(m_pDynamicsWorld->getPhysics());
	
	if (mass == 0.0f) {
		PxRigidStatic* staticActor;
		if (name.compare("plane") == 0) {
			staticActor = PxCreatePlane(*gPhysics,
				PxPlane(0.0f, 1.0f, 0.0f, 0.0f),
				*(gPhysics->createMaterial(0.5f, 0.5f, 0.6f))
				);

		}
		else {
			staticActor = gPhysics->createRigidStatic(PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix()))));
			PxTriangleMeshGeometry triGeom;
			triGeom.triangleMesh = gPhysics->createTriangleMesh(getTriangleMeshGeo(m_pDynamicsWorld, aScene));
			staticActor->createShape(triGeom, *(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)));
		}
		staticActor->userData = aScene.get();
		m_pDynamicsWorld->addActor(*staticActor);
	} 
	else {
		if (name.compare("man") == 0) {
			PxCapsuleControllerDesc desc;
			desc.height = 1.3f;
			desc.radius = 0.35f;
			PxVec3 pos = PxMat44(const_cast<float*> (aScene->getTransform().getMatrix())).getPosition();
			desc.position = PxExtendedVec3(pos.x, pos.y, pos.z);
			desc.material = gPhysics->createMaterial(0.5f, 0.5f, 0.6f);
			desc.userData = aScene.get();
			desc.reportCallback = this;
			desc.climbingMode = PxCapsuleClimbingMode::eCONSTRAINED;
			desc.stepOffset = 0.25f;
			desc.upDirection = PxVec3(0, 1, 0);
			//desc.slopeLimit = cosf(DegToRad(80.0f));
			controller = manager->createController(desc);
		} 
		else {
			PxRigidDynamic* dynamic;
			//if (name.compare("ball") == 0) {
			//	dynamic = PxCreateDynamic(*gPhysics,
			//		PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix()))),
			//		PxSphereGeometry(1),
			//		*(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)),
			//		10.0f
			//	);
			//	//dynamic->setLinearVelocity(PxVec3(0, -50, -100));
			//}
			//else {
				PxTransform trans = PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix())));
				dynamic = gPhysics->createRigidDynamic(trans);
				PxConvexMesh * convexMesh = gPhysics->createConvexMesh(getTriangleMeshGeo(m_pDynamicsWorld, aScene, false));
				PxConvexMeshGeometry convGeom(convexMesh);
				//PxConvexMeshGeometry convGeom(convexMesh, PxMeshScale(0.5f));
				//convGeom.convexMesh = gPhysics->createConvexMesh(getTriangleMeshGeo(m_pDynamicsWorld, aScene, false));

				//PxShape *shape = dynamic->createShape(convGeom, *(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)), PxShapeFlag::eSIMULATION_SHAPE | PxShapeFlag::eVISUALIZATION | PxShapeFlag::eSCENE_QUERY_SHAPE);
				PxShape *shape = dynamic->createShape(convGeom, *(gPhysics->createMaterial(0.5f, 0.5f, 0.6f)));
				//shape->setFlag(PxShapeFlag::eSIMULATION_SHAPE, true);
				//dynamic->setRigidBodyFlag(PxRigidBodyFlag::eKINEMATIC, false);
			//}
			dynamic->userData = aScene.get();
			//dynamic->setAngularDamping(0.5f);
			//dynamic->setLinearVelocity(velocity);

			m_pDynamicsWorld->addActor(*dynamic);
		}
	}

	//m_RigidBodies[name] = body;
	//m_pDynamicsWorld->addActor(*body);
}

void
PhsXWorld::_addCloth(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {
	PxPhysics *gPhysics = &(m_pDynamicsWorld->getPhysics());

	std::shared_ptr<nau::scene::SceneObject> &aObject = aScene->getSceneObject(0);
	std::shared_ptr<VertexData> &vd = aObject->getRenderable()->getVertexData();
	std::vector<std::shared_ptr<MaterialGroup>> &matGroups = aObject->getRenderable()->getMaterialGroups();
	std::vector<std::shared_ptr<MaterialGroup>>::iterator matGroupsIter;

	PxDefaultMemoryOutputStream writeBuffer;

	int count = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());
	PxClothParticle *particles = new PxClothParticle[count];

	PxClothMeshDesc meshDesc;
	matGroupsIter = matGroups.begin();
	for (; matGroupsIter != matGroups.end(); matGroupsIter++) {
		if ((*matGroupsIter)->getIndexData()->getIndexSize()) {
			std::shared_ptr<std::vector<unsigned int>> &indexes = (*matGroupsIter)->getIndexData()->getIndexData();
			PxClothParticle *ptls = particles;
			std::shared_ptr<std::vector<VertexData::Attr>> points = vd->getDataOf(VertexData::GetAttribIndex(std::string("position")));
			for (int i = 0; i < count; i++) {
				VertexData::Attr tempPoint = points->at(i);
				//ptls[i] = PxClothParticle(PxVec3(tempPoint.x, tempPoint.y, tempPoint.z), (i==0 || i==1) ? 0.0f : 1.0f);
				ptls[i] = PxClothParticle(PxVec3(tempPoint.x, tempPoint.y, tempPoint.z), i == 0 ? 0.0f : 0.1f);
			}

			meshDesc.points.data = reinterpret_cast<const unsigned char *>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0)));
			meshDesc.points.count = count;
			meshDesc.points.stride = 4 * sizeof(float);

			meshDesc.invMasses.data = &particles->invWeight;
			meshDesc.invMasses.count = count;
			meshDesc.invMasses.stride = sizeof(PxClothParticle);

			meshDesc.triangles.data = reinterpret_cast<const unsigned char *>(&((*indexes)[0]));
			meshDesc.triangles.count = static_cast<int> (indexes->size() / 3);
			meshDesc.triangles.stride = 3 * sizeof(unsigned int);

		}
	}
	//float * points = reinterpret_cast<float *>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0)));
	//PxU32 numParticles = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());
	//PxU32 stride = 4 * sizeof(float);
	//// create particles
	//PxClothParticle* particles = new PxClothParticle[numParticles];
	//PxClothParticle* pIt = particles;
	//for (PxU32 i = 0; i<numParticles; ++i) {
	//	pIt->invWeight = i==0 ? 0.0f : 1.0f;
	//	int tempStride = i*stride;
	//	pIt->pos = PxVec3(points[tempStride], points[tempStride + 1], points[tempStride + 2]);
	//}

	PxClothFabric* fabric = PxClothFabricCreate(*gPhysics, meshDesc, PxVec3(0, -1, 0));
	PxTransform pose = PxTransform(PxMat44(const_cast<float*> (aScene->getTransform().getMatrix())));
	PxClothFlags flags = PxClothFlags();
	/*if(!flags.isSet(PxClothFlag::eSCENE_COLLISION))
		flags.set(PxClothFlag::eSCENE_COLLISION);
	if (!flags.isSet(PxClothFlag::eGPU))
		flags.set(PxClothFlag::eGPU);
	if (!flags.isSet(PxClothFlag::eSWEPT_CONTACT))
		flags.set(PxClothFlag::eSWEPT_CONTACT);*/
	cloth = gPhysics->createCloth(pose, *fabric, particles, flags);
	cloth->userData = aScene.get();
	cloth->setClothFlag(PxClothFlag::eSCENE_COLLISION, true);
	cloth->setClothFlag(PxClothFlag::eGPU, true);
	cloth->setClothFlag(PxClothFlag::eSWEPT_CONTACT, true);
	cloth->setSolverFrequency(300.0f);
	cloth->setInertiaScale(0.9f);

	cloth->setStretchConfig(PxClothFabricPhaseType::eVERTICAL, PxClothStretchConfig(0.2f));
	cloth->setStretchConfig(PxClothFabricPhaseType::eHORIZONTAL, PxClothStretchConfig(0.2f));
	cloth->setStretchConfig(PxClothFabricPhaseType::eSHEARING, PxClothStretchConfig(0.75f));
	cloth->setStretchConfig(PxClothFabricPhaseType::eBENDING, PxClothStretchConfig(0.2f));
	m_pDynamicsWorld->addActor(*cloth);

}

void
PhsXWorld::_addParticles(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {
	PxPhysics *gPhysics = &(m_pDynamicsWorld->getPhysics());

	// set immutable properties.
	PxU32 maxParticles = 100;
	PxU32 numParticles = 1;
	bool perParticleRestOffset = false;

	// create particle system in PhysX SDK
	particleSystem = gPhysics->createParticleSystem(maxParticles, perParticleRestOffset);

	// add particle system to scene, in case creation was successful
	if (particleSystem)
		m_pDynamicsWorld->addActor(*particleSystem);




	std::vector<PxU32> mTmpIndexArray;
	mTmpIndexArray.resize(numParticles);
	PxStrideIterator<PxU32> indexData(&mTmpIndexArray[0]);
	// allocateIndices() may clamp the number of inserted particles
	numParticles = particleIndexPool->allocateIndices(numParticles, indexData);
	particlePositions = std::vector<PxVec3>();
	particlePositions.push_back(PxVec3(0.0f, 5.0f, 0.0f));
	
	PxParticleCreationData particleCreationData;
	particleCreationData.numParticles = numParticles;
	particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(&mTmpIndexArray[0]);
	particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(&particlePositions[0]);
	//particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(&particles.velocities[0]);
	bool ok = particleSystem->createParticles(particleCreationData);



	// declare particle descriptor for creating new particles
	// based on numNewAppParticles count and newAppParticleIndices, newAppParticlePositions arrays and newAppParticleVelocity
	//PxParticleCreationData particleCreationData;
	//particleCreationData.numParticles = 1;
	//particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(newAppParticleIndices);
	//particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(newAppParticlePositions);
	//particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(&newAppParticleVelocity, 0);

	// create particles in *PxParticleSystem* ps
	//bool success = particleSystem->createParticles(particleCreationData);


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
