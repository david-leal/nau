#include "PhysXRigidManager.h"

using namespace physx;

PhysXRigidManager::PhysXRigidManager() {
}


PhysXRigidManager::~PhysXRigidManager() {
}

void 
PhysXRigidManager::update(const physx::PxActiveTransform * activeTransforms, physx::PxU32 nbActiveTransforms) {
	for (PxU32 i = 0; i < nbActiveTransforms; ++i) {
		if (activeTransforms[i].userData != NULL) {
			std::string *n = static_cast<std::string*>(activeTransforms[i].userData);
			if (rigidBodies.find(*n) != rigidBodies.end()) {
				getMatFromPhysXTransform(activeTransforms[i].actor2World, rigidBodies[*n].extInfo.transform);
			}
		}
	}
}

void 
PhysXRigidManager::createInfo(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
}

physx::PxMaterial* 
PhysXRigidManager::createMaterial(physx::PxScene * world, float staticFriction, float dynamicFrction, float restitution) {
	PxPhysics *gPhysics = &(world->getPhysics());
	PxMaterial * material = gPhysics->createMaterial(staticFriction, dynamicFrction, restitution);
	return material;
}

void 
PhysXRigidManager::addStaticBody(const std::string & scene, physx::PxScene * world, physx::PxCooking * mCooking, nau::physics::IPhysics::BoundingVolume shape, physx::PxMaterial * material) {
	PxPhysics *gPhysics = &(world->getPhysics());
	PxRigidStatic * staticActor;
	PxTransform trans = PxTransform(PxMat44(rigidBodies[scene].extInfo.transform));
	switch (shape.sceneShape)
	{
	case nau::physics::IPhysics::BOX:
	{
		staticActor = PxCreateStatic(
			world->getPhysics(),
			trans,
			PxBoxGeometry(shape.max[0], shape.max[1], shape.max[2]),
			*material
		);
	}
	break;
	case nau::physics::IPhysics::SPHERE:
	{
		staticActor = PxCreateStatic(
			world->getPhysics(),
			trans,
			PxSphereGeometry(PxVec3(shape.max[0], shape.max[1], shape.max[2]).magnitude()),
			*material
		);
	}
	break;
	case nau::physics::IPhysics::CAPSULE:
	{
		staticActor = PxCreateStatic(
			world->getPhysics(),
			trans,
			PxCapsuleGeometry(
				PxVec3(shape.max[0], shape.max[1], shape.max[2]).magnitude(),
				shape.max[1]
			),
			*material
		);
	}
	break;
	default:
	{
		if (scene.compare("plane") == 0) {
			staticActor = PxCreatePlane(
				world->getPhysics(),
				PxPlane(0.0f, 1.0f, 0.0f, 0.0f),
				*material
			);
		}
		else {
			staticActor = gPhysics->createRigidStatic(trans);
			PxTriangleMeshGeometry triGeom;
			triGeom.triangleMesh = gPhysics->createTriangleMesh(*getTriangleMeshGeo(world, mCooking, rigidBodies[scene].extInfo, true));
			staticActor->createShape(triGeom, *material);
		}
	}
	break;
	}
	staticActor->userData = static_cast<void*> (new std::string(scene));
	world->addActor(*staticActor);
	rigidBodies[scene].actor = staticActor;
}

void
PhysXRigidManager::addDynamicBody(const std::string & scene, physx::PxScene * world, physx::PxCooking * mCooking, nau::physics::IPhysics::BoundingVolume shape, physx::PxMaterial * material) {
	PxPhysics *gPhysics = &(world->getPhysics());
	PxRigidDynamic * dynamic;
	PxTransform trans = PxTransform(PxMat44(rigidBodies[scene].extInfo.transform));
	switch (shape.sceneShape)
	{
	case nau::physics::IPhysics::BOX:
	{
		dynamic = gPhysics->createRigidDynamic(trans);
		dynamic->createShape(PxBoxGeometry(shape.max[0], shape.max[1], shape.max[2]), *material);
	}
	break;
	case nau::physics::IPhysics::SPHERE:
	{
		dynamic = gPhysics->createRigidDynamic(trans);
		dynamic->createShape(PxSphereGeometry(PxVec3(shape.max[0], shape.max[1], shape.max[2]).magnitude()), *material);
	}
	break;
	case nau::physics::IPhysics::CAPSULE:
	{
		dynamic = gPhysics->createRigidDynamic(trans);
		dynamic->createShape(
			PxCapsuleGeometry(
				PxVec3(shape.max[0], shape.max[1], shape.max[2]).magnitude(),
				shape.max[1]
			),
			*material
		);
	}
	break;
	default:
	{
		dynamic = gPhysics->createRigidDynamic(trans);
		PxConvexMesh * convexMesh = gPhysics->createConvexMesh(*getTriangleMeshGeo(world, mCooking, rigidBodies[scene].extInfo, false));
		dynamic->createShape(PxConvexMeshGeometry(convexMesh), *material);
	}
	break;
	}

	dynamic->userData = static_cast<void*> (new std::string(scene));
	world->addActor(*dynamic);
	rigidBodies[scene].actor = dynamic;
}

physx::PxInputStream*
PhysXRigidManager::getTriangleMeshGeo(PxScene *world, physx::PxCooking* mCooking, ExternalInfo externInfo, bool isStatic) {
	PxPhysics *gPhysics = &(world->getPhysics());

	PxDefaultMemoryOutputStream *writeBuffer = new PxDefaultMemoryOutputStream();

	bool status;
	if (isStatic) {
		PxTriangleMeshDesc meshDesc;
		meshDesc.points.count = externInfo.nbVertices;
		meshDesc.points.stride = 4 * sizeof(float);
		meshDesc.points.data = externInfo.vertices;

		meshDesc.triangles.count = static_cast<int> (externInfo.nbIndices / 3);
		meshDesc.triangles.stride = 3 * sizeof(unsigned int);
		meshDesc.triangles.data = externInfo.indices;
		
		status = mCooking->cookTriangleMesh(meshDesc, *writeBuffer);
	}
	else {
		PxConvexMeshDesc meshDesc2;

		meshDesc2.points.count = externInfo.nbVertices;
		meshDesc2.points.stride = 4 * sizeof(float);
		meshDesc2.points.data = externInfo.vertices;
		meshDesc2.flags |= PxConvexFlag::eCOMPUTE_CONVEX | PxConvexFlag::eINFLATE_CONVEX;
		meshDesc2.vertexLimit = 256;

		status = mCooking->cookConvexMesh(meshDesc2, *writeBuffer);

	}
	return new PxDefaultMemoryInputData(writeBuffer->getData(), writeBuffer->getSize());
}

//void 
//PhysXRigidManager::addStaticBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform, float staticFriction, float dynamicFrction, float restitution) {
//	rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
//	//setExtInfo(scene, nbVertices, vertices, nbIndices, indices, transform);
//
//	PxPhysics *gPhysics = &(world->getPhysics());
//	PxMaterial * material = gPhysics->createMaterial(staticFriction, dynamicFrction, restitution);
//
//	PxRigidStatic* staticActor;
//	if (scene.compare("plane") == 0) {
//		staticActor = PxCreatePlane(*gPhysics,
//			PxPlane(0.0f, 1.0f, 0.0f, 0.0f),
//			*material
//		);
//
//	}
//	else {
//		staticActor = gPhysics->createRigidStatic(PxTransform(PxMat44(rigidBodies[scene].extInfo.transform)));
//		PxTriangleMeshGeometry triGeom;
//		triGeom.triangleMesh = gPhysics->createTriangleMesh(*getTriangleMeshGeo(world, mCooking, rigidBodies[scene].extInfo));
//		staticActor->createShape(triGeom, *material);
//	}
//	staticActor->userData = static_cast<void*> (new std::string(scene));
//	world->addActor(*staticActor);
//	
//	rigidBodies[scene].actor = staticActor;
//
//}
//
//void PhysXRigidManager::addDynamicBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform, float staticFriction, float dynamicFrction, float restitution) {
//	rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
//	//setExtInfo(scene, nbVertices, vertices, nbIndices, indices, transform);
//
//	PxPhysics *gPhysics = &(world->getPhysics());
//	PxMaterial * material = gPhysics->createMaterial(staticFriction, dynamicFrction, restitution);
//	PxRigidDynamic* dynamic;
//	PxTransform trans = PxTransform(PxMat44(rigidBodies[scene].extInfo.transform));
//
//	if (scene.find("billiardBall") != std::string::npos) {
//		dynamic = PxCreateDynamic(*gPhysics,
//					trans,
//					PxSphereGeometry(1),
//					*material,
//					1.0f
//				);
//	}
//	else {
//		dynamic = gPhysics->createRigidDynamic(trans);
//		PxConvexMesh * convexMesh = gPhysics->createConvexMesh(*getTriangleMeshGeo(world, mCooking, rigidBodies[scene].extInfo, false));
//		PxShape *shape = dynamic->createShape(PxConvexMeshGeometry(convexMesh), *material);
//	}
//	dynamic->userData = static_cast<void*> (new std::string(scene));
//	world->addActor(*dynamic);
//	rigidBodies[scene].actor = dynamic;
//}

void 
PhysXRigidManager::setMass(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		PxRigidDynamic * dyn = rigidBodies[name].actor->is<PxRigidDynamic>();
		if (dyn) {
			dyn->setMass(value);
		}
	}
}

void 
PhysXRigidManager::setStaticFriction(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		PxRigidActor * actor = rigidBodies[name].actor->is<PxRigidActor>();
		if (actor) {
			PxU32 nbShapes = actor->getNbShapes();
			std::vector<PxShape *> list(nbShapes);
			actor->getShapes(&list.at(0), nbShapes);
			for (PxShape* shape : list) {
				PxU32 nbMat = shape->getNbMaterials();
				std::vector<PxMaterial *> matList(nbMat);
				shape->getMaterials(&matList.at(0), nbMat);
				for (PxMaterial * mat : matList) {
					mat->setStaticFriction(value);
				}
			}
		}
	}
}

void 
PhysXRigidManager::setDynamicFriction(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		PxRigidActor * actor = rigidBodies[name].actor->is<PxRigidActor>();
		if (actor) {
			PxU32 nbShapes = actor->getNbShapes();
			std::vector<PxShape *> list(nbShapes);
			actor->getShapes(&list.at(0), nbShapes);
			for (PxShape* shape : list) {
				PxU32 nbMat = shape->getNbMaterials();
				std::vector<PxMaterial *> matList(nbMat);
				shape->getMaterials(&matList.at(0), nbMat);
				for (PxMaterial * mat : matList) {
					mat->setDynamicFriction(value);
				}
			}
		}
	}
}

void 
PhysXRigidManager::setRestitution(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		PxRigidActor * actor = rigidBodies[name].actor->is<PxRigidActor>();
		if (actor) {
			PxU32 nbShapes = actor->getNbShapes();
			std::vector<PxShape *> list(nbShapes);
			actor->getShapes(&list.at(0), nbShapes);
			for (PxShape* shape : list) {
				PxU32 nbMat = shape->getNbMaterials();
				std::vector<PxMaterial *> matList(nbMat);
				shape->getMaterials(&matList.at(0), nbMat);
				for (PxMaterial * mat : matList) {
					mat->setRestitution(value);
				}
			}
		}
	}
}

void 
PhysXRigidManager::move(std::string scene, float * transform) {
	if (rigidBodies.find(scene) != rigidBodies.end()) {
		PxRigidActor * actor = rigidBodies[scene].actor->is<PxRigidActor>();
		if (actor) {
			rigidBodies[scene].extInfo.transform = transform;
			actor->setGlobalPose(PxTransform(PxMat44(transform)));
		}
	}
}

void 
PhysXRigidManager::setForce(std::string scene, float * force) {
	if (rigidBodies.find(scene) != rigidBodies.end()) {
		PxRigidDynamic * actor = rigidBodies[scene].actor->is<PxRigidDynamic>();
		if (actor) {
			actor->addForce(PxVec3(force[0], force[1], force[2]), PxForceMode::eIMPULSE);
		}
	}
	//dynamic->addForce(PxVec3(500.0, 0, 0), PxForceMode::eIMPULSE);
}

/*void
PhysXRigidManager::setExtInfo(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	rigidBodies[scene].extInfo.nbVertices = nbVertices;
	rigidBodies[scene].extInfo.vertices = vertices;
	rigidBodies[scene].extInfo.nbIndices = nbIndices;
	rigidBodies[scene].extInfo.indices = indices;
	rigidBodies[scene].extInfo.transform = transform;
	//rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
}*/
