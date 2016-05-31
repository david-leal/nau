#include "PhysXRigidManager.h"

using namespace physx;

PhysXRigidManager::PhysXRigidManager() {
}


PhysXRigidManager::~PhysXRigidManager() {
}

void PhysXRigidManager::update(const physx::PxActiveTransform * activeTransforms, physx::PxU32 nbActiveTransforms) {
	for (PxU32 i = 0; i < nbActiveTransforms; ++i) {
		if (activeTransforms[i].userData != NULL) {
			std::string *n = static_cast<std::string*>(activeTransforms[i].userData);
			if (rigidBodies.find(*n) != rigidBodies.end()) {
				getMatFromPhysXTransform(activeTransforms[i].actor2World, rigidBodies[*n].extInfo.transform);
			}
		}
	}
}

physx::PxInputStream *
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

void 
PhysXRigidManager::addStaticBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);

	PxPhysics *gPhysics = &(world->getPhysics());
	PxRigidStatic* staticActor;
	if (scene.compare("plane") == 0) {
		staticActor = PxCreatePlane(*gPhysics,
			PxPlane(0.0f, 1.0f, 0.0f, 0.0f),
			*(gPhysics->createMaterial(1.0f, 1.0f, 1.0f))
		);

	}
	else {
		staticActor = gPhysics->createRigidStatic(PxTransform(PxMat44(rigidBodies[scene].extInfo.transform)));
		PxTriangleMeshGeometry triGeom;
		triGeom.triangleMesh = gPhysics->createTriangleMesh(*getTriangleMeshGeo(world, mCooking, rigidBodies[scene].extInfo));
		staticActor->createShape(triGeom, *(gPhysics->createMaterial(1.0f, 1.0f, 1.0f)));
	}
	staticActor->userData = static_cast<void*> (new std::string(scene));
	world->addActor(*staticActor);
	
	rigidBodies[scene].actor = staticActor;

}

void PhysXRigidManager::addDynamicBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
	PxPhysics *gPhysics = &(world->getPhysics());
	PxRigidDynamic* dynamic;
	PxTransform trans = PxTransform(PxMat44(rigidBodies[scene].extInfo.transform));
	dynamic = gPhysics->createRigidDynamic(trans);

	PxConvexMesh * convexMesh = gPhysics->createConvexMesh(*getTriangleMeshGeo(world, mCooking, rigidBodies[scene].extInfo, false));
	PxShape *shape = dynamic->createShape(PxConvexMeshGeometry(convexMesh), *(gPhysics->createMaterial(1.0f, 1.0f, 1.0f)));
	dynamic->userData = static_cast<void*> (new std::string(scene));

	world->addActor(*dynamic);
	rigidBodies[scene].actor = dynamic;
}

void PhysXRigidManager::setMass(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		PxRigidDynamic * dyn = rigidBodies[name].actor->is<PxRigidDynamic>();
		if (dyn) {
			dyn->setMass(value);
		}
	}
}

void PhysXRigidManager::setFriction(std::string name, float value) {
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
					mat->setStaticFriction(value);
				}
			}
		}
	}
}

void PhysXRigidManager::setRestitution(std::string name, float value) {
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

void PhysXRigidManager::move(std::string scene, float * transform) {
	if (rigidBodies.find(scene) != rigidBodies.end()) {
		PxRigidActor * actor = rigidBodies[scene].actor->is<PxRigidActor>();
		if (actor) {
			rigidBodies[scene].extInfo.transform = transform;
			actor->setGlobalPose(PxTransform(PxMat44(transform)));
		}
	}
}
