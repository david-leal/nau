#include "physXSoftManager.h"

using namespace physx;

PhysXSoftManager::PhysXSoftManager() {
}


PhysXSoftManager::~PhysXSoftManager() {
}

void PhysXSoftManager::update() {
	for (auto scene : softBodies) {
		PxCloth * cloth = scene.second.actor->is<PxCloth>();
		if (cloth) {
			getMatFromPhysXTransform(cloth->getGlobalPose(), scene.second.extInfo.transform);
			PxClothParticleData* pData = cloth->lockParticleData();
			PxClothParticle* pParticles = pData->particles;
			float * points = scene.second.extInfo.vertices;

			for (int i = 0; i < scene.second.extInfo.nbVertices; i++) {
				points[4 * i] = pParticles[i].pos.x;
				points[(4 * i) + 1] = pParticles[i].pos.y;
				points[(4 * i) + 2] = pParticles[i].pos.z;
			}
			pData->unlock();
		}
	}
}

void PhysXSoftManager::addSoftBody(physx::PxScene * world, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	softBodies[scene].extInfo.nbVertices = nbVertices;
	softBodies[scene].extInfo.vertices = vertices;
	softBodies[scene].extInfo.nbIndices = nbIndices;
	softBodies[scene].extInfo.indices = indices;
	softBodies[scene].extInfo.transform = transform;
	//softBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
	PxPhysics *gPhysics = &(world->getPhysics());

	PxDefaultMemoryOutputStream writeBuffer;

	int stride = 4 * sizeof(float);
	PxClothParticle *particles = new PxClothParticle[nbVertices];

	PxClothMeshDesc meshDesc;
	PxClothParticle *ptls = particles;
	for (int i = 0; i < nbVertices; i++) {
		ptls[i] = PxClothParticle(PxVec3(vertices[4*i], vertices[(4*i)+1], vertices[(4*i)+2]), i == 0 ? 0.0f : 0.1f);
	}

	meshDesc.points.data = reinterpret_cast<const unsigned char *>(vertices);
	meshDesc.points.count = nbVertices;
	meshDesc.points.stride = 4 * sizeof(float);

	meshDesc.invMasses.data = &particles->invWeight;
	meshDesc.invMasses.count = nbVertices;
	meshDesc.invMasses.stride = sizeof(PxClothParticle);

	meshDesc.triangles.data = reinterpret_cast<const unsigned char *>(indices);
	meshDesc.triangles.count = nbIndices / 3;
	meshDesc.triangles.stride = 3 * sizeof(unsigned int);

	//TODO: Get gravity from world
	PxClothFabric* fabric = PxClothFabricCreate(*gPhysics, meshDesc, PxVec3(0, -1, 0));
	PxTransform pose = PxTransform(PxMat44(const_cast<float*> (transform)));
	PxClothFlags flags = PxClothFlags();
	/*if(!flags.isSet(PxClothFlag::eSCENE_COLLISION))
	flags.set(PxClothFlag::eSCENE_COLLISION);
	if (!flags.isSet(PxClothFlag::eGPU))
	flags.set(PxClothFlag::eGPU);
	if (!flags.isSet(PxClothFlag::eSWEPT_CONTACT))
	flags.set(PxClothFlag::eSWEPT_CONTACT);*/
	
	//TODO: Set parameters apart from inicialization
	PxCloth* cloth = gPhysics->createCloth(pose, *fabric, particles, flags);
	cloth->userData = static_cast<void*> (new std::string(scene));;
	cloth->setClothFlag(PxClothFlag::eSCENE_COLLISION, true);
	cloth->setClothFlag(PxClothFlag::eGPU, true);
	cloth->setClothFlag(PxClothFlag::eSWEPT_CONTACT, true);
	cloth->setSolverFrequency(300.0f);
	cloth->setInertiaScale(0.9f);

	cloth->setStretchConfig(PxClothFabricPhaseType::eVERTICAL, PxClothStretchConfig(0.2f));
	cloth->setStretchConfig(PxClothFabricPhaseType::eHORIZONTAL, PxClothStretchConfig(0.2f));
	cloth->setStretchConfig(PxClothFabricPhaseType::eSHEARING, PxClothStretchConfig(0.75f));
	cloth->setStretchConfig(PxClothFabricPhaseType::eBENDING, PxClothStretchConfig(0.2f));
	world->addActor(*cloth);
	
	softBodies[scene].actor = cloth;
}

void PhysXSoftManager::setFriction(std::string name, float value) {
	if (softBodies.find(name) != softBodies.end()) {
		PxCloth * cloth = softBodies[name].actor->is<PxCloth>();
		if (cloth) {

		}
	}
}

void PhysXSoftManager::setRestitution(std::string name, float value) {
	if (softBodies.find(name) != softBodies.end()) {
		PxCloth * cloth = softBodies[name].actor->is<PxCloth>();
		if (cloth) {

		}
	}
}

void PhysXSoftManager::move(std::string scene, float * transform) {
	if (softBodies.find(scene) != softBodies.end()) {
		PxCloth * cloth = softBodies[scene].actor->is<PxCloth>();
		if (cloth) {
			softBodies[scene].extInfo.transform = transform;
			cloth->setGlobalPose(PxTransform(PxMat44(const_cast<float*> (transform))));
		}
	}
}