#include "physXParticleManager.h"

using namespace physx;

PhysXParticleManager::PhysXParticleManager() {
}


PhysXParticleManager::~PhysXParticleManager() {
}

void PhysXParticleManager::update() {
	for (auto scene : particleSystems) {
		PxParticleFluidReadData* rd = scene.second.particleSystem->lockParticleFluidReadData();

		if (rd) {
			PxU32 nPart = rd->nbValidParticles;
			PxVec4* newPositions = new PxVec4[nPart];

			PxStrideIterator<const PxParticleFlags> flagsIt(rd->flagsBuffer);
			PxStrideIterator<const PxVec3> positionIt(rd->positionBuffer);

			int index = 0;
			for (unsigned i = 0; i < rd->validParticleRange; ++i, ++flagsIt, ++positionIt) {
				if (*flagsIt & PxParticleFlag::eVALID) {
					const PxVec3& position = *positionIt;
					newPositions[index++] = PxVec4(position, 1.0f);
				}
			}
			/*nau::scene::IScene* m_IScene = static_cast<nau::scene::IScene*>(particleSystem->userData);
			particlePass->setPropui(Pass::INSTANCE_COUNT, nPart);
			particlePositionBuffer->setData(nPart * sizeof(PxVec4), newPositions);
			m_IScene->getSceneObject(0)->getRenderable()->getVertexData()->resetCompilationFlag();
			m_IScene->getSceneObject(0)->getRenderable()->getVertexData()->compile();*/

			rd->unlock();
			int np = nPart;
			if ((np < scene.second.maxParticles) && ((scene.second.iterStep++ % scene.second.maxIterStep) == 0)) {
				float waterPos[] = { 0.0f,3.0f,0.0f };
				createParticles(scene.first, 100, 0.1f, waterPos);
			}
		}
	}
}

void PhysXParticleManager::addParticleSystem(physx::PxScene * world, int max, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	particleSystems[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
	particleSystems[scene].maxParticles = max;
	particleSystems[scene].maxIterStep = 10000;
	particleSystems[scene].iterStep = 1;

	PxPhysics *gPhysics = &(world->getPhysics());

	float particleDistance = 0.01f;

	particleSystems[scene].particleSystem = gPhysics->createParticleFluid(max);

	//particleSystems[scene].particleSystem->setGridSize(5.0f);
	particleSystems[scene].particleSystem->setMaxMotionDistance(0.3f);
	//particleSystem->setRestOffset(particleDistance*0.3f);
	particleSystems[scene].particleSystem->setRestOffset(0.04f);
	//particleSystem->setContactOffset(particleDistance*0.3f * 2);
	particleSystems[scene].particleSystem->setContactOffset(0.036f);
	particleSystems[scene].particleSystem->setDamping(0.0f);
	particleSystems[scene].particleSystem->setRestitution(0.3f);
	particleSystems[scene].particleSystem->setDynamicFriction(0.001f);
	particleSystems[scene].particleSystem->setRestParticleDistance(particleDistance);
	particleSystems[scene].particleSystem->setViscosity(60.0f);
	particleSystems[scene].particleSystem->setStiffness(45.0f);
	//particleSystems[scene].particleSystem->setParticleReadDataFlag(PxParticleReadDataFlag::eVELOCITY_BUFFER, true);

	particleSystems[scene].particleSystem->userData = static_cast<void*> (new std::string(scene));

	if (particleSystems[scene].particleSystem)
		world->addActor(*particleSystems[scene].particleSystem);

	particleSystems[scene].particleIndexPool = PxParticleExt::createIndexPool(max);
	//createParticles();
}

float getRandomNumber(float factor) {
	return ((std::rand() % int((factor*2) *100.f)) / 100.0f) - factor;
}

void PhysXParticleManager::createParticles(std::string scene, int n, float randomFactor, float * position) {
	PxU32 existingParticles = particleSystems[scene].numParticles;
	particleSystems[scene].numParticles += n;
	std::vector<PxU32> mTmpIndexArray;
	mTmpIndexArray.resize(particleSystems[scene].numParticles);
	PxStrideIterator<PxU32> indexData(&mTmpIndexArray[0]);
	
	particleSystems[scene].numParticles = particleSystems[scene].particleIndexPool->allocateIndices((particleSystems[scene].numParticles- existingParticles), indexData);
	PxVec3 partPosistions[100];
	for (int i = 0; i < 100; i++) {
		partPosistions[i] = PxVec3(position[0] + getRandomNumber(randomFactor), position[1], position[2] + getRandomNumber(randomFactor));
	}

	//PxVec3 appParticleVelocities[] = { PxVec3(0.2f, 0.0f, 0.0f)	};

	PxParticleCreationData particleCreationData;
	particleCreationData.numParticles = particleSystems[scene].numParticles;
	particleCreationData.indexBuffer = PxStrideIterator<const PxU32>(&mTmpIndexArray[0]);
	particleCreationData.positionBuffer = PxStrideIterator<const PxVec3>(&partPosistions[0]);
	//particleCreationData.velocityBuffer = PxStrideIterator<const PxVec3>(&partVelocities[0]);
	bool ok = particleSystems[scene].particleSystem->createParticles(particleCreationData);

	//PxVec3 appParticleForces[] = { PxVec3(0.2f, 0.0f, 0.0f) };
	//// specify strided iterator to provide update forces
	//PxStrideIterator<const PxVec3> forceBuffer(&appParticleForces[0]);

	//// specify strided iterator to provide indices of particles that need to be updated
	//PxStrideIterator<const PxU32> indexBuffer(&mTmpIndexArray[0]);

	//// specify force update on PxParticleSystem ps choosing the "force" unit
	//particleSystem->addForces(10, indexBuffer, forceBuffer, PxForceMode::eFORCE);
}
