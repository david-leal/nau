#ifndef _PHYSXPARTICLEMANAGER_H
#define _PHYSXPARTICLEMANAGER_H

#include <map>
#include <vector>
#include <stdlib.h>
#include <time.h> 
#include "PxPhysicsAPI.h"
#include "physXScene.h"

class PhysXParticleManager
{
public:
	PhysXParticleManager();
	~PhysXParticleManager();

	void update();
	void addParticleSystem(physx::PxScene * world, const std::string &scene, float * maxParticles, float * nbParticles, float *transform);
	void createParticles(std::string scene, float n, float randomFactor);
	float * getPositions(std::string scene);

protected:
	typedef struct {
		externalParticles extInfo;
		physx::PxParticleFluid * particleSystem;
		physx::PxParticleExt::IndexPool* particleIndexPool;
		std::vector<float> * positions;
		//bool perParticleRestOffset = false;
		int maxIterStep;
		int iterStep;

	} PhysXParticleSystem;

	std::map<std::string, PhysXParticleSystem> particleSystems;

};

#endif