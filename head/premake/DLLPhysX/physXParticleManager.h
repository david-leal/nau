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
	void addParticleSystem(physx::PxScene * world, int max, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	void createParticles(std::string scene, int n, float randomFactor, float * position);

protected:
	typedef struct {
		externalInfo extInfo;
		physx::PxParticleFluid * particleSystem;
		physx::PxParticleExt::IndexPool* particleIndexPool;
		int maxParticles;
		int numParticles;
		//bool perParticleRestOffset = false;
		int maxIterStep;
		int iterStep;
	} PhysXParticleSystem;


	std::map<std::string, PhysXParticleSystem> particleSystems;

};

#endif