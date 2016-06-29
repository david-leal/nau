#ifndef _PHYSXSOFTMANAGER_H
#define _PHYSXSOFTMANAGER_H

#include <map>
#include <vector>
#include "PxPhysicsAPI.h"
#include "physXScene.h"

class PhysXSoftManager {
public:
	PhysXSoftManager();
	~PhysXSoftManager();

	void update();
	void addSoftBody(physx::PxScene * world, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	void move(std::string scene, float * transform);

protected:
	std::map<std::string, PhysXScene> softBodies;
};

#endif