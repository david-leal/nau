#ifndef _PHYSXSOFTMANAGER_H
#define _PHYSXSOFTMANAGER_H

#include <map>
#include <vector>
#include "PxPhysicsAPI.h"
#include "physXScene.h"

#define CLOTH_CONDITION_NONE -1
#define CLOTH_CONDITION_GT 0
#define CLOTH_CONDITION_LT 1
#define CLOTH_CONDITION_EGT 2
#define CLOTH_CONDITION_ELT 3
#define CLOTH_CONDITION_EQ 4

class PhysXSoftManager {
public:
	PhysXSoftManager();
	~PhysXSoftManager();

	void update();
	void createInfo(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform, int condition, float * conditionValue);
	physx::PxClothParticle createClothParticle(std::string scene, physx::PxVec3 vertice);
	void addSoftBody(physx::PxScene * world, const std::string &scene);
	void move(std::string scene, float * transform);

protected:

	typedef struct {
		PhysXScene info;
		int condition;
		physx::PxPlane contidionPlane;
	} SoftScene;


	std::map<std::string, SoftScene> softBodies;
};

#endif