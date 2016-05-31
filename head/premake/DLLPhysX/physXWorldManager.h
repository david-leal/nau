#ifndef _PHYSXWORLDMANAGER_H
#define _PHYSXWORLDMANAGER_H

#include "PxPhysicsAPI.h"
#include "nau/physics/iPhysics.h"
#include "physXRigidManager.h"
#include "physXSoftManager.h"

class PhysXWorldManager {

private:
	physx::PxScene *world;
	physx::PxDefaultAllocator gAllocator;
	physx::PxDefaultErrorCallback gErrorCallback;
	physx::PxVisualDebuggerConnection* gConnection = NULL;
	physx::PxCooking* mCooking;

	PhysXRigidManager * rigidManager;
	PhysXSoftManager * softManager;

public:
	PhysXWorldManager();
	~PhysXWorldManager();

	void update();
	void setGravity(float x, float y, float z);
	void addRigid(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform, bool isStatic=false);
	void setRigidProperty(std::string scene, std::string propName, float value);
	void moveRigid(std::string scene, float * transform);

	void addCloth(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform);
	void setSoftProperty(std::string scene, std::string propName, float value);
	void moveSoft(std::string scene, float * transform);
	
	//void addCharacter();

	physx::PxScene * getWorld() { return world; }
	
};

#endif