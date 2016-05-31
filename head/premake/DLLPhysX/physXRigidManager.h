#ifndef _PHYSXRIGIDMANAGER_H
#define _PHYSXRIGIDMANAGER_H

#include <map>
#include <vector>
#include "PxPhysicsAPI.h"
#include "physXScene.h"

class PhysXRigidManager {

public:
	PhysXRigidManager();
	~PhysXRigidManager();

	void update(const physx::PxActiveTransform* activeTransforms, physx::PxU32 nbActiveTransforms);
	void addStaticBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	void addDynamicBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	void setMass(std::string name, float value);
	void setFriction(std::string name, float value);
	void setRestitution(std::string name, float value);
	void move(std::string scene, float * transform);

protected:
	std::map<std::string, PhysXScene> rigidBodies;
	physx::PxInputStream * getTriangleMeshGeo(physx::PxScene *world, physx::PxCooking* mCooking, ExternalInfo externInfo, bool isStatic = true);
};

#endif
