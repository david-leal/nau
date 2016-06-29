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
	void addStaticBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform, float staticFriction, float dynamicFrction, float restitution);
	void addDynamicBody(physx::PxScene * world, physx::PxCooking* mCooking, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform, float staticFriction, float dynamicFrction, float restitution);
	void setMass(std::string name, float value);
	void setStaticFriction(std::string name, float value);
	void setDynamicFriction(std::string name, float value);
	void setRestitution(std::string name, float value);
	void move(std::string scene, float * transform);
	void setForce(std::string scene, float * force);

protected:
	std::map<std::string, PhysXScene> rigidBodies;
	physx::PxInputStream * getTriangleMeshGeo(physx::PxScene *world, physx::PxCooking* mCooking, ExternalInfo externInfo, bool isStatic = true);
	//void setExtInfo(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform);
};

#endif
