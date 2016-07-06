#ifndef _PHYSXRIGIDMANAGER_H
#define _PHYSXRIGIDMANAGER_H

#include <map>
#include <vector>
#include "nau/physics/iPhysics.h"
#include "PxPhysicsAPI.h"
#include "physXScene.h"

class PhysXRigidManager {

public:
	PhysXRigidManager();
	~PhysXRigidManager();

	void update(const physx::PxActiveTransform* activeTransforms, physx::PxU32 nbActiveTransforms);
	void createInfo(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	physx::PxMaterial * createMaterial(physx::PxScene * world, float staticFriction, float dynamicFrction, float restitution);
	void addStaticBody(const std::string & scene, physx::PxScene * world, physx::PxCooking * mCooking, nau::physics::IPhysics::BoundingVolume shape, physx::PxMaterial * material);
	void addDynamicBody(const std::string & scene, physx::PxScene * world, physx::PxCooking * mCooking, nau::physics::IPhysics::BoundingVolume shape, physx::PxMaterial * material);
	void setMass(std::string name, float value);
	void setStaticFriction(std::string name, float value);
	void setDynamicFriction(std::string name, float value);
	void setRestitution(std::string name, float value);
	void move(std::string scene, float * transform);
	void setForce(std::string scene, float * force);

protected:
	std::map<std::string, PhysXScene> rigidBodies;
	physx::PxInputStream * getTriangleMeshGeo(physx::PxScene *world, physx::PxCooking* mCooking, ExternalInfo externInfo, bool isStatic);
};

#endif
