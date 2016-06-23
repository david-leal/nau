#ifndef _PHYSXCHARACTERMANAGER_H
#define _PHYSXCHARACTERMANAGER_H

#include <map>
#include <vector>
#include "PxPhysicsAPI.h"
#include "physXScene.h"

class PhysXCharacterManager : public physx::PxUserControllerHitReport {

public:
	static int nextControllerIndex;

	PhysXCharacterManager(physx::PxScene * world);
	~PhysXCharacterManager();

	void update(float time, physx::PxVec3 gravity = physx::PxVec3(0.0f));
	void addCharacter(physx::PxScene * world, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	void move(const std::string & scene, float time, physx::PxVec3 gravity = physx::PxVec3(0.0f));
	void setDirection(std::string scene, physx::PxVec3 dir);
	void setPace(std::string scene, float pace);
	void setHitMagnitude(std::string scene, float hitMagnitude);
	void setMass(std::string scene, float value);
	void setFriction(std::string scene, float value);
	void setRestitution(std::string scene, float value);
	void setHeight(std::string scene, float value); //height
	void setRadius(std::string scene, float value); //radius
	void setStepOffset(std::string scene, float value);	//stepOffset

protected:
	

	typedef struct {
		externalInfo extInfo;
		int index;
		physx::PxVec3 * direction;
		float pace;
		float hitMagnitude;
	} PhysXController;

	std::map<std::string, PhysXController> controllers;
	physx::PxControllerManager * manager;

	// Inherited via PxUserControllerHitReport
	virtual void onShapeHit(const physx::PxControllerShapeHit & hit) override;
	virtual void onControllerHit(const physx::PxControllersHit & hit) override;
	virtual void onObstacleHit(const physx::PxControllerObstacleHit & hit) override;
};

#endif