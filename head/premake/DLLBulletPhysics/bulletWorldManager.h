#ifndef _BULLETWORLDMANAGER_H
#define _BULLETWORLDMANAGER_H

#include "nau/physics/iPhysics.h"
#include "btBulletDynamicsCommon.h"
#include "btBulletCollisionCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>

#include "bulletRigidManager.h"
#include "bulletSoftManager.h"

class BulletWorldManager {

private:
	btSoftRigidDynamicsWorld * world;
	BulletRigidManager * rigidManager;
	BulletSoftManager * softManager;

public:
	BulletWorldManager();
	~BulletWorldManager();

	void update();
	void setGravity(float x, float y, float z);
	void addRigid(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform, bool isStatic = false);
	void setRigidProperty(std::string scene, std::string propName, float value);
	void moveRigid(std::string scene, float * transform);

	void addCloth(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform);
	void setSoftProperty(std::string scene, std::string propName, float value);
	void moveSoft(std::string scene, float * transform);

	//void addCharacter();

	btDynamicsWorld * getWorld() { return world; };

};

#endif