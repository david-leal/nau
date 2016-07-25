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
#include "bulletDebugger.h"

class BulletWorldManager {

private:
	btSoftRigidDynamicsWorld * world;
	BulletRigidManager * rigidManager;
	BulletSoftManager * softManager;
	BulletDebugger * debugDrawer;
	float timeStep;

public:
	BulletWorldManager();
	~BulletWorldManager();

	void update();
	void setGravity(float x, float y, float z);
	void addRigid(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform, nau::physics::IPhysics::BoundingVolume shape, float mass, bool isStatic = false);
	void setRigidProperty(std::string scene, std::string propName, float value);
	void setRigidProperty(std::string scene, std::string propName, float * value);
	void moveRigid(std::string scene, float * transform);

	void addCloth(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform);
	void setSoftProperty(std::string scene, std::string propName, float value);
	void moveSoft(std::string scene, float * transform);
	std::vector<float> * getDebug() { return debugDrawer->getDebugPoints(); }
	void setDebug();

	//void addCharacter();

	void setTimeStep(float tStep) { timeStep = tStep; };

};

#endif