#ifndef _BULLETRIGIDMANAGER_H
#define _BULLETRIGIDMANAGER_H

#include <map>
#include <vector>
#include "bulletScene.h"
#include "bulletMotionState.h"
#include "btBulletDynamicsCommon.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

class BulletRigidManager {

public:
	BulletRigidManager();
	~BulletRigidManager();

	void update();
	btRigidBody * addStaticBody(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	btRigidBody * addDynamicBody(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	void setMass(std::string name, float value);
	void setFriction(std::string name, float value);
	void setRestitution(std::string name, float value);
	void move(std::string scene, float * transform);

protected:
	std::map<std::string, BulletScene> rigidBodies;
	btCollisionShape* getMeshShape(ExternalInfo externInfo, bool isStatic = true);

};

#endif