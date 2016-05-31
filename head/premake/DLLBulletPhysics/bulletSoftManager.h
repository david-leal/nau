#ifndef _BULLETSOFTMANAGER_H
#define _BULLETSOFTMANAGER_H

#include <map>
#include <vector>
#include "bulletScene.h"
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btDefaultSoftBodySolver.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>


class BulletSoftManager {

public:
	BulletSoftManager();
	~BulletSoftManager();

	void update();
	btSoftBody * addSoftBody(btSoftBodyWorldInfo & worldInfo, const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);
	void setFriction(std::string name, float value);
	void setRestitution(std::string name, float value);
	void move(std::string scene, float * transform);

protected:
	std::map<std::string, BulletScene> softBodies;
};

#endif