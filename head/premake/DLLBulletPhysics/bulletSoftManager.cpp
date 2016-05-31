#include "bulletSoftManager.h"



BulletSoftManager::BulletSoftManager() {
}


BulletSoftManager::~BulletSoftManager() {
}

void BulletSoftManager::update() {
	for (auto scene : softBodies) {
		btSoftBody * cloth = btSoftBody::upcast(scene.second.object);
		float m[16];
		cloth->getWorldTransform().getOpenGLMatrix(m);
		for (int k = 0; k < 16; k++) { scene.second.extInfo.transform[k] = m[k]; }
		btSoftBody::tNodeArray&   nodes(cloth->m_nodes);
		for (int i = 0; i<nodes.size(); ++i) {
			scene.second.extInfo.vertices[4 * i] =		 nodes[i].m_x.x();
			scene.second.extInfo.vertices[(4 * i) + 1] = nodes[i].m_x.y();
			scene.second.extInfo.vertices[(4 * i) + 2] = nodes[i].m_x.z();
		}
	}
}

btSoftBody * BulletSoftManager::addSoftBody(btSoftBodyWorldInfo & worldInfo, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	softBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
	btSoftBody * cloth = btSoftBodyHelpers::CreateFromTriMesh(worldInfo, vertices, reinterpret_cast<const int *> (indices), nbIndices / 3);
	btSoftBody::Material * pm = cloth->appendMaterial();
	pm->m_kLST = 0.5;
	cloth->generateBendingConstraints(2, pm);
	cloth->m_cfg.viterations = 50;
	cloth->m_cfg.piterations = 50;
	cloth->m_cfg.kDF = 0.5;
	cloth->randomizeConstraints();
	cloth->setTotalMass(10, true);
	btTransform * trans = new btTransform();
	float m[16];
	for (int i = 0; i < 16; i++) { m[i] = transform[i]; }
	trans->setFromOpenGLMatrix(m);
	cloth->m_initialWorldTransform = *trans;
	softBodies[scene].object = cloth;
	return cloth;
}

void BulletSoftManager::setFriction(std::string name, float value) {
}

void BulletSoftManager::setRestitution(std::string name, float value) {
}

void BulletSoftManager::move(std::string scene, float * transform) {
	if (softBodies.find(scene) != softBodies.end()) {
		btSoftBody * cloth = btSoftBody::upcast(softBodies[scene].object);
		if (cloth) {
			btTransform * trans = new btTransform();
			float m[16];
			for (int i = 0; i < 16; i++) { m[i] = transform[i]; }
			trans->setFromOpenGLMatrix(m);
			cloth->setWorldTransform(*trans);
		}
	}
}
