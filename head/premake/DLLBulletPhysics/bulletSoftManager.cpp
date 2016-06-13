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

		//btSoftBody::Node& nodeRef(cloth->m_nodes.at(0));
		btSoftBody::tNodeArray&   nodes(cloth->m_nodes);
		for (int i = 0; i < nodes.size(); i++) {
			/*scene.second.extInfo.vertices[4 * i] =		 nodes[i].m_x.x();
			scene.second.extInfo.vertices[(4 * i) + 1] = nodes[i].m_x.y();
			scene.second.extInfo.vertices[(4 * i) + 2] = nodes[i].m_x.z();*/
			for (int j = 0; j < 3; j++) {
				scene.second.extInfo.vertices[(4 * i) + j] = nodes[i].m_x.m_floats[j];
			}
		}

		/*btSoftBody::tFaceArray& faces(cloth->m_faces);
		for (int i = 0; i < faces.size(); i++) {
			for (int j = 0; j < 3; j++) {
				btSoftBody::Node* node = faces[i].m_n[j];
				int index = int(node - &nodeRef);
				for (int k = 0; k < 3; k++) {
					scene.second.extInfo.vertices[(index * 4) + k] = node->m_x.m_floats[k];
				}
				scene.second.extInfo.indices[(i * 3) + j] = index;
			}
		}*/
		//cloth->setWindVelocity(btVector3(9.0f, 0.0, -1.0f));
		//cloth->addForce(btVector3(1.0f, 0.0f, -1.0f));
		btSoftBody::PSolve_Anchors(cloth, 0.0f, 0.0f);
	}
}

btSoftBody * BulletSoftManager::addSoftBody(btSoftBodyWorldInfo & worldInfo, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	softBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);

	float * newPoints = new float[(nbVertices * 3)];
	for (int i = 0; i < nbVertices; i++) {
		for (int j = 0; j < 3; j++) {
			newPoints[(i * 3) + j] = vertices[(i * 4) + j];
		}
	}

	btSoftBody * cloth = btSoftBodyHelpers::CreateFromTriMesh(
		worldInfo,
		newPoints,
		reinterpret_cast<const int *>(indices),
		static_cast<int>(nbIndices / 3)
	);
	/*btSoftBody::Material * pm = cloth->appendMaterial();
	pm->m_kLST = 0.5;
	cloth->generateBendingConstraints(2, pm);
	cloth->m_cfg.viterations = 50;
	cloth->m_cfg.piterations = 50;
	cloth->m_cfg.kDF = 0.5;
	cloth->randomizeConstraints();*/

	//cloth->setTotalMass(0.2f);
	//cloth->setWindVelocity(btVector3(9.0f, 0.0, -1.0f));
	//cloth->addForce(btVector3(1.0f, 0.0f, -1.0f));
	//cloth->m_cfg.aeromodel = btSoftBody::eAeroModel::V_TwoSidedLiftDrag;

	float m[16];
	for (int i = 0; i < 16; i++) { m[i] = transform[i]; }
	btTransform * trans = new btTransform();
	trans->setFromOpenGLMatrix(m);
	//cloth->setWorldTransform(*trans);
	cloth->transform(*trans);

	btTransform * startTransform = new btTransform();
	startTransform->setIdentity();
	startTransform->setOrigin(btVector3(0.0f, 0.0f, 0.0f));
	btDefaultMotionState * motion = new btDefaultMotionState(*startTransform);
	btRigidBody * body = new btRigidBody(0, motion, new btBoxShape(btVector3(1.0f, 1.0f, 1.0f)));

	cloth->appendAnchor(1, body);
	//cloth->appendAnchor(19, body);

	cloth->setUserPointer(static_cast<void *>(new std::string(scene)));

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
			float m[16];
			for (int i = 0; i < 16; i++) { m[i] = transform[i]; }
			btTransform * trans = new btTransform();
			trans->setFromOpenGLMatrix(m);
			cloth->transform(*trans);
			//cloth->setWorldTransform(*trans);
		}
	}
}
