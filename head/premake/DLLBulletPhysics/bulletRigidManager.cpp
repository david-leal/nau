#include "bulletRigidManager.h"



BulletRigidManager::BulletRigidManager() {
}


BulletRigidManager::~BulletRigidManager() {
}

void BulletRigidManager::update() {

}

btCollisionShape * BulletRigidManager::getMeshShape(ExternalInfo externInfo, bool isStatic) {

	btTriangleIndexVertexArray * indexVertexArrays = new btTriangleIndexVertexArray();

	btIndexedMesh * mesh = new btIndexedMesh();
	mesh->m_numTriangles = externInfo.nbIndices / 3;
	mesh->m_triangleIndexBase = reinterpret_cast<const unsigned char *>(externInfo.indices);
	mesh->m_triangleIndexStride = 3 * sizeof(unsigned int);
	mesh->m_numVertices = externInfo.nbVertices;
	mesh->m_vertexBase = reinterpret_cast<const unsigned char *>(externInfo.vertices);
	mesh->m_vertexStride = 4 * sizeof(float);
	
	indexVertexArrays->addIndexedMesh(*mesh, PHY_INTEGER);

	if (isStatic) {
		bool useQuantizedAabbCompression = true;
		return new btBvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression);
	}
	else {
		btGImpactMeshShape *gImpa = new btGImpactMeshShape(indexVertexArrays);
		gImpa->updateBound();
		return gImpa;
	}
}

btRigidBody * BulletRigidManager::addStaticBody(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
	btRigidBody * body;
	BulletMotionState * motionState = new BulletMotionState(transform);
	btVector3 localInertia(0, 0, 0);
	if (scene.compare("plane") == 0) {
		btRigidBody::btRigidBodyConstructionInfo rbGroundInfo(0, motionState, new btStaticPlaneShape(btVector3(0, 1, 0), 0));
		body = new btRigidBody(rbGroundInfo);
	}
	else {
		body = new btRigidBody(0, motionState, getMeshShape(rigidBodies[scene].extInfo), localInertia);
	}
	body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	rigidBodies[scene].object = body;
	return body;
}

btRigidBody * BulletRigidManager::addDynamicBody(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	rigidBodies[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
	BulletMotionState * motionState = new BulletMotionState(transform);
	btVector3 localInertia(0, 0, 0);

	btCollisionShape * aShape = getMeshShape(rigidBodies[scene].extInfo, false);
	aShape->calculateLocalInertia(1, localInertia);
	btRigidBody * body = new btRigidBody(1, motionState, aShape, localInertia);
	rigidBodies[scene].object = body;
	return body;
}

void BulletRigidManager::setMass(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		btRigidBody * body = btRigidBody::upcast(rigidBodies[name].object);
		if (body) {
			btVector3 localInertia = body->getLocalInertia();
			//body->setMassProps(value, localInertia);
			//body->updateInertiaTensor();
		}
	}
}

void BulletRigidManager::setFriction(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		btRigidBody * body = btRigidBody::upcast(rigidBodies[name].object);
		if (body) {
			body->setFriction(value);
		}
	}
}

void BulletRigidManager::setRestitution(std::string name, float value) {
	if (rigidBodies.find(name) != rigidBodies.end()) {
		btRigidBody * body = btRigidBody::upcast(rigidBodies[name].object);
		if (body) {
			body->setRestitution(value);
		}
	}
}

void BulletRigidManager::move(std::string scene, float * transform) {
	if (rigidBodies.find(scene) != rigidBodies.end()) {
		btRigidBody * body = btRigidBody::upcast(rigidBodies[scene].object);
		if (body) {
			rigidBodies[scene].extInfo.transform = transform;
			btTransform trans;
			trans.setFromOpenGLMatrix(transform);
			body->getMotionState()->getWorldTransform(trans);
		}
	}
}


