#include "nau/world/bulletWorld.h"

#include "nau/geometry/iBoundingVolume.h"
#include "nau/world/bulletMotionState.h"
#include "nau/geometry/vertexData.h"
#include "nau/material/materialGroup.h"
#include "nau\world\bulletDebugger.h"

using namespace nau::world;
using namespace nau::geometry;
using namespace nau::scene;
using namespace nau::render;
using namespace nau::material;


BulletWorld::BulletWorld(void): m_pScene (0), m_pDynamicsWorld (0) {
}


BulletWorld::~BulletWorld(void) {
	delete m_pDynamicsWorld;
}


void 
BulletWorld::update (void) {
	if (0 != m_pDynamicsWorld) {
		m_pDynamicsWorld->stepSimulation(1 / 60.0f);
		m_pDynamicsWorld->debugDrawWorld();
	}
}


void 
BulletWorld::build (void)
{
	if (0 == m_pDynamicsWorld) {
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new	btCollisionDispatcher (collisionConfiguration);

		btBroadphaseInterface* broadphase = new btDbvtBroadphase();
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

		btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

		m_pDynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
		m_pDynamicsWorld->setGravity(btVector3(0,-10,0));
		
		BulletDebugger *debugDrawer = new BulletDebugger();
		//debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		debugDrawer->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
		m_pDynamicsWorld->setDebugDrawer(debugDrawer);
	}
}


void 
BulletWorld::setScene (nau::scene::IScene *aScene) {
	m_pScene = aScene;
}

btCollisionShape* getMeshShape(std::shared_ptr<nau::scene::IScene> &aScene, bool isStatic=true) {
	std::shared_ptr<nau::scene::SceneObject> &aObject = aScene->getSceneObject(0);
	std::shared_ptr<VertexData> &vd = aObject->getRenderable()->getVertexData();
	std::vector<std::shared_ptr<MaterialGroup>> &matGroups = aObject->getRenderable()->getMaterialGroups();
	std::vector<std::shared_ptr<MaterialGroup>>::iterator matGroupsIter;

	btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray();

	matGroupsIter = matGroups.begin();
	for (; matGroupsIter != matGroups.end(); matGroupsIter++) {
		if ((*matGroupsIter)->getIndexData()->getIndexSize()) {
			std::shared_ptr<std::vector<unsigned int>> &indexes = (*matGroupsIter)->getIndexData()->getIndexData();
			btIndexedMesh *mesh = new btIndexedMesh();
			mesh->m_numTriangles = static_cast<int> (indexes->size() / 3);
			mesh->m_triangleIndexBase = reinterpret_cast<const unsigned char *>(&((*indexes)[0]));
			mesh->m_triangleIndexStride = 3 * sizeof(unsigned int);
			mesh->m_numVertices = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());
			mesh->m_vertexBase = reinterpret_cast<const unsigned char *>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0)));
			mesh->m_vertexStride = 4 * sizeof(float);
			indexVertexArrays->addIndexedMesh(*mesh, PHY_INTEGER);
		}
	}
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

void 
BulletWorld::_addRigid(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {
	btRigidBody* body;
	if (name.compare("plane") == 0) {
		NauBulletMotionState *motionState = new NauBulletMotionState(aScene);
		btVector3 localInertia(0, 0, 0);
		//btCollisionShape *trimeshShape = getMeshShape(aScene);
		//body = new btRigidBody(0, motionState, trimeshShape, localInertia);
		body = new btRigidBody(0, motionState, new btStaticPlaneShape(btVector3(0, 1, 0), 0), localInertia);
		//body = new btRigidBody(0, motionState, new btBoxShape(btVector3(1.0, 1.0, 1.0)), localInertia);

		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
		body->setRestitution(1.0f);
	}
	else {
		btCollisionShape *aShape = getMeshShape(aScene, false);
		//aShape->setLocalScaling(btVector3(0.5f, 0.5f, 0.5f));
		//btCollisionShape *aShape = new btBoxShape(k);
		//btCollisionShape *aShape = new btSphereShape(aVec.z);
		NauBulletMotionState *motionState = new NauBulletMotionState(aScene);

		btVector3 localInertia(0, 0, 0);
		aShape->calculateLocalInertia(mass, localInertia);
		body = new btRigidBody(mass, motionState, aShape, localInertia);
		body->setFriction(0.2f);
		//m_RigidBodies[name]->setActivationState (DISABLE_DEACTIVATION);
		//body->setAngularFactor(0.0f);
		//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		body->setRestitution(0.3f);
		//body->setDeactivationTime(0.5);
	}
	m_RigidBodies[name] = body;
	m_pDynamicsWorld->addRigidBody(body);
}

void
BulletWorld::_addCloth(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {

}

void
BulletWorld::_addParticles(nau::render::Pass* pass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::material::IBuffer* positions) {

}

void 
BulletWorld::setKinematic (std::string name) {
	m_RigidBodies[name]->setCollisionFlags (m_RigidBodies[name]->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	m_RigidBodies[name]->setActivationState (DISABLE_DEACTIVATION);
}

void 
BulletWorld::setDynamic (std::string name) {
	m_RigidBodies[name]->setCollisionFlags (m_RigidBodies[name]->getCollisionFlags() ^ btCollisionObject::CF_KINEMATIC_OBJECT);
	m_RigidBodies[name]->activate();
}

void 
BulletWorld::setVelocity (std::string name, nau::math::vec3 vel) {
	if (m_RigidBodies.count(name)) {
		//m_RigidBodies[name]->setLinearVelocity (btVector3 (vel.x, vel.y, vel.z));
		if (m_RigidBodies[name]->getVelocityInLocalPoint (m_RigidBodies[name]->getCenterOfMassPosition()).length() < 10.0f) {
			m_RigidBodies[name]->applyCentralImpulse (btVector3 (3*vel.x, 3*vel.y, 3*vel.z));//, m_RigidBodies[name]->getCenterOfMassPosition()); /**???**/
		}
	}
}

void 
BulletWorld::disableObject (std::string name) {
	m_pDynamicsWorld->removeCollisionObject (m_RigidBodies[name]);
}

void 
BulletWorld::enableObject (std::string name) {
	m_pDynamicsWorld->addCollisionObject (m_RigidBodies[name]);
}
