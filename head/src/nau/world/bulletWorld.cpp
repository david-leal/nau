#include "nau/world/bulletWorld.h"

#include "nau/geometry/iBoundingVolume.h"
#include "nau/world/bulletMotionState.h"
#include "nau/geometry/vertexData.h"
#include "nau/material/materialGroup.h"
#include "nau\world\bulletDebugger.h"
#include <BulletCollision\CollisionDispatch\btGhostObject.h>
#include <BulletDynamics\Character\btKinematicCharacterController.h>

using namespace nau::world;
using namespace nau::geometry;
using namespace nau::scene;
using namespace nau::render;
using namespace nau::material;

btSoftBody * cloth;


BulletWorld::BulletWorld(void): m_pScene (0), m_pDynamicsWorld (0) {
}


BulletWorld::~BulletWorld(void) {
	delete m_pDynamicsWorld;
}


void 
BulletWorld::update (void) {
	if (0 != m_pDynamicsWorld) {
		((BulletDebugger*)m_pDynamicsWorld->getDebugDrawer())->clearPoints();
		m_pDynamicsWorld->stepSimulation(1 / 60.0f);

		//nau::scene::IScene *m_IScene = static_cast<nau::scene::IScene*>(cloth->getUserPointer());
		//std::shared_ptr<VertexData> &vd = m_IScene->getSceneObject(0)->getRenderable()->getVertexData();
		//int count = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());

		//std::shared_ptr<std::vector<VertexData::Attr>> points = vd->getDataOf(VertexData::GetAttribIndex(std::string("position")));

		//btSoftBody::tNodeArray&   nodes(cloth->m_nodes);
		//for (int i = 0; i<nodes.size(); ++i) {
		//	/*float x = scene.second.extInfo.vertices[4 * i];
		//	float y = scene.second.extInfo.vertices[(4 * i) + 1];
		//	float z = scene.second.extInfo.vertices[(4 * i) + 2];*/
		//	/*scene.second.extInfo.vertices[4 * i] = nodes[i].m_x.getX();
		//	scene.second.extInfo.vertices[(4 * i) + 1] = nodes[i].m_x.getY();
		//	scene.second.extInfo.vertices[(4 * i) + 2] = nodes[i].m_x.getZ();*/
		//	points->at(i).set(nodes[i].m_x.getX(), nodes[i].m_x.getY(), nodes[i].m_x.getZ());
		//}
		//vd->resetCompilationFlag();
		//vd->compile();

		//m_pDynamicsWorld->debugDrawWorld();
		//((BulletDebugger*)m_pDynamicsWorld->getDebugDrawer())->compilePoints();
	}
}


void 
BulletWorld::build (void)
{
	if (0 == m_pDynamicsWorld) {
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new	btCollisionDispatcher(collisionConfiguration);

		btBroadphaseInterface* broadphase = new btDbvtBroadphase();
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

		btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

		//m_pDynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
		m_pDynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		m_pDynamicsWorld->setGravity(btVector3(0,-10,0));
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
BulletWorld::_addRigid(float mass, float friction, float restitution, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {
	btRigidBody* body;
	NauBulletMotionState *motionState = new NauBulletMotionState(aScene);

	btVector3 localInertia(0, 0, 0);
	if (mass == 0.0f) {
		if (name.compare("plane") == 0) {
			btRigidBody::btRigidBodyConstructionInfo rbGroundInfo(0, motionState, new btStaticPlaneShape(btVector3(0, 1, 0), 0));
			body = new btRigidBody(rbGroundInfo);
		}
		else {
			body = new btRigidBody(0, motionState, getMeshShape(aScene), localInertia);
		}
		body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
	}
	else {
		btCollisionShape *aShape;
		/*if (name.compare("box") == 0) {
			aShape = new btBoxShape(btVector3(aVec.x, aVec.y, aVec.z));
			//aShape = new btSphereShape(aVec.z);
		}
		else {*/
		aShape = getMeshShape(aScene, false);
		//}
		//aShape->setLocalScaling(btVector3(0.5f, 0.5f, 0.5f));
		aShape->calculateLocalInertia(mass, localInertia);
		//btRigidBody::btRigidBodyConstructionInfo rbBodyInfo(mass, motionState, aShape);
		//body = new btRigidBody(rbBodyInfo);
		body = new btRigidBody(mass, motionState, aShape, localInertia);
		//m_RigidBodies[name]->setActivationState (DISABLE_DEACTIVATION);
		//body->setAngularFactor(0.0f);
		//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		//body->setDeactivationTime(0.5);
	}
	body->setRestitution(restitution);
	body->setFriction(friction);
	m_RigidBodies[name] = body;
	m_pDynamicsWorld->addRigidBody(body);
}

void
BulletWorld::_addCharacter(float mass, float radius, float height, float stedHeight, std::shared_ptr<nau::scene::IScene> &aScene, std::string name) {
	btTransform startTransform;
	startTransform.setFromOpenGLMatrix(aScene->getTransform().getMatrix());

	btConvexShape* capsule = new btCapsuleShape(radius, height);

	btPairCachingGhostObject* m_ghostObject = new btPairCachingGhostObject();
	m_ghostObject->setWorldTransform(startTransform);
	m_pDynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	m_ghostObject->setCollisionShape(capsule);
	m_ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);

	btKinematicCharacterController* charCon = new btKinematicCharacterController(m_ghostObject, capsule, stedHeight);

	charCon->setGravity(-m_pDynamicsWorld->getGravity().getY());
	charCon->setWalkDirection(btVector3(0.0f, 0.0f, -0.5f));

	m_pDynamicsWorld->addCollisionObject(m_ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::AllFilter);
	m_pDynamicsWorld->addAction(charCon);
}

void
BulletWorld::_addCloth(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) {

	/*std::shared_ptr<nau::scene::SceneObject> &aObject = aScene->getSceneObject(0);
	std::shared_ptr<VertexData> &vd = aObject->getRenderable()->getVertexData();
	std::vector<std::shared_ptr<MaterialGroup>> &matGroups = aObject->getRenderable()->getMaterialGroups();
	std::vector<std::shared_ptr<MaterialGroup>>::iterator matGroupsIter;

	int count = static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size());

	matGroupsIter = matGroups.begin();
	for (; matGroupsIter != matGroups.end(); matGroupsIter++) {
		if ((*matGroupsIter)->getIndexData()->getIndexSize()) {
			std::shared_ptr<std::vector<unsigned int>> &indexes = (*matGroupsIter)->getIndexData()->getIndexData();
			cloth = btSoftBodyHelpers::CreateFromTriMesh(
				m_pDynamicsWorld->getWorldInfo(),
				reinterpret_cast<const float *>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0))),
				reinterpret_cast<const int *> (reinterpret_cast<const unsigned char *>(&((*indexes)[0]))),
				static_cast<int> (indexes->size() / 3)
			);
		}
	}
	btTransform trans;
	trans.setFromOpenGLMatrix(aScene->getTransform().getMatrix());
	cloth->setWorldTransform(trans);
	cloth->setUserPointer(static_cast<void*>(aScene.get()));
	m_pDynamicsWorld->addSoftBody(cloth);*/
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


void
BulletWorld::setDebug(nau::scene::IScene* debugScene, nau::material::IBuffer* debugPositions) {
	BulletDebugger *debugDrawer = new BulletDebugger(debugScene, debugPositions);
	debugDrawer->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
	//debugDrawer->setDebugMode(btIDebugDraw::DBG_MAX_DEBUG_DRAW_MODE);
	m_pDynamicsWorld->setDebugDrawer(debugDrawer);
}