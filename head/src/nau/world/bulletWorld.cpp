#include "nau/world/bulletWorld.h"

#include "nau/geometry/iBoundingVolume.h"
#include "nau/world/bulletMotionState.h"
#include "nau/geometry/vertexData.h"
#include "nau/material/materialGroup.h"

using namespace nau::world;
using namespace nau::geometry;
using namespace nau::scene;
using namespace nau::render;
using namespace nau::material;


BulletWorld::BulletWorld(void): m_pScene (0), m_pDynamicsWorld (0)
{
}


BulletWorld::~BulletWorld(void)
{
	delete m_pDynamicsWorld;
}


void 
BulletWorld::update (void)
{
	if (0 != m_pDynamicsWorld) {
		m_pDynamicsWorld->stepSimulation(1 / 120.0f);
		//m_pDynamicsWorld->updateAabbs();
		//m_pDynamicsWorld->debugDrawWorld();
	}
}


void 
BulletWorld::build (void) /***MARK***/ //I'm assuming all objects inside scene are static objects
{
	/*if (0 != m_pScene) {
		if (0 != m_pDynamicsWorld) {
			delete m_pDynamicsWorld;
			m_pDynamicsWorld = 0;
		}*/

	if (0 == m_pDynamicsWorld) {
		btDefaultCollisionConfiguration* collisionConfiguration = new btDefaultCollisionConfiguration();
		btCollisionDispatcher* dispatcher = new	btCollisionDispatcher (collisionConfiguration);

		//IBoundingVolume &sceneAabb = m_pScene->getBoundingVolume();

		//btVector3 worldAabbMin  (sceneAabb.getMin().x, sceneAabb.getMin().y, sceneAabb.getMin().z);
		//btVector3 worldAabbMax  (sceneAabb.getMax().x, sceneAabb.getMax().y, sceneAabb.getMax().z);

		//btBroadphaseInterface* broadphase = new btAxisSweep3 (worldAabbMin,worldAabbMax);//  new btMultiSapBroadphase();//new btAxisSweep3(worldAabbMin,worldAabbMax,maxProxies);
		btBroadphaseInterface* broadphase = new btDbvtBroadphase();
		btSequentialImpulseConstraintSolver* solver = new btSequentialImpulseConstraintSolver;

		btGImpactCollisionAlgorithm::registerAlgorithm(dispatcher);

		m_pDynamicsWorld = new btDiscreteDynamicsWorld(dispatcher,broadphase,solver,collisionConfiguration);
		m_pDynamicsWorld->setGravity(btVector3(0,-10,0)); /***MARK***/ //Should be user definable
		
		//m_pDynamicsWorld->getDebugDrawer()->setDebugMode(btIDebugDraw::DBG_DrawWireframe);
		

		//std::vector<std::shared_ptr<SceneObject>> sceneObjects;
		//m_pScene->getAllObjects(&sceneObjects);

		//for (auto &so:  sceneObjects) {
		//	std::shared_ptr<VertexData> &vd = so->getRenderable()->getVertexData();
		//	

		//	std::vector<std::shared_ptr<MaterialGroup>> &matGroups = so->getRenderable()->getMaterialGroups();
		//	std::vector<std::shared_ptr<MaterialGroup>>::iterator matGroupsIter;

		//	matGroupsIter = matGroups.begin();

		//	for ( ; matGroupsIter != matGroups.end(); matGroupsIter++) {

		//		if ((*matGroupsIter)->getIndexData()->getIndexSize()) {
		//		
		//			std::shared_ptr<std::vector<unsigned int>> &indexes = (*matGroupsIter)->getIndexData()->getIndexData();
		//			btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray (
		//					static_cast<int> (indexes->size() / 3), 
		//					reinterpret_cast<int *>(&((*indexes)[0])),
		//					3 * sizeof(unsigned int),
		//					static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size()),
		//					reinterpret_cast<btScalar*>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0))),
		//					/*3*/4 * sizeof (float)
		//				);


		//			bool useQuantizedAabbCompression = true;
		//			btBvhTriangleMeshShape *trimeshShape  = new btBvhTriangleMeshShape(indexVertexArrays,useQuantizedAabbCompression);

		//			NauBulletMotionState *motionState = new NauBulletMotionState (so);

		//			btVector3 localInertia (0, 0, 0);
		//			btRigidBody* body = new btRigidBody(0,motionState,trimeshShape,localInertia);

		//			if (0 != so->getName().compare ("pPlane1")){
		//				body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
		//			} else {
		//				m_RigidBodies["water"] = body;

		//				body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		//				body->setActivationState (DISABLE_DEACTIVATION);
		//			}
		//			body->setRestitution (0.0f);

		//			m_pDynamicsWorld->addRigidBody (body);
		//		}
		//	}
		//}
	}
}


void 
BulletWorld::setScene (nau::scene::IScene *aScene) 
{
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
		//return new btConvexTriangleMeshShape(indexVertexArrays);
		btGImpactMeshShape *gImpa = new btGImpactMeshShape(indexVertexArrays);
		gImpa->updateBound();
		return gImpa;
	}
}

void 
BulletWorld::_add (float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec)
{
	btRigidBody* body;
	if (name.compare("plane") == 0) {

		//std::shared_ptr<nau::scene::SceneObject> &aObject = aScene->getSceneObject(0);
		//std::shared_ptr<VertexData> &vd = aObject->getRenderable()->getVertexData();
		//std::vector<std::shared_ptr<MaterialGroup>> &matGroups = aObject->getRenderable()->getMaterialGroups();
		//std::vector<std::shared_ptr<MaterialGroup>>::iterator matGroupsIter;

		//matGroupsIter = matGroups.begin();

		//for (; matGroupsIter != matGroups.end(); matGroupsIter++) {

		//	if ((*matGroupsIter)->getIndexData()->getIndexSize()) {

		//		std::shared_ptr<std::vector<unsigned int>> &indexes = (*matGroupsIter)->getIndexData()->getIndexData();
		//		btTriangleIndexVertexArray* indexVertexArrays = new btTriangleIndexVertexArray(
		//			static_cast<int> (indexes->size() / 3),
		//			reinterpret_cast<int *>(&((*indexes)[0])),
		//			3 * sizeof(unsigned int),
		//			static_cast<int> (vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->size()),
		//			reinterpret_cast<btScalar*>(&(vd->getDataOf(VertexData::GetAttribIndex(std::string("position")))->at(0))),
		//			/*3*/4 * sizeof(float)
		//			);


		//		bool useQuantizedAabbCompression = true;
		//		btBvhTriangleMeshShape *trimeshShape = new btBvhTriangleMeshShape(indexVertexArrays, useQuantizedAabbCompression);

				//btBvhTriangleMeshShape *trimeshShape = getMeshShape(aScene);
				btCollisionShape *trimeshShape = getMeshShape(aScene);
				NauBulletMotionState *motionState = new NauBulletMotionState(aScene);

				btVector3 localInertia(0, 0, 0);
				body = new btRigidBody(0, motionState, trimeshShape, localInertia);
				//body = new btRigidBody(0, motionState, new btStaticPlaneShape(btVector3(0, 1, 0), 0), localInertia);
				//body = new btRigidBody(0, motionState, new btBoxShape(btVector3(1.0, 1.0, 1.0)), localInertia);

				body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_STATIC_OBJECT);
				body->setRestitution(1.0f);
				//m_RigidBodies[name] = body;
//			}
//		}
	}
	else {
		btCollisionShape *aShape = getMeshShape(aScene, false);
		btVector3 k; k.setX(aVec.x);
		k.setY(aVec.y);
		k.setZ(aVec.z);
		//btCollisionShape *aShape = new btBoxShape(k);// btSphereShape(aVec.z);
		//btCollisionShape *aShape = new btSphereShape(aVec.z);
		NauBulletMotionState *motionState = new NauBulletMotionState(aScene);

		btVector3 localInertia(0, 0, 0);
		aShape->calculateLocalInertia(mass, localInertia);
		body = new btRigidBody(mass, motionState, aShape, localInertia);
		body->setFriction(0.2f);

		//m_RigidBodies[name] = body;
		//m_RigidBodies[name]->setActivationState (DISABLE_DEACTIVATION);
		//body->setAngularFactor(0.0f);
		//body->setCollisionFlags(body->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
		body->setRestitution(0.5f);
		body->setDeactivationTime(0.5);
	}
	m_RigidBodies[name] = body;
	m_pDynamicsWorld->addRigidBody(body);
}

void 
BulletWorld::setKinematic (std::string name)
{
	m_RigidBodies[name]->setCollisionFlags (m_RigidBodies[name]->getCollisionFlags() | btCollisionObject::CF_KINEMATIC_OBJECT);
	m_RigidBodies[name]->setActivationState (DISABLE_DEACTIVATION);
}

void 
BulletWorld::setDynamic (std::string name)
{
	m_RigidBodies[name]->setCollisionFlags (m_RigidBodies[name]->getCollisionFlags() ^ btCollisionObject::CF_KINEMATIC_OBJECT);
	m_RigidBodies[name]->activate();
}

void 
BulletWorld::setVelocity (std::string name, nau::math::vec3 vel)
{
	if (m_RigidBodies.count(name)) {
		//m_RigidBodies[name]->setLinearVelocity (btVector3 (vel.x, vel.y, vel.z));
		if (m_RigidBodies[name]->getVelocityInLocalPoint (m_RigidBodies[name]->getCenterOfMassPosition()).length() < 10.0f) {
			m_RigidBodies[name]->applyCentralImpulse (btVector3 (3*vel.x, 3*vel.y, 3*vel.z));//, m_RigidBodies[name]->getCenterOfMassPosition()); /**???**/
		}
	}
}

void 
BulletWorld::disableObject (std::string name)
{
	m_pDynamicsWorld->removeCollisionObject (m_RigidBodies[name]);
}

void 
BulletWorld::enableObject (std::string name)
{
	m_pDynamicsWorld->addCollisionObject (m_RigidBodies[name]);
}
