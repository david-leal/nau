#include "bulletCharacterManager.h"
#include "bulletMotionState.h"

int BulletCharacterManager::nextControllerIndex = 0;

BulletCharacterManager::BulletCharacterManager() {
	cameraPositions = new std::map<std::string, float *>();
	currentCharacter = "";
}


BulletCharacterManager::~BulletCharacterManager() {
	delete &controllers;
	delete &cameraPositions;
}

void BulletCharacterManager::update(btSoftRigidDynamicsWorld * world) {
	for (auto scene : controllers) {
		currentCharacter = scene.first;
		//world->contactTest(scene.second.sceneInfo.object, *scene.second.callBack);
		world->contactTest(scene.second.sceneInfo.object, *this);
		btGhostObject * ghost = btGhostObject::upcast(scene.second.sceneInfo.object);
		btTransform trans = ghost->getWorldTransform();

		btVector3 up = world->getGravity().normalize() * -1.0f;
		btVector3 dir = *controllers[scene.first].direction;
		btVector3 right = up.cross(dir).normalize() * -1.0f;

		btMatrix3x3 mat = btMatrix3x3(
			right.getX(), up.getX(), dir.getX(),
			right.getY(), up.getY(), dir.getY(),
			right.getZ(), up.getZ(), dir.getZ()
		);

		btTransform tr = btTransform(trans.getBasis() * mat, trans.getOrigin());
		tr.getOpenGLMatrix(scene.second.sceneInfo.extInfo.transform);

		//btManifoldArray manifoldArray;
		//btAlignedObjectArray<btCollisionObject*>& objArray = (btGhostObject::upcast(scene.second.sceneInfo.object))->getOverlappingPairs();

		//for (int i = 0; i < objArray.size(); i++) {
		//	manifoldArray.clear();
		//	btBroadphasePair* collisionPair = world->getPairCache()->findPair(scene.second.sceneInfo.object->getBroadphaseHandle(), objArray[i]->getBroadphaseHandle());

		//	if (!collisionPair) continue;

		//	if (collisionPair->m_algorithm)
		//		collisionPair->m_algorithm->getAllContactManifolds(manifoldArray);

		//	for (int j = 0; j<manifoldArray.size(); j++) {
		//		btPersistentManifold* manifold = manifoldArray[j];

		//		bool isFirstBody = manifold->getBody0() == scene.second.sceneInfo.object;
		//		bool isStatic = manifold->getBody1()->getCollisionFlags() == btCollisionObject::CollisionFlags::CF_STATIC_OBJECT;


		//		btScalar direction = isFirstBody ? btScalar(-1.0) : btScalar(1.0);

		//		if (!isStatic) {
		//			for (int p = 0; p < manifold->getNumContacts(); ++p) {
		//				const btManifoldPoint&pt = manifold->getContactPoint(p);

		//				if (pt.getDistance() < 0.f) {
		//					const btVector3& ptA = pt.getPositionWorldOnA();
		//					const btVector3& ptB = pt.getPositionWorldOnB();
		//					const btVector3& normalOnB = pt.m_normalWorldOnB;

		//					// handle collisions here
		//				}
		//			}
		//		}
		//	}
		//}
	}
}

void BulletCharacterManager::addCharacter(btSoftRigidDynamicsWorld * world, const std::string & scene, float height, float radius, float stepHeight) {
	//INFO: Radius, Height and StepHeight have to be set in intitialization and cannot be change
	//TODO: (remains to be tested by removing the character from the world and make the changes)
	btTransform startTransform;
	startTransform.setFromOpenGLMatrix(controllers[scene].sceneInfo.extInfo.transform);

	btConvexShape* capsule = new btCapsuleShape(radius, height);

	btPairCachingGhostObject* m_ghostObject = new btPairCachingGhostObject();
	m_ghostObject->setWorldTransform(startTransform);
	world->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(new btGhostPairCallback());
	m_ghostObject->setCollisionShape(capsule);
	m_ghostObject->setCollisionFlags(btCollisionObject::CF_CHARACTER_OBJECT);
	m_ghostObject->setUserPointer(static_cast<void *>(new std::string(scene)));

	btKinematicCharacterController* charCon = new btKinematicCharacterController(m_ghostObject, capsule, stepHeight);
	charCon->setGravity(-world->getGravity().getY());
	controllers[scene].controller = charCon;

	world->addCollisionObject(m_ghostObject, btBroadphaseProxy::CharacterFilter, btBroadphaseProxy::AllFilter);
	world->addAction(charCon);
	controllers[scene].sceneInfo.object = m_ghostObject;
	//controllers[scene].callBack = new ContactSensorCallback(*m_ghostObject);
}

void BulletCharacterManager::addCamera(btSoftRigidDynamicsWorld * world, const std::string &scene, float height, float radius, float stepHeight)
{
}

void BulletCharacterManager::move(const std::string & scene) {

}

void BulletCharacterManager::move(const std::string & scene, float * transform) {

}

void BulletCharacterManager::createInfo(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	controllers[scene].sceneInfo.extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
}

void BulletCharacterManager::setDirection(std::string scene, btVector3 dir) {
	btKinematicCharacterController * controller = getKinematicController(scene);
	if (controller) {
		if (controllers[scene].direction) {
			controllers[scene].direction->setX((btScalar)dir.getX());
			controllers[scene].direction->setY((btScalar)dir.getY());
			controllers[scene].direction->setZ((btScalar)dir.getZ());
		}
		else
			controllers[scene].direction = new btVector3(dir);
		controller->setWalkDirection(dir * controllers[scene].pace);
	}
}

void BulletCharacterManager::setPace(std::string scene, float pace) {
	btKinematicCharacterController * controller = getKinematicController(scene);
	if (controller) {
		controllers[scene].pace = pace;
		controller->setWalkDirection(*controllers[scene].direction * pace);
	}
}

void BulletCharacterManager::setMinPace(std::string scene, float pace) {
}

void BulletCharacterManager::setHitMagnitude(std::string scene, float hitMagnitude) {
	if (isPresent(scene))
		controllers[scene].hitMagnitude = hitMagnitude;
}

void BulletCharacterManager::setMass(std::string scene, float value) {
}

void BulletCharacterManager::setFriction(std::string scene, float value)
{
}

void BulletCharacterManager::setRestitution(std::string scene, float value)
{
}

void BulletCharacterManager::setHeight(std::string scene, float value) {

}

void BulletCharacterManager::setRadius(std::string scene, float value) {

}

void BulletCharacterManager::setStepOffset(std::string scene, float value)
{
}

void BulletCharacterManager::setTimeStep(std::string scene, float value)
{
}

btScalar BulletCharacterManager::addSingleResult(btManifoldPoint & cp, const btCollisionObjectWrapper * colObj0, int partId0, int index0, const btCollisionObjectWrapper * colObj1, int partId1, int index1) {
	bool isFirstBody = colObj0->m_collisionObject == controllers[currentCharacter].sceneInfo.object;
	std::string *n = static_cast<std::string*>(isFirstBody ? colObj0->m_collisionObject->getUserPointer() : colObj1->m_collisionObject->getUserPointer());
	bool isStatic = colObj1->m_collisionObject->getCollisionFlags() == btCollisionObject::CF_STATIC_OBJECT;

	btScalar direction = isFirstBody ? btScalar(-1.0) : btScalar(1.0);

	if (!isStatic) {
		const btVector3& ptA = cp.getPositionWorldOnA();
		const btVector3& ptB = cp.getPositionWorldOnB();
		const btVector3& normalOnB = cp.m_normalWorldOnB;
		(btRigidBody::upcast((btCollisionObject*)colObj1->m_collisionObject))->applyCentralForce(
			(normalOnB * direction) * controllers[*n].hitMagnitude
		);
	}
	return 0;
}

btKinematicCharacterController * BulletCharacterManager::getKinematicController(const std::string & scene) {
	if (isPresent(scene))
		return controllers[scene].controller;
	return NULL;
}

btGhostObject * BulletCharacterManager::getGhostObject(const std::string & scene) {
	if (isPresent(scene))
		return btGhostObject::upcast(controllers[scene].sceneInfo.object);
	return NULL;
}

bool BulletCharacterManager::isPresent(const std::string & scene) {
	return controllers.find(scene) != controllers.end();
}
