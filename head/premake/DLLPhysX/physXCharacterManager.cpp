#include "physXCharacterManager.h"

int PhysXCharacterManager::nextControllerIndex = 0;

using namespace physx;

PhysXCharacterManager::PhysXCharacterManager(physx::PxScene * world) {
	manager = PxCreateControllerManager(*world);
}


PhysXCharacterManager::~PhysXCharacterManager() {
	manager->release();
	delete &manager;
	delete &controllers;
}

void PhysXCharacterManager::update(float time, physx::PxVec3 gravity) {
	for (auto scene : controllers) {
		move(scene.first, time, gravity);
	}
}

void PhysXCharacterManager::addCharacter(physx::PxScene * world, const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	PxPhysics * gPhysics = &(world->getPhysics());
	PxCapsuleControllerDesc desc;
	//PxControllerDesc desc;
	desc.height = 1.0f;
	desc.radius = 1.0f;
	PxVec3 pos = PxMat44(transform).getPosition();
	desc.position = PxExtendedVec3( pos.x, pos.y, pos.z);
	desc.material = gPhysics->createMaterial(1.0f, 1.0f, 1.0f);
	desc.reportCallback = this;
	desc.climbingMode = PxCapsuleClimbingMode::eCONSTRAINED;
	//desc.stepOffset = stepHeight;
	desc.upDirection = PxVec3(0.0f, 1.0f, 0.0f);
	//desc.slopeLimit = cosf(DegToRad(80.0f));
	desc.userData = static_cast<void*> (new std::string(scene));
	manager->createController(desc);
	controllers[scene].extInfo.nbVertices = nbVertices;
	controllers[scene].extInfo.vertices = vertices;
	controllers[scene].extInfo.nbIndices = nbIndices;
	controllers[scene].extInfo.indices = indices;
	controllers[scene].extInfo.transform = transform;
	controllers[scene].index = PhysXCharacterManager::nextControllerIndex++;
}

void PhysXCharacterManager::move(const std::string & scene, float time, physx::PxVec3 gravity) {
	PxController * controller = manager->getController(controllers[scene].index);
	if (controllers[scene].direction) {
		PxVec3 movement = *controllers[scene].direction + gravity;
		//controller->move(PxVec3(0.0f, -9.81f, -0.3f), 0.2f, 1 / 60.0f, NULL);
		controller->move(movement, controllers[scene].pace, time, NULL);
		PxTransform trans = controller->getActor()->getGlobalPose();
		trans.rotate(*controllers[scene].direction);
		getMatFromPhysXTransform(trans, controllers[scene].extInfo.transform);
	}
}

void PhysXCharacterManager::setDirection(std::string scene, physx::PxVec3 dir) {
	controllers[scene].direction = new PxVec3(dir);
}

void PhysXCharacterManager::setPace(std::string scene, float pace) {
	controllers[scene].pace = pace;
}

void PhysXCharacterManager::setHitMagnitude(std::string scene, float hitMagnitude) {
	controllers[scene].hitMagnitude = hitMagnitude;
}

void PhysXCharacterManager::setMass(std::string scene, float value) {
	manager->getController(controllers[scene].index)->getActor()->setMass(value);
}

void PhysXCharacterManager::setFriction(std::string scene, float value) {
	PxRigidDynamic * actor = manager->getController(controllers[scene].index)->getActor();
	PxU32 nbShapes = actor->getNbShapes();
	std::vector<PxShape *> list(nbShapes);
	actor->getShapes(&list.at(0), nbShapes);
	for (PxShape* shape : list) {
		PxU32 nbMat = shape->getNbMaterials();
		std::vector<PxMaterial *> matList(nbMat);
		shape->getMaterials(&matList.at(0), nbMat);
		for (PxMaterial * mat : matList) {
			mat->setDynamicFriction(value);
			mat->setStaticFriction(value);
		}
	}
}

void PhysXCharacterManager::setRestitution(std::string scene, float value) {
	PxRigidDynamic * actor = manager->getController(controllers[scene].index)->getActor();
	PxU32 nbShapes = actor->getNbShapes();
	std::vector<PxShape *> list(nbShapes);
	actor->getShapes(&list.at(0), nbShapes);
	for (PxShape* shape : list) {
		PxU32 nbMat = shape->getNbMaterials();
		std::vector<PxMaterial *> matList(nbMat);
		shape->getMaterials(&matList.at(0), nbMat);
		for (PxMaterial * mat : matList) {
			mat->setRestitution(value);
		}
	}
}

void PhysXCharacterManager::setHeight(std::string scene, float value) {
	((PxCapsuleController *)manager->getController(controllers[scene].index))->setHeight(value);
}

void PhysXCharacterManager::setRadius(std::string scene, float value) {
	((PxCapsuleController *)manager->getController(controllers[scene].index))->setRadius(value);
}

void PhysXCharacterManager::setStepOffset(std::string scene, float value) {
	manager->getController(controllers[scene].index)->setStepOffset(value);
}

void PhysXCharacterManager::onShapeHit(const physx::PxControllerShapeHit & hit) {
	if (hit.actor->getType() == PxActorType::eRIGID_DYNAMIC) {
		std::string *n = static_cast<std::string*>(hit.controller->getUserData());
		hit.actor->is<PxRigidDynamic>()->addForce(PxVec3(hit.dir) * controllers[*n].hitMagnitude);
	}
}

void PhysXCharacterManager::onControllerHit(const physx::PxControllersHit & hit) {
}

void PhysXCharacterManager::onObstacleHit(const physx::PxControllerObstacleHit & hit) {
}
