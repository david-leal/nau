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

void PhysXCharacterManager::addCharacter(const std::string & scene, physx::PxMaterial * material, physx::PxVec3 up, bool isCamera) {
	PxCapsuleControllerDesc desc;
	//PxControllerDesc desc;
	desc.height = 1.0f;
	desc.radius = 1.0f;
	PxVec3 pos = PxMat44(controllers[scene].extInfo.transform).getPosition();
	desc.position = PxExtendedVec3( pos.x, pos.y, pos.z);
	desc.material = material;
	desc.reportCallback = this;
	desc.climbingMode = PxCapsuleClimbingMode::eCONSTRAINED;
	//desc.stepOffset = stepHeight;
	desc.upDirection = up.getNormalized();
	//desc.slopeLimit = cosf(DegToRad(80.0f));
	desc.userData = static_cast<void*> (new std::string(scene));
	manager->createController(desc);
	if (!isCamera) {
		PxMat44 initTrans = PxMat44(controllers[scene].extInfo.transform);
		initTrans.setPosition(PxVec3(0.0f));
		controllers[scene].initialTrans = initTrans;
	}
	controllers[scene].isCamera = isCamera;
	controllers[scene].index = PhysXCharacterManager::nextControllerIndex++;
}

void PhysXCharacterManager::move(const std::string & scene, float time, physx::PxVec3 gravity) {
	if (controllers.find(scene) != controllers.end()) {
		PxController * controller = manager->getController(controllers[scene].index);
		if (controllers[scene].pace > 0.0) {
			PxVec3 movement = *controllers[scene].direction + gravity;
			controller->move(movement, controllers[scene].pace, time, NULL);

			PxTransform trans;
			if (controllers[scene].isCamera) {
				trans = PxTransform(PxMat44(controllers[scene].extInfo.transform));
				trans.transform(PxVec3((PxReal)controller->getPosition().x, (PxReal)controller->getPosition().y, (PxReal)controller->getPosition().z));
			}
			else {
				PxVec3 up = controller->getUpDirection();
				PxVec3 dir = controllers[scene].direction->getNormalized();
				PxVec3 right = up.cross(dir).getNormalized();
				//PxVec3 right = controller->getUpDirection().cross(dir).getNormalized();
				//PxVec3 up = right.cross(dir).getNormalized();

				PxMat44 mat = PxMat44(
					PxVec3(right.x, up.x, dir.x),
					PxVec3(right.y, up.y, dir.y),
					PxVec3(right.z, up.z, dir.z),
					PxVec3((PxReal)controller->getPosition().x, (PxReal)controller->getPosition().y, (PxReal)controller->getPosition().z)
				);
				trans = PxTransform(mat * controllers[scene].initialTrans);
			}
			getMatFromPhysXTransform(trans, controllers[scene].extInfo.transform);
		}
	}
}

void PhysXCharacterManager::createInfo(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	controllers[scene].extInfo = externalInfo(nbVertices, vertices, nbIndices, indices, transform);
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
