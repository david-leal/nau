#include "nauBulletInterface.h"

#include <memory>

static char className[] = "Dummy";

__declspec(dllexport)
void *
createPhysics() {

	NauBulletInterface *p = new NauBulletInterface();
	return p;
}


__declspec(dllexport)
void
init() {

}


__declspec(dllexport)
char *
getClassName() {

	return className;
}



NauBulletInterface * NauBulletInterface::Create() {
	return new NauBulletInterface();
}

NauBulletInterface::NauBulletInterface() {
	//INFO: Declare Physics Properties reflected in XML file
	m_GlobalProps["GRAVITY"] = Prop(IPhysics::VEC4, 0.0f, -9.8f, 0.0f, 0.0f);

	m_MaterialProps["MASS"] = Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["FRICTION"] = Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["RESTITUTION"] = Prop(IPhysics::FLOAT, 1.0f);

	worldManager = new BulletWorldManager();
	Prop p = m_GlobalProps["GRAVITY"];
	worldManager->setGravity(p.x, p.y, p.z);
}


NauBulletInterface::~NauBulletInterface() {
}

void NauBulletInterface::update() {
	worldManager->update();
}

void NauBulletInterface::build() {
}

void NauBulletInterface::setSceneType(const std::string & scene, SceneType type) {
	m_Scenes[scene].sceneType = type;
}

void NauBulletInterface::applyFloatProperty(const std::string & scene, const std::string & property, float value) {
	if (m_Scenes[scene].sceneType == SceneType::RIGID || m_Scenes[scene].sceneType == SceneType::STATIC) {
		worldManager->setRigidProperty(scene, property, value);
	}
	if (m_Scenes[scene].sceneType == SceneType::CLOTH) {
		worldManager->setSoftProperty(scene, property, value);
	}
}

void NauBulletInterface::applyVec4Property(const std::string & scene, const std::string & property, float * value) {
}

void NauBulletInterface::applyGlobalFloatProperty(const std::string & property, float value) {
}

void NauBulletInterface::applyGlobalVec4Property(const std::string & property, float * value) {
	if (property.compare("GRAVITY") == 0) {
		worldManager->setGravity(value[0], value[1], value[2]);
	}
}

void NauBulletInterface::setScene(const std::string & scene, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform) {
	m_Scenes[scene].vertices = vertices;
	m_Scenes[scene].indices = indices;
	m_Scenes[scene].transform = transform;
	switch (m_Scenes[scene].sceneType)
	{
	case IPhysics::STATIC:
		worldManager->addRigid(scene, nbVertices, vertices, nbIndices, indices, transform, true);
		break;
	case IPhysics::RIGID:
		worldManager->addRigid(scene, nbVertices, vertices, nbIndices, indices, transform);
		break;
	case IPhysics::CLOTH:
		worldManager->addCloth(scene, nbVertices, vertices, nbIndices, indices, transform);
		break;
	default:
		break;
	}
}

float * NauBulletInterface::getSceneTransform(const std::string & scene) {
	return m_Scenes[scene].transform;
}

void NauBulletInterface::setSceneTransform(const std::string & scene, float * transform) {
	m_Scenes[scene].transform = transform;
	if (m_Scenes[scene].sceneType == SceneType::RIGID || m_Scenes[scene].sceneType == SceneType::STATIC) {
		worldManager->moveRigid(scene, transform);
	}
	if (m_Scenes[scene].sceneType == SceneType::CLOTH) {
		worldManager->moveSoft(scene, transform);
	}
}

std::map<std::string, nau::physics::IPhysics::Prop>& NauBulletInterface::getGlobalProperties() {
	return m_GlobalProps;
}

std::map<std::string, nau::physics::IPhysics::Prop>& NauBulletInterface::getMaterialProperties() {
	return m_MaterialProps;
}
