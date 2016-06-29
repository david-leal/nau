#include "nauPhysXInterface.h"

#include <memory>

static char className[] = "Dummy";
static NauPhysXInterface * Instance = NULL;

__declspec(dllexport)
void *
createPhysics() {

	Instance = new NauPhysXInterface();
	return Instance;
}

__declspec(dllexport)
void
deletePhysics() {

	delete Instance;
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

NauPhysXInterface * NauPhysXInterface::Create() {
	return new NauPhysXInterface();
}

NauPhysXInterface::NauPhysXInterface() {
	//INFO: Declare Physics Properties reflected in XML file
	m_GlobalProps["GRAVITY"] = Prop(IPhysics::VEC4, 0.0f, -9.8f, 0.0f, 0.0f);

	m_MaterialProps["MASS"]				= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["STATIC_FRICTION"]	= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["DYNAMIC_FRICTION"]	= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["RESTITUTION"]		= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["PACE"]				= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["HIT_MAGNITUDE"]	= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["HEIGHT"]			= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["RADIUS"]			= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["STEP_OFFSET"]		= Prop(IPhysics::FLOAT, 1.0f);
	m_MaterialProps["DIRECTION"]		= Prop(IPhysics::VEC4, 0.0f, 0.0f, -1.0f, 1.0f);
	
	worldManager = new PhysXWorldManager();
	Prop p = m_GlobalProps["GRAVITY"];
	worldManager->setGravity(p.x, p.y, p.z);
}


NauPhysXInterface::~NauPhysXInterface() {
}

void NauPhysXInterface::setPropertyManager(nau::physics::IPhysicsPropertyManager * pm) {
	m_PropertyManager = pm;
}

void NauPhysXInterface::update() {
	worldManager->update();
	for (auto particleMaterial : *worldManager->getMaterialParticleNb()) {
		m_PropertyManager->setMaterialFloatProperty(particleMaterial.first, "NBPARTICLES", particleMaterial.second);
	}
}

void NauPhysXInterface::build() {
	
}

void NauPhysXInterface::setSceneType(const std::string & scene, SceneType type) {
	m_Scenes[scene].sceneType = type;
}

void NauPhysXInterface::applyFloatProperty(const std::string & scene, const std::string & property, float value) {
	if (m_Scenes[scene].sceneType == SceneType::RIGID || m_Scenes[scene].sceneType == SceneType::STATIC) {
		worldManager->setRigidProperty(scene, property, value);
	}
	else if (m_Scenes[scene].sceneType == SceneType::CLOTH) {
		worldManager->setSoftProperty(scene, property, value);
	}
	else if (m_Scenes[scene].sceneType == SceneType::CHARACTER) {
		worldManager->setCharacterProperty(scene, property, value);
	}
}

void NauPhysXInterface::applyVec4Property(const std::string & scene, const std::string & property, float * value) {
	if (m_Scenes[scene].sceneType == SceneType::CHARACTER) {
		worldManager->setCharacterProperty(scene, property, value);
	}
}

void NauPhysXInterface::applyGlobalFloatProperty(const std::string & property, float value) {
}

void NauPhysXInterface::applyGlobalVec4Property(const std::string & property, float * value) {
	if (property.compare("GRAVITY") == 0) {
		worldManager->setGravity(value[0], value[1], value[2]);
	}
}

void NauPhysXInterface::setScene(const std::string &scene, const std::string &material, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform) {
	m_Scenes[scene].vertices = vertices;
	m_Scenes[scene].indices = indices;
	m_Scenes[scene].transform = transform;
	switch (m_Scenes[scene].sceneType)
	{
	case IPhysics::STATIC:
		worldManager->addRigid(
			scene,
			nbVertices,
			vertices,
			nbIndices,
			indices,
			transform,
			m_PropertyManager->getMaterialFloatProperty(material, "STATIC_FRICTION"),
			m_PropertyManager->getMaterialFloatProperty(material, "DYNAMIC_FRICTION"),
			m_PropertyManager->getMaterialFloatProperty(material, "RESTITUTION"),
			true
		);
		break;
	case IPhysics::RIGID:
		worldManager->addRigid(
			scene,
			nbVertices,
			vertices,
			nbIndices,
			indices,
			transform,
			m_PropertyManager->getMaterialFloatProperty(material, "STATIC_FRICTION"),
			m_PropertyManager->getMaterialFloatProperty(material, "DYNAMIC_FRICTION"),
			m_PropertyManager->getMaterialFloatProperty(material, "RESTITUTION")
		);
		break;
	case IPhysics::CLOTH:
		worldManager->addCloth(scene, nbVertices, vertices, nbIndices, indices, transform);
		break;
	case IPhysics::CHARACTER:
		worldManager->addCharacter(scene, nbVertices, vertices, nbIndices, indices, transform);
	case IPhysics::PARTICLES:
		worldManager->addParticles(
			scene,
			material,
			m_PropertyManager->getMaterialFloatProperty(material, "MAX_PARTICLES"),
			vertices,
			transform
		);
	default:
		break;
	}
}

float * NauPhysXInterface::getSceneTransform(const std::string & scene) {
	return m_Scenes[scene].transform;
}

void NauPhysXInterface::setSceneTransform(const std::string & scene, float * transform) {
	m_Scenes[scene].transform = transform;
	if (m_Scenes[scene].sceneType == SceneType::RIGID || m_Scenes[scene].sceneType == SceneType::STATIC) {
		worldManager->moveRigid(scene, transform);
	}
	else if (m_Scenes[scene].sceneType == SceneType::CLOTH) {
		worldManager->moveSoft(scene, transform);
	}
	else if (m_Scenes[scene].sceneType == SceneType::CHARACTER) {
		worldManager->moveCharacter(scene, transform);
	}
}

std::vector<float> * NauPhysXInterface::getDebug() {
	return nullptr;
}

std::map<std::string, nau::physics::IPhysics::Prop>& NauPhysXInterface::getGlobalProperties() {
	return m_GlobalProps;
}

std::map<std::string, nau::physics::IPhysics::Prop>& NauPhysXInterface::getMaterialProperties() {
	return m_MaterialProps;
}
