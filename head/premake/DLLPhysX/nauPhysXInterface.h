#ifndef _NAUPHYSXINTERFACE_H
#define _NAUPHYSXINTERFACE_H

#include "nau/physics/iPhysics.h"

#include <map>
#include <string>

#include "physXWorldManager.h"

class NauPhysXInterface : public nau::physics::IPhysics {

private:
	PhysXWorldManager * worldManager;

public:

	static NauPhysXInterface *Create();
	NauPhysXInterface();
	~NauPhysXInterface();

	void update();
	void build();

	void setSceneType(const std::string &scene, SceneType type);

	void applyFloatProperty(const std::string &scene, const std::string &property, float value);
	void applyVec4Property(const std::string &scene, const std::string &property, float *value);

	void applyGlobalFloatProperty(const std::string &property, float value);
	void applyGlobalVec4Property(const std::string &property, float *value);

	void setScene(const std::string &scene, int nbVertices, float *vertices, int nbIndices, unsigned int *indices, float *transform);

	float *getSceneTransform(const std::string &scene);
	void setSceneTransform(const std::string &scene, float *transform);

	int getParticleCount(const std::string &scene);

	void setDebug(std::vector<float> * debugPoint);

	std::map<std::string, nau::physics::IPhysics::Prop> &getGlobalProperties();
	std::map<std::string, nau::physics::IPhysics::Prop> &getMaterialProperties();
};

extern "C" {

	__declspec(dllexport) void *createPhysics();
	__declspec(dllexport) void init();
	__declspec(dllexport) char *getClassName();
}

#endif