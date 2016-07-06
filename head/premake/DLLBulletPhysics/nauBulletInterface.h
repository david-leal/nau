#ifndef _NAUBULLETINTERFACE_H
#define _NAUBULLETINTERFACE_H

#include "nau/physics/iPhysics.h"
#include <map>
#include <string>
#include "bulletWorldManager.h"

class NauBulletInterface : public nau::physics::IPhysics {

private:
	BulletWorldManager * worldManager;

public:
	
	static NauBulletInterface * Create();
	NauBulletInterface();
	~NauBulletInterface();

	virtual void setPropertyManager(nau::physics::IPhysicsPropertyManager *pm);
	
	void update();
	void build();

	//void setSceneType(const std::string &scene, SceneType type);

	void applyFloatProperty(const std::string &scene, const std::string &property, float value);
	void applyVec4Property(const std::string &scene, const std::string &property, float *value);

	void applyGlobalFloatProperty(const std::string &property, float value);
	void applyGlobalVec4Property(const std::string &property, float *value);

	void setScene(const std::string &scene, const std::string & material, int nbVertices, float * vertices, int nbIndices, unsigned int * indices, float * transform);

	float *getSceneTransform(const std::string &scene);
	void setSceneTransform(const std::string &scene, float *transform);

	//void setParticleScene(const std::string &scene, float maxParticles, float * nbParticles, float * transform);
	//float * getParticlePositions(const std::string &scene);

	std::vector<float> * getDebug();

	std::map<std::string, nau::physics::IPhysics::Prop> &getGlobalProperties();
	std::map<std::string, nau::physics::IPhysics::Prop> &getMaterialProperties();

};

extern "C" {

	__declspec(dllexport) void *createPhysics();
	__declspec(dllexport) void init();
	__declspec(dllexport) char *getClassName();
	__declspec(dllexport) void deletePhysics();
}

#endif