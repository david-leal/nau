#ifndef PHYSXWORLD_H
#define PHYSXWORLD_H

#include "nau/world/iWorld.h"
#include "nau/scene/iScene.h"
#include "PxPhysicsAPI.h"

namespace nau
{
	namespace world
	{
		class PhsXWorld :
			public nau::world::IWorld , public physx::PxUserControllerHitReport
		{
		private:
			static const int maxProxies = 32766;

			nau::scene::IScene *m_pScene;
			physx::PxScene *m_pDynamicsWorld;

			std::map <std::string, physx::PxActor*> m_RigidBodies;

		public:
			PhsXWorld(void);

			void update(void);
			void build(void);
			void setScene(nau::scene::IScene *aScene);

			void _addRigid(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec);
			void _addCloth(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec);
			void _addParticles(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec);
			void setKinematic(std::string name);
			void setDynamic(std::string name);

			void disableObject(std::string name);
			void enableObject(std::string name);

			void setVelocity(std::string name, nau::math::vec3 vel);
			void onShapeHit(const physx::PxControllerShapeHit &hit);
			void onControllerHit(const physx::PxControllersHit &hit);
			void onObstacleHit(const physx::PxControllerObstacleHit &hit);

			~PhsXWorld(void);
		};
	};
};
#endif
