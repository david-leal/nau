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
			public nau::world::IWorld
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

			void _add(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec);
			void setKinematic(std::string name);
			void setDynamic(std::string name);

			void disableObject(std::string name);
			void enableObject(std::string name);

			void setVelocity(std::string name, nau::math::vec3 vel);

		public:
			~PhsXWorld(void);
		};
	};
};
#endif