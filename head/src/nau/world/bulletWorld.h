#ifndef BULLETWORLD_H
#define BULLETWORLD_H

#include <btBulletDynamicsCommon.h>
#include "nau/world/iWorld.h"
#include "nau/scene/iScene.h"
#include "nau/material/iBuffer.h"
#include "nau/render/pass.h"
#include "BulletCollision/Gimpact/btGImpactShape.h"
#include "BulletCollision/Gimpact/btGImpactCollisionAlgorithm.h"

namespace nau
{
	namespace world
	{
		class BulletWorld :
			public nau::world::IWorld
		{
		private:
			static const int maxProxies = 32766;

			nau::scene::IScene *m_pScene;
			btDynamicsWorld *m_pDynamicsWorld;

			std::map <std::string, btRigidBody*> m_RigidBodies;

		public:
			BulletWorld(void);

			void update (void);
			void build (void);
			void setScene (nau::scene::IScene *aScene);

			void _addRigid (float mass, float friction, float restitution, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec);
			void _addCloth(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec);
			void _addParticles(nau::render::Pass* pass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::material::IBuffer* positions);
			void _addCharacter(float mass, float radius, float height, float stepHeight, std::shared_ptr<nau::scene::IScene> &aScene, std::string name);

			void setKinematic (std::string name);
			void setDynamic (std::string name);

			void disableObject (std::string name);
			void enableObject (std::string name);

			void setVelocity (std::string name, nau::math::vec3 vel);

			void setDebug(nau::scene::IScene* debugScene = 0, nau::material::IBuffer* debugPositions = 0);
		
		public:
			~BulletWorld(void);
		};
	};
};
#endif
