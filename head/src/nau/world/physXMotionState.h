#ifndef PHYSXMOTIONSTATE_H
#define PHYSXMOTIONSTATE_H

#include "nau/scene/iScene.h"
#include "PxPhysicsAPI.h"
#include "PxScene.h"

namespace nau
{
	namespace world
	{

		class PhysXMotionState : public physx::PxScene
		{
		private:
			std::shared_ptr<nau::scene::IScene> m_IScene;
			nau::math::mat4 m_Matrix;

		public:
			PhysXMotionState(std::shared_ptr<nau::scene::IScene> &aScene);
			~PhysXMotionState(void);

			//bool fetchResults(bool block = false, physx::PxU32* errorState = 0);
			/*btMotionState interface*/
			//void getWorldTransform(btTransform &worldTrans) const;
			//void setWorldTransform(const btTransform &worldTrans);
		};
	};
};

#endif //PHYSXMOTIONSTATE_H
