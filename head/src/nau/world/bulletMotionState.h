#ifndef NAUBULLETMOTIONSTATE_H
#define NAUBULLETMOTIONSTATE_H

#include <btBulletDynamicsCommon.h>

#include "nau/scene/iScene.h"

namespace nau
{
	namespace world
	{

		class NauBulletMotionState :
			public btMotionState
		{
		private:
			std::shared_ptr<nau::scene::IScene> m_IScene;
			nau::math::mat4 m_Matrix;

		public:
			NauBulletMotionState(std::shared_ptr<nau::scene::IScene> &aScene);
			~NauBulletMotionState(void);

			/*btMotionState interface*/
			void getWorldTransform (btTransform &worldTrans) const;
			void setWorldTransform (const btTransform &worldTrans);
		};
	};
};

#endif //NAUBULLETMOTIONSTATE_H
