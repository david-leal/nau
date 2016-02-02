#ifndef ISCENEPARTITIONED_H
#define ISCENEPARTITIONED_H

#include "nau/scene/iScene.h"
#include "nau/geometry/frustum.h"
#include "nau/geometry/boundingBox.h"

namespace nau {

	namespace scene {

		class IScenePartitioned : public IScene
		{
		protected:

			bool m_Built;

		public:

			IScenePartitioned(): IScene(), m_Built(false) {}

			//virtual bool load (std::string &aSceneFile);
			virtual void build (void) = 0;
			virtual bool isBuilt(void) {return m_Built;};

			~IScenePartitioned (void) { }
		};
	};
};

#endif
