#ifndef GLAPI_SUPPORT_H
#define GLAPI_SUPPORT_H

#include "nau/render/IAPISupport.h"

using namespace nau::render;

namespace nau
{
	namespace render
	{

		class GLAPISupport : public IAPISupport
		{

		public:
			void setAPISupport();

		};
	};
};

#endif
