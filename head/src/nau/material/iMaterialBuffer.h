#ifndef MATERIALBUFFER_H
#define MATERIALBUFFER_H

#include "nau/attribute.h"
#include "nau/attributeValues.h"
#include "nau/material/iBuffer.h"

#include <string>

using namespace nau;

namespace nau
{
	namespace material
	{
		class IMaterialBuffer : public AttributeValues
		{
		public:

			INT_PROP(BINDING_POINT, 0);
			ENUM_PROP(TYPE, 0);

			static AttribSet Attribs;

			static IMaterialBuffer* Create(IBuffer *b);

			std::string& getLabel(void) {
				return m_Label;
			};

			virtual void bind() = 0;
			virtual void unbind() = 0;
			void setBuffer(IBuffer *b);
			IBuffer *getBuffer();

			virtual ~IMaterialBuffer(void) {};

		protected:

			IMaterialBuffer() { registerAndInitArrays(Attribs); };

			static bool Init();
			static bool Inited;

			std::string m_Label;
			IBuffer *m_Buffer;
		};
	};
};


#endif // IBUFFER_H