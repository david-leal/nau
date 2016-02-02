#ifndef TEXTURE_H
#define TEXTURE_H

#include "nau/config.h"
#include "nau/attribute.h"
#include "nau/attributeValues.h"
#include "nau/material/iTextureSampler.h"


//#ifdef __SLANGER__
//#include <wx/bitmap.h>
//#include <wx/image.h>
//#include <IL/ilu.h>
//#endif

#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#endif
#include <cmath>
#include <string>


using namespace nau;

namespace nau {

	namespace material {
		class ITextureSampler;
	}
}

namespace nau
{
	namespace material
	{
		class ITexture: public AttributeValues
		{
		public:

			ENUM_PROP(DIMENSION, 0);
			ENUM_PROP(FORMAT, 1);
			ENUM_PROP(TYPE, 2);
			ENUM_PROP(INTERNAL_FORMAT, 3);
			ENUM_PROP(COUNT_ENUMPROPERTY, 4);
			
			INT_PROP(ID, 0);
			INT_PROP(WIDTH, 1);
			INT_PROP(HEIGHT, 2);
			INT_PROP(DEPTH, 3);
			INT_PROP(LEVELS, 4);
			INT_PROP(SAMPLES, 5);
			INT_PROP(LAYERS, 6);
			INT_PROP(COMPONENT_COUNT, 7);
			INT_PROP(ELEMENT_SIZE, 8);

			BOOL_PROP(MIPMAP, 0);

			static AttribSet Attribs;

			//int addAtrib(std::string name, Enums::DataType dt, void *value);

			// Note: no validation is performed!
			//void setProp(int prop, Enums::DataType type, void *value);

			static ITexture* Create (std::string file, std::string label, bool mipmap=true);
			//static ITexture* Create (std::string label);

			//static ITexture* Create(std::string label, std::string internalFormat,
			//	std::string aFormat, std::string aType, int width, int height, 
			//	unsigned char* data );

			static ITexture* Create(std::string label, std::string internalFormat,
				int width, int height, int depth = 1, int layers = 1, int levels = 0, int samples = 0);

			//static ITexture* CreateMS(std::string label, std::string internalFormat,
			//	int width, int height, 
			//	int samples );

			static ITexture* Create(std::string label);
	

//#ifdef __SLANGER__
//			virtual wxBitmap *getBitmap(void);
//#endif
			virtual std::string& getLabel (void);
			virtual void setLabel (std::string label);
			//! prepare a texture for rendering
			virtual void prepare(unsigned int unit, ITextureSampler *ts) = 0;
			//! restore default texture in texture unit
			virtual void restore(unsigned int unit, ITextureSampler *ts) = 0;
			//! builds a texture with the attribute parameters previously set
			virtual void build(int immutable = 0) = 0;

			virtual void clear() = 0;
			virtual void clearLevel(int l) = 0;

			virtual void generateMipmaps() = 0;

			virtual void resize(unsigned int x, unsigned int y, unsigned int z) = 0;

			virtual ~ITexture(void);

		protected:
			// For textures with data, ex. loaded images
			//ITexture(std::string label, std::string aDimension, std::string internalFormat, 
			//	std::string aFormat, std::string aType, int width, int height);
			/// For 2D textures without data, ex texture storage
			//ITexture(std::string label, std::string aDimension, std::string internalFormat, 
			//	int width, int height);

			ITexture(std::string label);

			/// For inheritance reasons only
			ITexture() {/*bitmap=NULL;*/};

			static bool Init();
			static bool Inited;

			std::string m_Label;
//#ifdef __SLANGER__
//			wxBitmap *bitmap;
//#endif
		};
	};
};

#endif
