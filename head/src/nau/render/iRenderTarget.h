#ifndef RENDERTARGET_H
#define RENDERTARGET_H

#include "nau/attribute.h"
#include "nau/attributeValues.h"
#include "nau/material/iTexture.h"
#include "nau/render/iRenderTarget.h"

#include <string>
#include <vector>

using namespace nau::material;

namespace nau
{
	namespace render
	{
		class IRenderTarget: public AttributeValues
		{
		public:

			UINT_PROP(SAMPLES, 0);
			UINT_PROP(LAYERS, 1);
			UINT_PROP(LEVELS, 2);

			UINT2_PROP(SIZE, 0);

			FLOAT4_PROP(CLEAR_VALUES, 0);

			static AttribSet Attribs;

		protected:
			unsigned int m_Id; 
			unsigned int m_Color; // number of color targets;
			unsigned int m_Depth;
			unsigned int m_Stencil;
			//unsigned int m_Samples;
			//unsigned int m_Layers;
			//unsigned int m_Width;
			//unsigned int m_Height;
			std::string m_Name;
			std::vector<ITexture*> m_TexId;
			ITexture *m_DepthTexture;
			ITexture *m_StencilTexture;

			// clear values per channel
			//nau::math::vec4 m_ClearValues;


		public:

			static IRenderTarget* Create (std::string name, unsigned int width, unsigned int height);
			static IRenderTarget* Create (std::string name);

			virtual bool checkStatus() = 0;
			virtual void resize() = 0;

			virtual void bind (void) = 0;
			virtual void unbind (void) = 0;

			virtual void addColorTarget (std::string name, std::string internalFormat) = 0;
			virtual void addDepthTarget (std::string name, std::string internalFormat) = 0;
			virtual void addStencilTarget (std::string name) = 0;
			virtual void addDepthStencilTarget(std::string name) = 0;

			nau::material::ITexture* getTexture(unsigned int i);
			nau::material::ITexture* getDepthTexture();
			nau::material::ITexture* getStencilTexture();

			virtual void setPropui2(UInt2Property prop, uivec2 &value) = 0;

			virtual unsigned int getNumberOfColorTargets();
			virtual int getId (void);
			virtual std::string &getName (void);

			virtual ~IRenderTarget(void) {};

		protected:
			IRenderTarget();
			static bool Init();
			static bool Inited;

		};
	};
};

#endif 
