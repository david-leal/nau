#ifndef PROJECTLOADER_H
#define PROJECTLOADER_H

#include <string>
#include <vector>

#include <tinyxml.h>
#include <nau/system/fileutil.h>
#include <nau/render/pass.h>
#include <nau/material/materialLib.h>

using namespace nau::render;
using namespace nau::material;

namespace nau
{
	namespace loader
	{
		class ProjectLoader
		{
		public:
			static void load (std::string file, int *width, int *height, 
												bool *tangents, bool *triangleIDs);
			static void loadMatLib (std::string file);
			static std::string s_Path;
			static std::string s_File;

		private:
			ProjectLoader(void);


			static void loadAssets (TiXmlHandle &hRoot, std::vector<std::string> &matLib);
			static void loadPipelines (TiXmlHandle &hRoots);

			static void *readAttr(std::string parent, TiXmlElement *pElem, Enums::DataType type, AttribSet attribs);
			// Asset Loading
			static void loadUserAttrs(TiXmlHandle handle);
			static void loadScenes(TiXmlHandle handle);
			static void loadGeometry(TiXmlElement *elem); 
			static void loadViewports(TiXmlHandle handle);
			static void loadCameras(TiXmlHandle handle);
			static void loadLights(TiXmlHandle handle);
			static void loadEvents(TiXmlHandle handle);
			static void loadAtomicSemantics(TiXmlHandle handle);

			//Pass Elements
			static void loadPassCamera(TiXmlHandle hPass, Pass *aPass); 	
			static void loadPassLights(TiXmlHandle hPass, Pass *aPass); 	
			static void loadPassScenes(TiXmlHandle hPass, Pass *aPass);
			static void loadPassClearDepthAndColor(TiXmlHandle hPass, Pass *aPass);
			static void loadPassViewport(TiXmlHandle hPass, Pass *aPass);
			static void loadPassParams(TiXmlHandle hPass, Pass *aPass);
			static void loadPassRenderTargets(TiXmlHandle hPass, Pass *aPass, std::map<std::string, Pass*> passMapper);
			static void loadPassTexture(TiXmlHandle hPass, Pass *aPass);
			static void loadPassMaterialMaps(TiXmlHandle hPass, Pass *aPass);
			static void loadPassInjectionMaps(TiXmlHandle hPass, Pass *aPass);
#ifdef NAU_OPTIX
			static void loadPassOptixSettings(TiXmlHandle hPass, Pass *aPass);
#endif
#ifdef NAU_OPTIX_PRIME
			static void loadPassOptixPrimeSettings(TiXmlHandle hPass, Pass *aPass);
#endif
			static void loadPassComputeSettings(TiXmlHandle hPass, Pass *aPass);
			//static void loadPassShaderMaps(TiXmlHandle hPass, Pass *aPass);
			//static void loadPassStateMaps(TiXmlHandle hPass, Pass *aPass);

			static void loadMatLibRenderTargets(TiXmlHandle hPass, MaterialLib *aLib, std::string path);
			static void loadMatLibTextures(TiXmlHandle hPass, MaterialLib *aLib, std::string path);
			static void loadMatLibStates(TiXmlHandle hPass, MaterialLib *aLib);
			static void loadMatLibShaders(TiXmlHandle hPass, MaterialLib *aLib, std::string path);
			static void loadMatLibBuffers(TiXmlHandle hPass, MaterialLib *aLib, std::string path);

			static void loadMaterialColor(TiXmlHandle handle, MaterialLib *aLib, Material *aMat);
			static void loadMaterialTextures(TiXmlHandle handle, MaterialLib *aLib, Material *aMat);
			static void loadMaterialImageTextures(TiXmlHandle handle, MaterialLib *aLib, Material *aMat);
			static void loadMaterialBuffers(TiXmlHandle handle, MaterialLib *aLib, Material *aMat);
			static void loadMaterialShader(TiXmlHandle handle, MaterialLib *aLib, Material *aMat);
			static void loadMaterialState(TiXmlHandle handle, MaterialLib *aLib, Material *aMat);

			static void loadState(TiXmlElement *pElemAux, MaterialLib *aLib, Material *aMat, IState *s);

			// aux pre alocated variable
			static char s_pFullName[256];

			static string s_Dummy;
			static std::string toLower(std::string);

			static vec4 s_Dummy_vec4;
			static bvec4 s_Dummy_bvec4;
			static float s_Dummy_float;
			static int s_Dummy_int;
			static bool s_Dummy_bool;
		};
	};
};

#endif //PROJECTLOADER_H


