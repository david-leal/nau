#ifndef CBOLOADER_H
#define CBOLOADER_H

#include "nau/scene/iScene.h"
#include "nau/geometry/vertexData.h"
#include "nau/material/material.h"
#include "nau/scene/octreeByMatScene.h"

using namespace nau::render;
using namespace nau::material;
using namespace nau::scene;

namespace nau 
{

	namespace loader 
	{
		class CBOLoader
		{
		public:	
			static void loadScene (nau::scene::IScene *aScene, std::string &aFilename, std::string &params);
			static void writeScene (nau::scene::IScene *aScene, std::string &aFilename);

		private:
			CBOLoader(void) {};
			~CBOLoader(void) {};

			static std::string m_FileName;

			static void _writeMaterial(std::string matName, std::string path, std::fstream &f);
			static void _readMaterial(std::string path, std::fstream &f);
			static void _writeVertexData (std::shared_ptr<VertexData>& aVertexData, std::fstream &f) ;
			static void _readVertexData (std::shared_ptr<VertexData>& aVertexData, std::fstream &f);
			static void _writeIndexData (std::shared_ptr<IndexData>& aVertexData, std::fstream &f) ;
			static void _readIndexData (std::shared_ptr<IndexData>& anIndexData, std::fstream &f);
			//static void _ignoreVertexData (std::fstream &f);
			static void _writeString (const std::string& aString, std::fstream &f);
			static void _readString ( char *buffer, std::fstream &f);
			static void _ignoreString (std::fstream &f);

			static void _writeOctreeByMat(OctreeByMatScene *aScene, std::fstream &f);
			static void _writeOctreeByMatNode(std::shared_ptr<OctreeByMatNode> &n, std::fstream &f)	;
			static void _writeOctreeByMatSceneObject(std::shared_ptr<SceneObject> &so, std::fstream &f)	;

			static void _readOctreeByMat(OctreeByMatScene *aScene, std::fstream &f);
			static void _readOctreeByMatNode(std::shared_ptr<OctreeByMatNode> &n, std::fstream &f)	;
			static void _readOctreeByMatSceneObject(std::shared_ptr<SceneObject> &so, std::fstream &f)	;
		};
	};
};

#endif //CBOLOADER_H
