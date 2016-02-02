#ifndef OCTREEBYMATNODE_H
#define OCTREEBYMATNODE_H

#include <vector>

#include "nau/scene/sceneObject.h"
#include "nau/scene/camera.h"
#include "nau/material/materialGroup.h"
#include "nau/render/iRenderable.h"
#include "nau/geometry/boundingBox.h"
#include "nau/geometry/mesh.h"
#include "nau/geometry/frustum.h"
#include "nau/math/vec3.h"


namespace nau {
	namespace loader {
		class CBOLoader;
	};
};

namespace nau
{
	namespace scene
	{

		class OctreeByMatNode //: public SceneObject
		{
			friend class nau::loader::CBOLoader;
			friend class OctreeByMat;
			
		protected:
			
			static const int MAXPRIMITIVES = 25000;

			enum {
			  TOPFRONTLEFT = 0,
			  TOPFRONTRIGHT,
			  TOPBACKLEFT,
			  TOPBACKRIGHT,
			  BOTTOMFRONTLEFT,
			  BOTTOMFRONTRIGHT,
			  BOTTOMBACKLEFT,
			  BOTTOMBACKRIGHT,
			  ROOT
			};

			std::shared_ptr<OctreeByMatNode> m_pParent;
			std::shared_ptr<OctreeByMatNode> m_pChilds[8];

			int m_ChildCount;
			bool m_Divided;
			int m_NodeId;
			int m_NodeDepth;

			std::map<std::string, std::shared_ptr<SceneObject>> m_pLocalMeshes;

		public:
			OctreeByMatNode ();
			
			OctreeByMatNode (OctreeByMatNode* parent, vec3 bbMin, vec3 bbMax, int nodeId = 0, int nodeDepth = 0);
			void updateNodeTransform(nau::math::mat4 &t);
			void setRenderable (std::shared_ptr<nau::render::IRenderable> &renderable);

			void getMaterialNames(std::set<std::string> *nameList);

			virtual std::string getType (void);
			void setName(std::string name);
			std::string getName();

			//virtual void writeSpecificData (std::fstream &f);
			//virtual void readSpecificData (std::fstream &f);
			void tightBoundingVolume();
			void unitize(vec3 &center, vec3 &min, vec3 &max);
			
			virtual ~OctreeByMatNode(void);


		protected:
			std::string m_Name;
			void _compile (void);
			void _findVisibleSceneObjects (std::vector<std::shared_ptr<SceneObject>> *m_vReturnVector,
																nau::geometry::Frustum &aFrustum, 
																nau::scene::Camera &aCamera,
																bool conservative = false);
			void getAllSceneObjects (std::vector<std::shared_ptr<SceneObject>> *m_vReturnVector);

			nau::geometry::BoundingBox m_BoundingVolume, m_TightBoundingVolume;

			std::shared_ptr<OctreeByMatNode> &_getChild (int i);
			void _setParent (std::shared_ptr<OctreeByMatNode> &parent);
			void _setChild (int i, std::shared_ptr<OctreeByMatNode> &aNode);
			int _getChildCount (void);

		private:
			int _octantFor (VertexData::Attr& v);
			std::shared_ptr<OctreeByMatNode> _createChild (int octant);
			std::string _genOctName (void);
			void _split();
			void _unifyLocalMeshes();

		};
	};
};
#endif //COCTREENODE_H
