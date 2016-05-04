#ifndef IWORLD_H
#define IWORLD_H

#include "nau/scene/iScene.h"
#include "nau/math/vec3.h"
#include "nau/material/iBuffer.h"
#include "nau/render/pass.h"

namespace nau
{
	namespace world
	{
		class IWorld
		{
		public:
		        
		    virtual ~IWorld() {};
		  
	        virtual void update (void) = 0;
			virtual void build (void) = 0;
			virtual void setScene (nau::scene::IScene *aScene) = 0;

			virtual void _addRigid(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) = 0;
			virtual void _addCloth(float mass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::math::vec3 aVec) = 0;
			virtual void _addParticles(nau::render::Pass* pass, std::shared_ptr<nau::scene::IScene> &aScene, std::string name, nau::material::IBuffer* positions) = 0;
			
			virtual void setKinematic (std::string name) = 0;
			virtual void setDynamic (std::string name) = 0;

			virtual void disableObject (std::string name) = 0;
			virtual void enableObject (std::string name) = 0;

			virtual void setVelocity (std::string name, nau::math::vec3 vel) = 0;

			virtual void setDebug(nau::scene::IScene* debugScene = 0, nau::material::IBuffer* debugPositions = 0) = 0;
		};
	};
};
#endif
