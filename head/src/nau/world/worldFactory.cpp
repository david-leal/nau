#include "nau/world/worldFactory.h"

#include "nau/world/bulletWorld.h"
#include "nau/world/physXWorld.h"

using namespace nau::world;

nau::world::IWorld* 
WorldFactory::create (std::string type)
{
	if ("Bullet" == type) {
		return new BulletWorld;
	}
	if ("PhysX" == type) {
		return new PhsXWorld;
	}

	return 0;
}
