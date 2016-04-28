#include "nau/world/bulletDebugger.h"

void nau::world::BulletDebugger::drawLine(const btVector3 & from, const btVector3 & to, const btVector3 & fromColor, const btVector3 & toColor) {
	points->push_back(from.getX());
	points->push_back(from.getY());
	points->push_back(from.getZ());
	points->push_back(1.0f);
	points->push_back(to.getX());
	points->push_back(to.getY());
	points->push_back(to.getZ());
	points->push_back(1.0f);
}

void nau::world::BulletDebugger::drawLine(const btVector3 & from, const btVector3 & to, const btVector3 & color) {
	points->push_back(from.getX());
	points->push_back(from.getY());
	points->push_back(from.getZ());
	points->push_back(1.0f);
	points->push_back(to.getX());
	points->push_back(to.getY());
	points->push_back(to.getZ());
	points->push_back(1.0f);
}

//void nau::world::BulletDebugger::drawSphere(const btVector3 & p, btScalar radius, const btVector3 & color)
//{
//}
//
//void nau::world::BulletDebugger::drawTriangle(const btVector3 & a, const btVector3 & b, const btVector3 & c, const btVector3 & color, btScalar alpha)
//{
//}

void nau::world::BulletDebugger::drawContactPoint(const btVector3 & PointOnB, const btVector3 & normalOnB, btScalar distance, int lifeTime, const btVector3 & color) {
}

void nau::world::BulletDebugger::reportErrorWarning(const char * warningString) {
}

void nau::world::BulletDebugger::draw3dText(const btVector3 & location, const char * textString) {
}

void nau::world::BulletDebugger::setDebugMode(int debugMode) {
	m_debugMode = debugMode;
}

void nau::world::BulletDebugger::compilePoints(void) {
	debugPositions->setData(points->size() * sizeof(float), &points->at(0));
	scene->getSceneObject(0)->getRenderable()->getVertexData()->resetCompilationFlag();
	scene->getSceneObject(0)->getRenderable()->getVertexData()->compile();
}

void nau::world::BulletDebugger::clearPoints(void) {
	points = new std::vector<float>();
}
