#include "nau/world/bulletDebugger.h"

nau::world::BulletDebugger::BulletDebugger(void) {

}

nau::world::BulletDebugger::~BulletDebugger(void) {

}

void nau::world::BulletDebugger::drawLine(const btVector3 & from, const btVector3 & to, const btVector3 & fromColor, const btVector3 & toColor) {
	glPushMatrix();
	glBegin(GL_LINES);
	glColor3f(0.0f,1.0f,0.0f);
	glVertex3fv(from);
	glVertex3fv(to);
	glEnd();
	glPopMatrix();
}

void nau::world::BulletDebugger::drawLine(const btVector3 & from, const btVector3 & to, const btVector3 & color) {
	glPushMatrix();
	glBegin(GL_LINES);
	//glColor3fv(color);
	glColor3f(0.0f, 1.0f, 0.0f);
	glVertex3fv(from);
	glVertex3fv(to);
	glEnd();
	glPopMatrix();
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
