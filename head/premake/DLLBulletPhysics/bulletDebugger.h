#ifndef BULLETDEBUGGER_H
#define BULLETDEBUGGER_H

#include "LinearMath/btIDebugDraw.h"
#include <vector>

class BulletDebugger : public btIDebugDraw {
	
	int m_debugMode;

private:
	std::vector<float> * points;

public:
	BulletDebugger(std::vector<float> * debugPoints) {
		this->points = debugPoints;
	};
	~BulletDebugger() {};

	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& fromColor, const btVector3& toColor);
				 
	virtual void drawLine(const btVector3& from, const btVector3& to, const btVector3& color);

	//virtual void   drawSphere(const btVector3& p, btScalar radius, const btVector3& color);

	//virtual void   drawTriangle(const btVector3& a, const btVector3& b, const btVector3& c, const btVector3& color, btScalar alpha);

	virtual void drawContactPoint(const btVector3& PointOnB, const btVector3& normalOnB, btScalar distance, int lifeTime, const btVector3& color);
				 
	virtual void reportErrorWarning(const char* warningString);
				 
	virtual void draw3dText(const btVector3& location, const char* textString);
				 
	virtual void setDebugMode(int debugMode);

	virtual int getDebugMode() const { return m_debugMode; }

	void setPoints(std::vector<float> * debugPoints) { points = debugPoints; }

};

#endif