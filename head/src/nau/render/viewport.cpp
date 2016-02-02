#include "nau/render/viewport.h"

#include "nau.h"

using namespace nau::render;
using namespace nau::math;

bool
Viewport::Init() {

	// VEC2
	Attribs.add(Attribute(ORIGIN, "ORIGIN", Enums::DataType::VEC2, false, new vec2(0.0f, 0.0f), new vec2(0.0f, 0.0f)));
	Attribs.add(Attribute(SIZE, "SIZE", Enums::DataType::VEC2, false, new vec2(1.0f, 1.0f), new vec2(0.0f, 0.0f)));
	Attribs.add(Attribute(ABSOLUT_ORIGIN, "ABSOLUT_ORIGIN", Enums::DataType::VEC2, true, new vec2(0.0f, 0.0f), new vec2(0.0f, 0.0f)));
	Attribs.add(Attribute(ABSOLUT_SIZE, "ABSOLUT_SIZE", Enums::DataType::VEC2, true, new vec2(1.0f, 1.0f), new vec2(0.0f, 0.0f)));

	// VEC4
	Attribs.add(Attribute(CLEAR_COLOR, "CLEAR_COLOR", Enums::DataType::VEC4, false, new vec4(), new vec4(), new vec4(1.0f),IAPISupport::OK, Attribute::COLOR));

	// BOOL
	Attribs.add(Attribute(FULL, "FULL", Enums::DataType::BOOL, false, new NauInt(true)));

	// FLOAT
	Attribs.add(Attribute(RATIO, "RATIO", Enums::DataType::FLOAT, false, new NauFloat(0.0f), new NauFloat(0.0f)));

#ifndef _WINDLL
	NAU->registerAttributes("VIEWPORT", &Attribs);
#endif

	return true;
}



AttribSet Viewport::Attribs;
bool Viewport::Inited = Init();


Viewport::Viewport(void) :
	m_Name("default") {

	registerAndInitArrays(Attribs);
	EVENTMANAGER->addListener("WINDOW_SIZE_CHANGED", this);
}


Viewport::~Viewport(void) {

	EventManager *ev = EVENTMANAGER;
	if (ev)
		ev->removeListener("WINDOW_SIZE_CHANGED",this);
}


void 
Viewport::setName(std::string aName) {

	m_Name = aName;
}


std::string&
Viewport::getName() { 

	return m_Name;
}


void
Viewport::setPropb(BoolProperty prop, bool value) {

	if (prop == FULL) {
		m_BoolProps[prop] = value;

		if (value == true) {
			m_Float2Props[SIZE] = vec2((float)NAU->getWindowWidth(), (float)NAU->getWindowHeight());
			m_Float2Props[ABSOLUT_SIZE] = vec2((float)NAU->getWindowWidth(), (float)NAU->getWindowHeight());
			m_Float2Props[ORIGIN] = vec2(0.0f, 0.0f);
			m_Float2Props[ABSOLUT_ORIGIN] = vec2(0.0f, 0.0f);
			m_FloatProps[RATIO] = 0;
		}
	}
}


void
Viewport::setPropf(FloatProperty prop, float value) {


	switch (prop) {
	case RATIO:
		// if ratio is bigger than zero
		if (value > 0.0f) {
			m_FloatProps[prop] = value;
			setPropf2(SIZE, m_Float2Props[SIZE]);
			setPropf2(ORIGIN, m_Float2Props[ORIGIN]);
		}
		break;
	default:
		AttributeValues::setPropf(prop, value);
	}
}


float
Viewport::getPropf(FloatProperty prop) {

	switch(prop) {
	
		case RATIO:
			if (m_Float2Props[ABSOLUT_SIZE].y != 0)
				return (m_Float2Props[ABSOLUT_SIZE].x / m_Float2Props[ABSOLUT_SIZE].y);
			else
				return 0;
		default:
			return AttributeValues::getPropf(prop);
	}
}


void
Viewport::setPropf2(Float2Property prop, vec2& values) {

	float width = values.x;
	float height = values.y;
	
	switch(prop) {
		case SIZE:
		case ABSOLUT_SIZE:
			m_Float2Props[SIZE] = values;
			if (m_FloatProps[RATIO] > 0.0f)
				m_Float2Props[SIZE].y = m_Float2Props[SIZE].x * m_FloatProps[RATIO];

			m_BoolProps[FULL] = false;

			if (width <=1) {
				m_Float2Props[ABSOLUT_SIZE].x = width * NAU->getWindowWidth();
			}
			else {
				m_Float2Props[ABSOLUT_SIZE].x = width;
			}

			if (height <= 1) {
				m_Float2Props[ABSOLUT_SIZE].y = height * NAU->getWindowHeight();
			}
			else {
				m_Float2Props[ABSOLUT_SIZE].y = height;
			}

			if (m_FloatProps[RATIO] > 0) {
				m_Float2Props[ABSOLUT_SIZE].y = m_Float2Props[ABSOLUT_SIZE].x * m_FloatProps[RATIO];
			}
			break;
		case ORIGIN:
		case ABSOLUT_ORIGIN:
			m_BoolProps[FULL] = false;
			m_Float2Props[ORIGIN] = values;

			if (values.x < 1) {
				m_Float2Props[ABSOLUT_ORIGIN].x = NAU->getWindowWidth() * values.x;
			}
			else {
				m_Float2Props[ABSOLUT_ORIGIN].x = values.x;
			}

			if (values.y < 1) {
				m_Float2Props[ABSOLUT_ORIGIN].y = NAU->getWindowHeight() * values.y;
			}
			else {
				m_Float2Props[ABSOLUT_ORIGIN].y = values.y;
			}
			break;
		default:
			AttributeValues::setPropf2(prop, values);
	}
}


void
Viewport::eventReceived(const std::string &sender, const std::string &eventType, 
	const std::shared_ptr<IEventData> &evtData) {

	if (eventType != "WINDOW_SIZE_CHANGED")
		return;

	vec3 *ev = (vec3 *)(evtData->getData());

	if (m_BoolProps[FULL]) {
	
		m_Float2Props[ABSOLUT_SIZE].set(ev->x, ev->y);
		m_Float2Props[SIZE].set(ev->x, ev->y);

	}
	else {
		if (m_Float2Props[ORIGIN].x <= 1) 
			m_Float2Props[ABSOLUT_ORIGIN].x = ev->x * m_Float2Props[ORIGIN].x;

		if (m_Float2Props[ORIGIN].y <= 1)
			m_Float2Props[ABSOLUT_ORIGIN].y = ev->y * m_Float2Props[ORIGIN].y;

		if (m_Float2Props[SIZE].x <= 1 )
			m_Float2Props[ABSOLUT_SIZE].x = ev->x * m_Float2Props[SIZE].x;

		if (m_Float2Props[SIZE].y <= 1 )
			m_Float2Props[ABSOLUT_SIZE].y = ev->y * m_Float2Props[SIZE].y;

		if (m_FloatProps[RATIO] > 0)
			m_Float2Props[ABSOLUT_SIZE].y = m_Float2Props[ABSOLUT_SIZE].x * m_FloatProps[RATIO];
	}
	EVENTMANAGER->notifyEvent("VIEWPORT_CHANGED", m_Name, "", NULL); 
}


