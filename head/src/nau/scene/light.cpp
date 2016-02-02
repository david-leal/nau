#include "nau/scene/light.h"

#include "nau.h"
using namespace nau;


bool
Light::Init() {

	// VEC4
	Attribs.add(Attribute(POSITION, "POSITION", Enums::DataType::VEC4, false, new vec4(0.0f, 0.0f, 0.0f, 1.0f)));
	Attribs.add(Attribute(DIRECTION, "DIRECTION", Enums::DataType::VEC4, false, new vec4(0.0f, 0.0f, -1.0f, 0.0f)));
	Attribs.add(Attribute(SPOT_DIRECTION, "SPOT_DIRECTION", Enums::DataType::VEC4, false, new vec4(0.0f, 0.0f, -1.0f, 0.0f)));
	Attribs.add(Attribute(NORMALIZED_DIRECTION, "NORMALIZED_DIRECTION", Enums::DataType::VEC4,true, new vec4(0.0f, 0.0f, -1.0f, 0.0f)));
	Attribs.add(Attribute(COLOR, "COLOR", Enums::DataType::VEC4, false, new vec4(1.0f, 1.0f, 1.0f, 1.0f)));
	Attribs.add(Attribute(AMBIENT, "AMBIENT", Enums::DataType::VEC4, false, new vec4(0.2f, 0.2f, 0.2f, 1.0f)));
	// FLOAT
	Attribs.add(Attribute(SPOT_EXPONENT, "SPOT_EXPONENT", Enums::DataType::FLOAT, false, new NauFloat(0.0f)));
	Attribs.add(Attribute(SPOT_CUTOFF, "SPOT_CUTOFF", Enums::DataType::FLOAT, false, new NauFloat(180.0f)));
	Attribs.add(Attribute(CONSTANT_ATT, "CONSTANT_ATT", Enums::DataType::FLOAT, false, new NauFloat(1.0f)));
	Attribs.add(Attribute(LINEAR_ATT, "LINEAR_ATT", Enums::DataType::FLOAT,false, new NauFloat(0.0f)));
	Attribs.add(Attribute(QUADRATIC_ATT, "QUADRATIC_ATT", Enums::DataType::FLOAT,false, new NauFloat(0.0)));
	// BOOL
	Attribs.add(Attribute(ENABLED, "ENABLED", Enums::DataType::BOOL, false, new NauInt(true)));
	//INT
	Attribs.add(Attribute(ID, "ID", Enums::DataType::INT,true, new NauInt(-1)));

#ifndef _WINDLL
	NAU->registerAttributes("LIGHT", &Attribs);
#endif

	return true;
}


AttribSet Light::Attribs;
bool Light::Inited = Init();


Light::Light (std::string &name) {

	m_Name = name;

	registerAndInitArrays(Attribs);
}


Light::~Light(void) {

}


void 
Light::eventReceived(const std::string &sender,
	const std::string &eventType,
	const std::shared_ptr<IEventData> &evt) {

}


std::string
Light::getType() {

	return("LIGHT");
}


void
Light::setPropf(FloatProperty prop, float value) {

	float final = value;

	switch (prop) {
		case CONSTANT_ATT:
		case LINEAR_ATT:
		case QUADRATIC_ATT:
			if (value < 0)
				final = 0;
			m_FloatProps[prop] = final;
			break;
		default:
			AttributeValues::setPropf(prop, value);
	}
}


void
Light::setPropf4(Float4Property prop, float x, float y, float z, float w){

	vec4 v;

	v.set(x,y,z,w);

	switch(prop) {
		case POSITION:
			v.w = 1;
			break;
		case DIRECTION:
			v.w = 0;
			m_Float4Props[DIRECTION].set(v);
			m_Float4Props[NORMALIZED_DIRECTION].set(v);
			m_Float4Props[NORMALIZED_DIRECTION].normalize();
			break;
		case NORMALIZED_DIRECTION:
			v.w = 0;
			v.normalize();
			m_Float4Props[DIRECTION].set(v);
			m_Float4Props[NORMALIZED_DIRECTION].set(v);
			break;
		default:
			AttributeValues::setPropf4(prop, v);

	}
	m_Float4Props[prop].set(v);
}


void
Light::setPropf4(Float4Property prop, vec4& values){

	setPropf4(prop, values.x, values.y, values.z, values.w);
}



