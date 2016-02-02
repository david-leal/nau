#include "nau/material/iUniform.h"

using namespace nau::material;

void 
IUniform::setName(std::string &name) {

	m_Name = name;
}


std::string&
IUniform::getName(void) {

	return m_Name;
}


Enums::DataType
IUniform::getSimpleType() {

	return (m_SimpleType);
}


std::string
IUniform::getStringSimpleType() {

	return Enums::DataTypeToString[m_SimpleType];
}
