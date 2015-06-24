#include "nau/render/passCompute.h"

#include "nau.h"
#include "nau/debug/profile.h"

using namespace nau::material;
using namespace nau::scene;
using namespace nau::render;
using namespace nau::geometry;

bool PassCompute::Inited = PassCompute::Init();

bool
PassCompute::Init() {

	//UINT
	Attribs.add(Attribute(DIM_X, "DIM_X", Enums::DataType::UINT, false, new unsigned int(1)));
	Attribs.add(Attribute(DIM_Y, "DIM_Y", Enums::DataType::UINT, false, new unsigned int(1)));
	Attribs.add(Attribute(DIM_Z, "DIM_Z", Enums::DataType::UINT, false, new unsigned int(1)));

//	NAU->registerAttributes("PASS_COMPUTE", &Attribs);
	return true;
}


PassCompute::PassCompute(const std::string &passName) : Pass(passName),
m_Mat(0), 
m_BufferX(0), m_BufferY(0), m_BufferZ(0),
m_OffsetX(0), m_OffsetY(0), m_OffsetZ(0) {

	m_ClassName = "compute";

	registerAndInitArrays(Attribs);
}


void
PassCompute::eventReceived(const std::string &sender, const std::string &eventType, IEventData *evtData) {

}


PassCompute::~PassCompute(){

}


void
PassCompute::prepare (void) {

	m_Mat->prepare();	

	if (m_BufferX) {
		m_BufferX->getData(m_OffsetX, 4, &m_UIntProps[DIM_X]);
	}
	if (m_BufferY) {
		m_BufferY->getData(m_OffsetY, 4, &m_UIntProps[DIM_Y]);
	}
	if (m_BufferZ) {
		m_BufferZ->getData(m_OffsetZ, 4, &m_UIntProps[DIM_Z]);
	}
}


void
PassCompute::restore (void) {

	m_Mat->restore();
}


void
PassCompute::doPass (void) {

	for (auto pp : m_PreProcessList)
		pp->process();

	PROFILE_GL("Compute shader");
	RENDERER->dispatchCompute(m_UIntProps[DIM_X], m_UIntProps[DIM_Y], m_UIntProps[DIM_Z]);

	for (auto pp : m_PostProcessList)
		pp->process();
}


void 
PassCompute::setMaterialName(const std::string &lName,const std::string &mName) {

	m_Mat = MATERIALLIBMANAGER->getMaterial(lName, mName);
	m_MaterialMap[mName] = MaterialID(lName, mName);
}


Material *
PassCompute::getMaterial() {

	return m_Mat;
}


void
PassCompute::setDimension(int dimX, int dimY, int dimZ) {

	m_UIntProps[DIM_X] = dimX;
	m_UIntProps[DIM_Y] = dimY;
	m_UIntProps[DIM_Z] = dimZ;
}


void
PassCompute::setDimFromBuffer(IBuffer  *buffNameX, unsigned int offX,
								IBuffer  *buffNameY, unsigned int offY, 
								IBuffer  *buffNameZ, unsigned int offZ ) {

	m_BufferX = buffNameX;
	m_BufferY = buffNameY;
	m_BufferZ = buffNameZ;

	m_OffsetX = offX;
	m_OffsetY = offY;
	m_OffsetZ = offZ;
}