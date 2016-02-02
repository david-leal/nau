#include "nau/scene/scenePoses.h"
#include "nau/geometry/meshWithPose.h"
#include "nau/slogger.h"

using namespace nau::scene;

ScenePoses::ScenePoses(void): Scene(), m_ActiveAnim("")
{
	// have to register for events
	EVENTMANAGER->addListener("SET_POSE_BY_INDEX",(nau::event_::IListener *)this);
	EVENTMANAGER->addListener("SET_POSE_BY_NAME",(nau::event_::IListener *)this);
	EVENTMANAGER->addListener("ANIMATE",(nau::event_::IListener *)this);

	m_Type = "ScenePoses";
}


ScenePoses::~ScenePoses(void) {

	// have to remove events
	EVENTMANAGER->removeListener("SET_POSE_BY_INDEX",(nau::event_::IListener *)this);
	EVENTMANAGER->removeListener("SET_POSE_BY_NAME",(nau::event_::IListener *)this);
	EVENTMANAGER->removeListener("ANIMATE",(nau::event_::IListener *)this);

}


void
ScenePoses::eventReceived(const std::string &sender, const std::string &eventType, 
	const std::shared_ptr<IEventData> &evt) {

	if (eventType == "SET_POSE_BY_INDEX")
		setPose(*(int *)evt->getData());
	else if (eventType == "SET_POSE_BY_NAME")
		setPose(*(std::string*)evt->getData());
	else if (eventType == "ANIMATE")
		setRelativeTime(m_ActiveAnim, * ((float *)evt->getData()));
}


void 
ScenePoses::compile() {

}


void 
ScenePoses::setActiveAnim(std::string aName) {

	if (m_Anims.count(aName) != 0)
		m_ActiveAnim = aName;
	else
		m_ActiveAnim = "";
}


std::string
ScenePoses::getActiveAnim() {

	return m_ActiveAnim;
}


void 
ScenePoses::addAnim(std::string aName, float aLength) {

	m_Anims[aName] = MeshPoseAnim();
	m_Anims[aName].setLength(aLength);

	// by default set the active anim to be the first one added.
	if (m_ActiveAnim == "")
		m_ActiveAnim = aName;
}


void 
ScenePoses::addAnimTrack(std::string aName, unsigned int meshIndex) {

	if (m_Anims.count (aName) > 0)
		m_Anims[aName].addTrack(meshIndex);
}


void
ScenePoses::setPose(int index) {

	unsigned int numScenes = (unsigned int)m_SceneObjects.size();
	for (unsigned int i = 0; i < numScenes; i++) {
		
		std::shared_ptr<MeshPose> &mp = dynamic_pointer_cast<MeshPose>(m_SceneObjects[i]);
		mp->setPose(i);
	}
}


void
ScenePoses::setPose(std::string name) {

	unsigned int numScenes = (unsigned int)m_SceneObjects.size();
	for (unsigned int i = 0; i < numScenes; i++) {
		
		std::shared_ptr<MeshPose> &mp = dynamic_pointer_cast<MeshPose>(m_SceneObjects[i]);
		mp->setPose(name);
	}
}


// must exist, otherwise ...
MeshPoseAnim &
ScenePoses::getAnim(std::string aName) {

	return m_Anims[aName];
}


void 
ScenePoses::setRelativeTime(std::string aAnim, float time) {

	SLOG("time = %f\n", time);
	if (m_Anims.count (aAnim) > 0) {

		MeshPoseAnim mpa = m_Anims[aAnim];

		for (unsigned int i = 0; i < m_SceneObjects.size(); i++) {
			std::shared_ptr<MeshPose> mp = std::dynamic_pointer_cast<MeshPose>(m_SceneObjects[i]->getRenderable());
			mp->setPose(mpa.getInfluences(i,time * mpa.getLength()));
		} 
	}
}


