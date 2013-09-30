#include "FWVisualizerHandle.h"

#include <thread>

namespace FW {

VisualizerHandle::VisualizerHandle(std::string id, View::Transforms::WPtr transforms, Events::EventHandler::Ptr eventHandler, Geometry::Ray::Ptr pickRay) : m_id(id), m_transforms(transforms), m_pickRay(pickRay) {
	m_events = Events::Handle::Ptr(new Events::Handle(id, eventHandler));
	m_modifier = eventHandler->modifier();
}

VisualizerHandle::~VisualizerHandle() {
}

View::Transforms::Ptr VisualizerHandle::transforms() {
	return m_transforms.lock();
}

Events::Handle::Ptr VisualizerHandle::events() {
	return m_events;
}

Events::Modifier::Ptr VisualizerHandle::modifier() {
	return m_modifier;
}

Geometry::Ray::Ptr VisualizerHandle::pickRay() {
	return m_pickRay;
}

} // FW
