#include "VisualizerHandle.h"
#include "Backend.h"

namespace GUI {

VisualizerHandle::VisualizerHandle(Property::Container::Ptr properties, Mode::Handle::Ptr modes, Log::Ptr log, Status::Ptr status) : m_properties(properties), m_modes(modes), m_log(log), m_status(status) {
	m_properties->get<Bool>({"__active__"})->setCallback([&] (bool active) {
		if (active) m_modes->enable(); else m_modes->disable();
	});
}

VisualizerHandle::~VisualizerHandle() {
}

Property::Container::Ptr VisualizerHandle::properties() {
	return m_properties;
}

Mode::Handle::Ptr VisualizerHandle::modes() {
	return m_modes;
}

Log::Ptr VisualizerHandle::log() {
	return m_log;
}

Status::Ptr VisualizerHandle::status() {
	return m_status;
}

} // GUI
