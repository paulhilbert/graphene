/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "GUIVisualizerHandle.h"
#include "GUIBackend.h"

namespace GUI {

VisualizerHandle::VisualizerHandle(Property::Container::Ptr properties, Mode::Handle::Ptr modes, Log::Ptr log, Status::Ptr status, bool alwaysActive) : m_properties(properties), m_modes(modes), m_log(log), m_status(status) {
	if (!alwaysActive) {
		m_properties->get<Bool>(std::vector<std::string>(1, "__active__"))->setCallback([&] (bool active) {
			if (active) m_modes->enable(); else m_modes->disable();
		});
	}
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
