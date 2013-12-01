/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "FWVisualizerHandle.h"

#include <thread>

namespace FW {

VisualizerHandle::VisualizerHandle(std::string id, View::Transforms::WPtr transforms, Events::EventHandler::Ptr eventHandler, Geometry::Ray::Ptr pickRay, std::map<std::string, EnvTex>* envMaps, std::string* crtMap) : m_id(id), m_transforms(transforms), m_pickRay(pickRay), m_envMaps(envMaps), m_crtMap(crtMap) {
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

optional<EnvTex> VisualizerHandle::environmentMaps() {
	if (!m_crtMap || !m_envMaps->size()) return none;
	return (*m_envMaps)[*m_crtMap];
}

} // FW
