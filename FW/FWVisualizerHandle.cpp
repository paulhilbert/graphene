/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "FWVisualizerHandle.h"

#include <thread>

namespace FW {

VisualizerHandle::VisualizerHandle(std::string id, Events::EventHandler::Ptr eventHandler, harmont::camera::const_ptr camera) : m_id(id), m_camera(camera)  {
	m_events = Events::Handle::Ptr(new Events::Handle(id, eventHandler));
	m_modifier = eventHandler->modifier();
}

VisualizerHandle::~VisualizerHandle() {
}

Events::Handle::Ptr VisualizerHandle::events() {
	return m_events;
}

Events::Modifier::Ptr VisualizerHandle::modifier() {
	return m_modifier;
}

 harmont::camera::const_ptr VisualizerHandle::camera() const {
     return m_camera;
 }

} // FW
