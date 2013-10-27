/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "FWFactory.h"

namespace FW {

Factory::Factory() {
}

Factory::~Factory() {
}

void Factory::setGUIHandle(GUI::FactoryHandle::Ptr handle) {
	m_gui = handle;
}

GUI::FactoryHandle::Ptr Factory::gui() {
	asserts(m_gui, "Trying to get invalid gui handle. Do not use the Factory::gui() function inside the factory constructor.");
	return m_gui;
}

} // FW
