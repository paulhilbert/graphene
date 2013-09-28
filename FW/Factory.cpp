#include "Factory.h"

#include <Testing/asserts.h>

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
