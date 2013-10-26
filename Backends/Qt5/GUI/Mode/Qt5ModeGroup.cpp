/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5ModeGroup.h"

#include "Qt5Option.h"

namespace GUI {
namespace Mode {

Qt5Group::Qt5Group(QToolBar* toolbar, Log::Ptr log) : Group(log), m_toolbar(toolbar) {
}

Qt5Group::~Qt5Group() {
	if (m_modes.size()) delete m_separator;
}

Option::Ptr Qt5Group::createOption(std::string id, std::string label, fs::path icon) {
	Qt5Option::Ptr option(new Qt5Option(id, label, icon, this));
	if (!m_modes.size()) m_separator = m_toolbar->addSeparator();
	m_toolbar->addWidget(option->widget());
	return std::dynamic_pointer_cast<Option>(option);
}

} // Mode
} // GUI
