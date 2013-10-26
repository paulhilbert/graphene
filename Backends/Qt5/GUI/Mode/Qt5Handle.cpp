/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Handle.h"

#include "Qt5ModeGroup.h"

namespace GUI {
namespace Mode {

Qt5Handle::Qt5Handle(QToolBar* toolbar, Log::Ptr log) : Handle(log), m_toolbar(toolbar) {
}

Qt5Handle::~Qt5Handle() {
}

Group::Ptr Qt5Handle::createGroup() {
	Qt5Group::Ptr group(new Qt5Group(m_toolbar, m_log));
	return std::dynamic_pointer_cast<Group>(group);
}

} // Mode
} // GUI
