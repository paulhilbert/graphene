#include "Qt5Handle.h"

#include "Qt5ModeGroup.h"

namespace GUI {
namespace Mode {

Qt5Handle::Qt5Handle(QToolBar* toolbar, Log::Ptr log) : Handle(log), m_toolbar(toolbar) {
}

Qt5Handle::~Qt5Handle() {
}

Group::Ptr Qt5Handle::createGroup(const Group::Callback& onChange) {
	Qt5Group::Ptr group(new Qt5Group(m_toolbar, m_log, onChange));
	return std::dynamic_pointer_cast<Group>(group);
}

} // Mode
} // GUI
