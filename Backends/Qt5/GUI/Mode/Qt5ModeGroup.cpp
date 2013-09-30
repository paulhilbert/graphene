#include "Qt5Group.h"

#include "Qt5Option.h"

namespace GUI {
namespace Mode {

Qt5Group::Qt5Group(QToolBar* toolbar, Log::Ptr log, const Callback& onChange) : Group(log, onChange), m_toolbar(toolbar) {
}

Qt5Group::~Qt5Group() {
}

Option::Ptr Qt5Group::createOption(std::string id, std::string label, fs::path icon) {
	Qt5Option::Ptr option(new Qt5Option(id, label, icon, this));
	if (!m_modes.size()) m_toolbar->addSeparator();
	m_toolbar->addWidget(option->widget());
	return std::dynamic_pointer_cast<Option>(option);
}

} // Mode
} // GUI
