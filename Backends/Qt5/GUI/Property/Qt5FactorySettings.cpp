/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5FactorySettings.h"

namespace GUI {
namespace Property {

Qt5FactorySettings::Qt5FactorySettings(std::string name) : Qt5Container(), m_group(new QGroupBox(QString::fromStdString(name + " settings"))) {
	setWidget(m_group);
}

Qt5FactorySettings::~Qt5FactorySettings() {
}

//QWidget* Qt5FactorySettings::widget() {
//	return m_area;
//}

} // Property
} // GUI
