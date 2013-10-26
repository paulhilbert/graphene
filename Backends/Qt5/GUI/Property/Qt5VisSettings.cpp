/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5VisSettings.h"


#include "Qt5Bool.h"
#include "Qt5Button.h"
#include "Qt5Color.h"
#include "Qt5PropGroup.h"
#include "Qt5String.h"
#include "Qt5Separator.h"

namespace GUI {
namespace Property {

Qt5VisSettings::Qt5VisSettings() : Qt5Container(true), m_page(new QWidget()) {
	setWidget(m_page);
}

Qt5VisSettings::~Qt5VisSettings() {
}

//QWidget* Qt5VisSettings::widget() {
//	return m_area;
//}

} // Property
} // GUI
