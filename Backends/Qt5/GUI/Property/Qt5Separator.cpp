/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Separator.h"

namespace GUI {
namespace Property {

Qt5Separator::Qt5Separator() : Separator(), m_frame(new QFrame()) {
	m_frame->setFrameShape(QFrame::HLine);
	m_frame->setFrameShadow(QFrame::Sunken);
}

Qt5Separator::~Qt5Separator() {
}

void Qt5Separator::show() {
	m_frame->show();
}

void Qt5Separator::hide() {
	m_frame->hide();
}

bool Qt5Separator::visible() const {
	return m_frame->isVisible();
}

void Qt5Separator::enable() {
	m_frame->setEnabled(true);
}

void Qt5Separator::disable() {
	m_frame->setEnabled(false);
}

bool Qt5Separator::enabled() const {
	return m_frame->isEnabled();
}

QWidget* Qt5Separator::widget() {
	return m_frame;
}

} // Property
} // GUI
