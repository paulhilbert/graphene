/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5ProgressBar.h"

#include <include/common.h>

namespace GUI {

Qt5ProgressBar::Qt5ProgressBar(std::string label) : m_label(label), m_area(new QWidget()), m_box(new QHBoxLayout()), m_labelWidget(new QLabel(QString::fromStdString(label))), m_bar(new QProgressBar()) {
	m_area->setLayout(m_box);
	m_box->addWidget(m_labelWidget, 0);
	m_box->addWidget(m_bar, 1);
	m_bar->setTextVisible(true);
	m_bar->setMinimum(0);
	m_bar->setMaximum(100);
	m_bar->setValue(0);
}

Qt5ProgressBar::~Qt5ProgressBar() {
	delete m_area;
}

QWidget* Qt5ProgressBar::widget() {
	return m_area;
}

void Qt5ProgressBar::poll(float progress) {
	m_bar->setValue(static_cast<int>(progress * 100.f));
}

} // GUI
