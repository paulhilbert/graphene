/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5ProgressBarPool.h"

#include "Qt5ProgressBarHandle.h"

namespace GUI {

Qt5ProgressBarPool::Qt5ProgressBarPool() : IO::AbstractProgressBarPool(), m_area(new QWidget()), m_box(new QVBoxLayout())  {
	m_area->setLayout(m_box);
}

Qt5ProgressBarPool::~Qt5ProgressBarPool() {
}

void Qt5ProgressBarPool::show() {
	m_area->show();
}

void Qt5ProgressBarPool::hide() {
	m_area->hide();
}

QWidget* Qt5ProgressBarPool::widget() {
	return m_area;
}

IO::AbstractProgressBar::Ptr Qt5ProgressBarPool::createProgressBar(std::string label, int steps) {
	Qt5ProgressBar::Ptr bar(new Qt5ProgressBar(label));
	Qt5ProgressBarHandle::Ptr handle(new Qt5ProgressBarHandle(this, m_count, label, steps));
	QObject::connect(handle.get(), SIGNAL(polled(float)), bar.get(), SLOT(poll(float)));
	m_box->addWidget(bar->widget());
	m_bars.push_back(bar);
	return std::dynamic_pointer_cast<IO::AbstractProgressBar>(handle);
}

void Qt5ProgressBarPool::removeProgressBar(int index) {
	auto iter = m_bars.begin(); std::advance(iter, index);
	m_bars.erase(iter);
}

} // GUI
