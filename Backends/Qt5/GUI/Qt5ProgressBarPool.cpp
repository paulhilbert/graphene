/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5ProgressBarPool.h"

#include "Qt5ProgressBarHandle.h"

namespace GUI {

Qt5ProgressBarPool::Qt5ProgressBarPool() : m_area(new QWidget()), m_box(new QVBoxLayout()), m_lastIndex(0), m_callback(nullptr) {
	m_area->setLayout(m_box);
	//m_mapper = new QSignalMapper();
	//connect(m_mapper, SIGNAL(mapped(int)), this, SLOT(removeProgressBar(int)));
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

void Qt5ProgressBarPool::setBarCountChangeCallback(std::function<void (int)> callback) {
	m_callback = std::move(callback);
}

void Qt5ProgressBarPool::create(QString label, int steps) {
	std::lock_guard<std::mutex> lg(m_mutex);
	Qt5ProgressBar::Ptr bar(new Qt5ProgressBar(label.toStdString()));
	Qt5ProgressBarHandle::Ptr handle(new Qt5ProgressBarHandle(m_lastIndex, label.toStdString(), steps));
	QObject::connect(handle.get(), SIGNAL(polled(int, float)), this, SLOT(poll(int, float)));
	connect(handle.get(), SIGNAL(sigDestroy(int)), this, SLOT(removeProgressBar(int))) ;
	//connect(handle.get(), SIGNAL(destroyed()), m_mapper, SLOT(map())) ;
	//m_mapper->setMapping(handle.get(), m_lastIndex) ;
	m_box->addWidget(bar->widget());
	m_bars[m_lastIndex] = bar;
	++m_lastIndex;
	if (m_callback) m_callback(m_bars.size());
	emit sendBar(std::dynamic_pointer_cast<ProgressBar>(handle));
}

void Qt5ProgressBarPool::removeProgressBar(int index) {
	std::lock_guard<std::mutex> lg(m_mutex);
	auto find = m_bars.find(index);
	if (find == m_bars.end()) return;
	m_bars.erase(find);
}

void Qt5ProgressBarPool::poll(int index, float progress) {
	std::lock_guard<std::mutex> lg(m_mutex);
	if (m_bars.find(index) == m_bars.end()) return;
	m_bars[index]->poll(progress);
}

} // GUI
