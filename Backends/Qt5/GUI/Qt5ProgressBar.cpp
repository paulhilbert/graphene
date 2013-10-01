#include "Qt5ProgressBar.h"

#include <iostream>

namespace GUI {

Qt5ProgressBar::Qt5ProgressBar(IO::AbstractProgressBarPool* pool, int idx, std::string label, int steps) : IO::AbstractProgressBar(label, steps), m_pool(pool), m_idx(idx), m_area(new QWidget()), m_box(new QHBoxLayout()), m_labelWidget(new QLabel(QString::fromStdString(label))), m_bar(new QProgressBar()) {
	m_area->setLayout(m_box);
	m_box->addWidget(m_labelWidget, 0);
	m_box->addWidget(m_bar, 1);
	m_bar->setTextVisible(true);
	m_bar->setMinimum(0);
	m_bar->setMaximum(100);
	m_bar->setValue(0);
}

Qt5ProgressBar::~Qt5ProgressBar() {
	m_pool->notifyRemove();
	delete m_area;
}

void Qt5ProgressBar::poll(float progress) {
	m_bar->setValue(static_cast<int>(progress * 100.f));
}

QWidget* Qt5ProgressBar::widget() {
	return m_area;
}

int Qt5ProgressBar::index() const {
	return m_idx;
}

} // GUI
