#include "Qt5ProgressBarPool.h"

namespace GUI {

Qt5ProgressBarPool::Qt5ProgressBarPool() : IO::AbstractProgressBarPool(), m_area(new QWidget()), m_box(new QVBoxLayout()), m_count(0)  {
	m_area->setLayout(m_box);
}

Qt5ProgressBarPool::~Qt5ProgressBarPool() {
}

void Qt5ProgressBarPool::show() {
	std::cout << "show" << "\n";
	m_area->show();
}

void Qt5ProgressBarPool::hide() {
	std::cout << "hide" << "\n";
	m_area->hide();
}

QWidget* Qt5ProgressBarPool::widget() {
	return m_area;
}

void Qt5ProgressBarPool::setBarCountChangeCallback(std::function<void (int)> func) {
	m_onChange = std::move(func);
}

void Qt5ProgressBarPool::notifyRemove() {
	m_onChange(--m_count);
}

IO::AbstractProgressBar::Ptr Qt5ProgressBarPool::createProgressBar(std::string label, int steps) {
	Qt5ProgressBar::Ptr bar(new Qt5ProgressBar(this, m_count, label, steps));
	m_box->addWidget(bar->widget());
	m_onChange(++m_count);
	return std::dynamic_pointer_cast<IO::AbstractProgressBar>(bar);
}

/*
void Qt5ProgressBarPool::removeProgressBar(QWidget* widget) {
	m_box->removeWidget(widget);
	m_onChange(--m_count);
}
*/

} // GUI
