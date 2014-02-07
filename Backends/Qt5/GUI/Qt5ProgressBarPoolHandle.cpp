#include "Qt5ProgressBarPoolHandle.h"

namespace GUI {

Qt5ProgressBarPoolHandle::Qt5ProgressBarPoolHandle() : QObject(), ProgressBarPool() {
}

Qt5ProgressBarPoolHandle::~Qt5ProgressBarPoolHandle() {
}

ProgressBar::Ptr Qt5ProgressBarPoolHandle::create(std::string label, int steps) {
	QMutex mutex;
	mutex.lock();
	emit sigCreate(QString::fromStdString(label), steps);
	m_waiter.wait(&mutex);
	mutex.unlock();
	ProgressBar::Ptr bar = m_savedBar;
	m_savedBar.reset();
	return bar;
}

void Qt5ProgressBarPoolHandle::receiveBar(ProgressBar::Ptr bar) {
	m_savedBar = bar;
	m_waiter.wakeAll();
}

} // GUI
