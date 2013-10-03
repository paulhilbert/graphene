#include "Qt5ProgressBarHandle.h"

namespace GUI {


Qt5ProgressBarHandle::Qt5ProgressBarHandle(IO::AbstractProgressBarPool* pool, int idx, std::string label, int steps) : IO::AbstractProgressBar(label, steps), m_pool(pool), m_idx(idx) {
}

Qt5ProgressBarHandle::~Qt5ProgressBarHandle() {
	m_pool->remove(m_idx);
}

void Qt5ProgressBarHandle::poll(float progress) {
	emit polled(progress);
}

} // GUI
