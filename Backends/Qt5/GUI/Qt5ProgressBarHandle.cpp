/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


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
