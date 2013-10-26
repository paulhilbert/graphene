/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Log.h"

#include <QtWidgets/QMessageBox>

namespace GUI {

Qt5Log::Qt5Log(Qt5LogDialog* log, bool verbose) : Log(verbose), m_log(log) {
}

Qt5Log::~Qt5Log() {
}

void Qt5Log::info(std::string text) {
	m_log->logInfo(text);
}

void Qt5Log::warn(std::string text) {
	m_log->logWarn(text);
}

void Qt5Log::error(std::string text) {
	m_log->logError(text);
}

void Qt5Log::verbose(std::string text) {
	if (m_verbose) m_log->logVerbose(text);
}

void Qt5Log::clear() {
	m_log->clear();
}

void Qt5Log::fail(std::string text) {
    QMessageBox::critical(nullptr, "Critical Error", QString::fromStdString(text), QMessageBox::Ok);
}

} // GUI
