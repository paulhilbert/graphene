#include "Qt5Log.h"

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

} // GUI
