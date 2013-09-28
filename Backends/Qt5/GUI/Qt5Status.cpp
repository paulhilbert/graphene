#include "Qt5Status.h"

#include <QtWidgets/QStatusBar>

namespace GUI {

Qt5Status::Qt5Status(QMainWindow* mainWindow) : Status(), m_mainWindow(mainWindow) {
}

Qt5Status::~Qt5Status() {
}

void Qt5Status::set(const std::string& text) {
	m_mainWindow->statusBar()->showMessage(QString::fromStdString(text));
}

std::string Qt5Status::get() const {
	return m_mainWindow->statusBar()->currentMessage().toStdString();
}

void Qt5Status::clear() {
	m_mainWindow->statusBar()->clearMessage();
}

} // GUI
