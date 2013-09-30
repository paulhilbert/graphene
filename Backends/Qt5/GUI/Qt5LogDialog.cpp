#include "Qt5LogDialog.h"
#include <iostream>

namespace GUI {

Qt5LogDialog::Qt5LogDialog(std::string title) : QDockWidget(QString::fromStdString(title)), m_text(new QTextEdit()) {
	m_text->setReadOnly(true);
	setWidget(m_text);
}

Qt5LogDialog::~Qt5LogDialog() {
}

void Qt5LogDialog::logInfo(std::string text) {
	append(formatPrefix("[I]", "#00dd00") + "  " + format(text));
}

void Qt5LogDialog::logWarn(std::string text) {
	append(formatPrefix("[W]", "#dd8800") + "  " +  format(text));
}

void Qt5LogDialog::logError(std::string text) {
	append(formatPrefix("[E]", "#dd0000") + "  " +  format(text));
}

void Qt5LogDialog::logVerbose(std::string text) {
	append(formatPrefix("[V]", "#800080") + "  " +  format(text));
}

void Qt5LogDialog::clear() {
	m_text->setHtml("");
}

QString Qt5LogDialog::format(const std::string& text) {
	return "<font style=\"font-family: Inconsolata; font-size: 12pt\"><b>" + QString::fromStdString(text) + "</b></font>";
}

QString Qt5LogDialog::formatPrefix(const std::string& text, const std::string& color) {
	return QString::fromStdString("<font style=\"font-family: Inconsolata; font-size: 12pt; color: " + color + "\"><b>" + text + "</b></font>");
}

void Qt5LogDialog::append(const QString& string) {
	m_text->setHtml(m_text->toHtml() + "</ br>" + string);
	show();
}

} // GUI
