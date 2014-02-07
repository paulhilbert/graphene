/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5LogDialog.h"
#include <iostream>

namespace GUI {

Qt5LogDialog::Qt5LogDialog(std::string title) : QDockWidget(QString::fromStdString(title)), m_area(new QWidget()), m_box(new QVBoxLayout()), m_text(new QTextEdit()) {
	m_area->setLayout(m_box);
	m_text->setReadOnly(true);
	m_box->addWidget(m_text);
	m_barPool = Qt5ProgressBarPool::Ptr(new Qt5ProgressBarPool());
	m_barPool->setBarCountChangeCallback([&] (int count) { if (count) m_barPool->show(); else m_barPool->hide(); });
	m_box->addWidget(m_barPool->widget());
	setWidget(m_area);
}

Qt5LogDialog::~Qt5LogDialog() {
}

void Qt5LogDialog::logInfo(std::string text) {
	append(formatPrefix("[I]", "#00dd00") + "  " + format(text));
}

void Qt5LogDialog::logWarn(std::string text) {
	append(formatPrefix("[W]", "#dd8800") + "  " +  format(text));
	show();
}

void Qt5LogDialog::logError(std::string text) {
	append(formatPrefix("[E]", "#dd0000") + "  " +  format(text));
	show();
}

void Qt5LogDialog::logVerbose(std::string text) {
	append(formatPrefix("[V]", "#800080") + "  " +  format(text));
}

void Qt5LogDialog::clear() {
	m_text->setHtml("");
}

ProgressBarPool::Ptr Qt5LogDialog::progressBarPool() {
	Qt5ProgressBarPoolHandle::Ptr pool(new Qt5ProgressBarPoolHandle());
	QObject::connect(pool.get(), SIGNAL(sigCreate(QString, int)), m_barPool.get(), SLOT(create(QString, int)));
	QObject::connect(m_barPool.get(), SIGNAL(sendBar(ProgressBar::Ptr)), pool.get(), SLOT(receiveBar(ProgressBar::Ptr)));
	return std::dynamic_pointer_cast<ProgressBarPool>(pool);
}

QString Qt5LogDialog::format(const std::string& text) {
	return "<font style=\"font-family: Inconsolata; font-size: 12pt\"><b>" + QString::fromStdString(text) + "</b></font>";
}

QString Qt5LogDialog::formatPrefix(const std::string& text, const std::string& color) {
	return QString::fromStdString("<font style=\"font-family: Inconsolata; font-size: 12pt; color: " + color + "\"><b>" + text + "</b></font>");
}

void Qt5LogDialog::append(const QString& string) {
	m_text->setHtml(m_text->toHtml() + "</ br>" + string);
}

} // GUI
