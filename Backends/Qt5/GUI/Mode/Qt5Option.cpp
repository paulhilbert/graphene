/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Option.h"

#include "Qt5ModeGroup.h"

namespace GUI {
namespace Mode {

Qt5Option::Qt5Option(std::string id, std::string label, fs::path icon, Qt5Group* group) : QObject(), Option(id, label, icon), m_group(group), m_button(new QToolButton()) {
	m_button->setText(QString::fromStdString(label));
	m_button->setIcon(QIcon(QString::fromStdString(icon.string())));
	m_button->setCheckable(true);
	QObject::connect(m_button, SIGNAL(toggled(bool)), this, SLOT(stateChanged()));
}

Qt5Option::~Qt5Option() {
	delete m_button;
}

bool Qt5Option::active() const {
	return m_button->isChecked();
}

void Qt5Option::setActive(bool active) {
	m_button->setChecked(active);
}

void Qt5Option::show() {
	//m_button->show();
	m_button->setVisible(true);
}

void Qt5Option::hide() {
	//m_button->hide();
	m_button->setVisible(false);
}

bool Qt5Option::visible() const {
	return m_button->isVisible();
}

void Qt5Option::enable() {
	m_button->setEnabled(true);
}

void Qt5Option::disable() {
	m_button->setEnabled(false);
}

bool Qt5Option::enabled() const {
	return m_button->isEnabled();
}

QWidget* Qt5Option::widget() {
	return m_button;
}

void Qt5Option::stateChanged() {
	m_group->notify(m_id);
}

} // Mode
} // GUI
