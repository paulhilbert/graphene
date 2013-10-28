/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */

#include <include/config.h>

#include "Qt5Folder.h"

#include <QtWidgets/QFileDialog>

namespace GUI {
namespace Property {

Qt5Folder::Qt5Folder(std::string label) : Folder(label), m_outer(new QWidget()), m_inner(new QWidget()), m_vBox(new QVBoxLayout()), m_hBox(new QHBoxLayout()), m_labelWidget(new QLabel(QString::fromStdString(label))), m_lineEdit(new QLineEdit()), m_button(new QPushButton())  {
	m_outer->setLayout(m_vBox);
	m_inner->setLayout(m_hBox);
	m_vBox->addWidget(m_labelWidget, 0, Qt::AlignLeft);
	m_vBox->addWidget(m_inner, 1);
	m_hBox->addWidget(m_lineEdit, 1);
	m_hBox->addWidget(m_button, 0);
	m_button->setIcon(QIcon(QString(ICON_PREFIX)+"fileopen.png"));
	QObject::connect(m_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
}

Qt5Folder::~Qt5Folder() {
}

void Qt5Folder::show() {
	m_outer->show();
}

void Qt5Folder::hide() {
	m_outer->hide();
}

bool Qt5Folder::visible() const {
	return m_outer->isVisible();
}

void Qt5Folder::enable() {
	m_outer->setEnabled(true);
}

void Qt5Folder::disable() {
	m_outer->setEnabled(false);
}

bool Qt5Folder::enabled() const {
	return m_outer->isEnabled();
}

fs::path Qt5Folder::value() const {
	return fs::path(m_lineEdit->text().toStdString());
}

void Qt5Folder::setValue(fs::path value) {
	m_lineEdit->setText(QString::fromStdString(value.string()));
}

void Qt5Folder::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5Folder::widget() {
	return m_outer;
}

void Qt5Folder::buttonClicked() {
	QString folder = QFileDialog::getExistingDirectory(nullptr, "Chose directory...", QString(), QFileDialog::ShowDirsOnly);
	m_lineEdit->setText(folder);
	notify(value());
}

} // Property
} // GUI
