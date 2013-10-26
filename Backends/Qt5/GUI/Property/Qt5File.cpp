/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5File.h"

#include <QtWidgets/QFileDialog>

namespace GUI {
namespace Property {

Qt5File::Qt5File(std::string label) : File(label), m_outer(new QWidget()), m_inner(new QWidget()), m_vBox(new QVBoxLayout()), m_hBox(new QHBoxLayout()), m_labelWidget(new QLabel(QString::fromStdString(label))), m_lineEdit(new QLineEdit()), m_button(new QPushButton()), m_mode(OPEN)  {
	m_outer->setLayout(m_vBox);
	m_inner->setLayout(m_hBox);
	m_vBox->addWidget(m_labelWidget, 0, Qt::AlignLeft);
	m_vBox->addWidget(m_inner, 1);
	m_hBox->addWidget(m_lineEdit, 1);
	m_hBox->addWidget(m_button, 0);
	QObject::connect(m_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
	m_button->setIcon(QIcon("Icons/fileopen.png"));
	m_lineEdit->setMinimumSize(200, 20);
}

Qt5File::~Qt5File() {
}

void Qt5File::setMode(Mode mode) {
	m_mode = mode;
}

void Qt5File::setExtensions(std::vector<std::string> extensions) {
	m_extensions = extensions;
}

void Qt5File::show() {
	m_outer->show();
}

void Qt5File::hide() {
	m_outer->hide();
}

bool Qt5File::visible() const {
	return m_outer->isVisible();
}

void Qt5File::enable() {
	m_outer->setEnabled(true);
}

void Qt5File::disable() {
	m_outer->setEnabled(false);
}

bool Qt5File::enabled() const {
	return m_outer->isEnabled();
}

fs::path Qt5File::value() const {
	return fs::path(m_lineEdit->text().toStdString());
}

void Qt5File::setValue(fs::path value) {
	m_lineEdit->setText(QString::fromStdString(value.string()));
}

void Qt5File::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5File::widget() {
	return m_outer;
}

void Qt5File::buttonClicked() {
	QString filter;
	if (!m_extensions.size()) {
		filter = tr("All Files (*.*)");
	} else {
		filter = tr("Valid Files (");
		for (unsigned int i=0; i<m_extensions.size(); ++i) {
			if (i) filter += " ";
			filter += "*."+QString::fromStdString(m_extensions[i]);
		}
		filter += tr(")");
	}
	QString file;
	if (m_mode == OPEN) {
		file = QFileDialog::getOpenFileName(nullptr, "Open...", QString(), filter, nullptr, 0);
	} else {
		file = QFileDialog::getSaveFileName(nullptr, "Save...", QString(), filter, nullptr, 0);
	}
	if (file == "") return;
	fs::path path(file.toStdString());
	if ((m_mode == OPEN && fs::exists(path)) || m_mode == SAVE) {
		m_lineEdit->setText(file);
		notify(path);
	}
}

} // Property
} // GUI
