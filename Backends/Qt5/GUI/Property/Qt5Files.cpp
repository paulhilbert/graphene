/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */

#include <include/config.h>

#include "Qt5Files.h"

#include <QtWidgets/QFileDialog>

namespace GUI {
namespace Property {

Qt5Files::Qt5Files(std::string label) : Files(label), m_outer(new QWidget()), m_inner(new QWidget()), m_vBox(new QVBoxLayout()), m_hBox(new QHBoxLayout()), m_labelWidget(new QLabel(QString::fromStdString(label))), m_lineEdit(new QLineEdit()), m_button(new QPushButton())  {
	m_outer->setLayout(m_vBox);
	m_inner->setLayout(m_hBox);
	m_vBox->addWidget(m_labelWidget, 0, Qt::AlignLeft);
	m_vBox->addWidget(m_inner, 1);
	m_hBox->addWidget(m_lineEdit, 1);
	m_hBox->addWidget(m_button, 0);
	m_button->setIcon(QIcon(QString(ICON_PREFIX)+"fileopen.png"));
	QObject::connect(m_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
}

Qt5Files::~Qt5Files() {
}

void Qt5Files::setExtensions(std::vector<std::string> extensions) {
	m_extensions = extensions;
}

void Qt5Files::show() {
	m_outer->show();
}

void Qt5Files::hide() {
	m_outer->hide();
}

bool Qt5Files::visible() const {
	return m_outer->isVisible();
}

void Qt5Files::enable() {
	m_outer->setEnabled(true);
}

void Qt5Files::disable() {
	m_outer->setEnabled(false);
}

bool Qt5Files::enabled() const {
	return m_outer->isEnabled();
}

Paths Qt5Files::value() const {
	return m_value;
}

void Qt5Files::setValue(Paths value) {
	QString text;
	for (const auto& p : value) {
		if (text != "") text += ", ";
		text += QString::fromStdString(p.string());
	}
	m_lineEdit->setText(text);
}

void Qt5Files::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5Files::widget() {
	return m_outer;
}

void Qt5Files::buttonClicked() {
	QString filter;
    filter = tr("All Files (*.*)");
#if !(defined(WIN32) || defined(_WIN32) || defined(__WIN32))
	if (m_extensions.size()) {
		filter = tr("Valid Files (");
		for (unsigned int i=0; i<m_extensions.size(); ++i) {
			if (i) filter += " ";
			filter += "*."+QString::fromStdString(m_extensions[i]);
		}
		filter += tr(")");
	}
#else
    filter = QDir::Files|QDir::Readable|QDir:: Writable;
#endif
	QStringList files = QFileDialog::getOpenFileNames(nullptr, "Open Files...", filter, tr("All Files (*.*)"), nullptr, 0);
	QStringList::Iterator it = files.begin();
	m_value.clear();
	QString text;
	while (it != files.end()) {
		m_value.push_back(fs::path(QDir::toNativeSeparators(*it).toStdString()));
		if (it != files.begin()) text += ", ";
		text += *it;
		++it;
	}
	m_lineEdit->setText(text);
	notify(m_value);
}

} // Property
} // GUI
