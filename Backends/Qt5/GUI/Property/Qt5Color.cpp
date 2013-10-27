/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Color.h"

#include <QtGui/QColor>
#include <QtWidgets/QColorDialog>
#include <Library/Colors/Conversion.h>

namespace GUI {
namespace Property {

Qt5Color::Qt5Color(std::string label, QLabel* labelWidget) : Color(label), m_labelWidget(labelWidget), m_button(new QPushButton()), m_color(1.f, 1.f, 1.f, 1.f) {
	m_button->setFlat(true);
	m_button->setText("Choose...");
	setButtonColor();
	QObject::connect(m_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
}

Qt5Color::~Qt5Color() {
}

void Qt5Color::show() {
	m_button->show();
	m_labelWidget->show();
}

void Qt5Color::hide() {
	m_button->hide();
	m_labelWidget->hide();
}

bool Qt5Color::visible() const {
	return m_button->isVisible();
}

void Qt5Color::enable() {
	m_button->setEnabled(true);
	m_labelWidget->setEnabled(true);
}

void Qt5Color::disable() {
	m_button->setEnabled(false);
	m_labelWidget->setEnabled(false);
}

bool Qt5Color::enabled() const {
	return m_button->isEnabled();
}

Eigen::Vector4f Qt5Color::value() const {
	return m_color;
}

void Qt5Color::setValue(Eigen::Vector4f value) {
	m_color = value;
	setButtonColor();
}

void Qt5Color::setLabel(std::string label) {
	m_label = label;
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5Color::widget() {
	return m_button;
}

void Qt5Color::setButtonColor() {
	QColor color = QColor::fromRgbF(m_color[0], m_color[1], m_color[2], m_color[3]);
	// compute text color
	auto hsva = Colors::Conversion::rgba2hsva(m_color);
	QString textColor = (hsva[2] > 0.5f) ? "#000000" : "#ffffff";
	m_button->setStyleSheet("QPushButton:flat { color: "+textColor+"; background-color:"+color.name()+"; border: 1px solid black; margin: 0px 0px 0px 2px}");
}

void Qt5Color::buttonClicked() {
	QColor initial = QColor::fromRgbF(m_color[0], m_color[1], m_color[2], m_color[3]);
	QColor color = QColorDialog::getColor(initial, nullptr, "Select Color", QColorDialog::ShowAlphaChannel | QColorDialog::DontUseNativeDialog);
	m_color << color.redF(), color.greenF(), color.blueF(), color.alphaF();
	setButtonColor();
	notify(m_color);
}

} // Property
} // GUI
