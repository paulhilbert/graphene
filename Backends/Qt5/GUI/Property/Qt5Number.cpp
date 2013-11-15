/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Number.h"

namespace GUI {
namespace Property {

Qt5Number::Qt5Number(std::string label, QLabel* labelWidget) : Number(label), m_labelWidget(labelWidget), m_spinBox(new QDoubleSpinBox()) {
	QObject::connect(m_spinBox, SIGNAL(valueChanged(double)), this, SLOT(valueChanged(double)));
}

Qt5Number::~Qt5Number() {
}

Number& Qt5Number::setMin(double min) {
	m_spinBox->setMinimum(min);
	return *this;
}

Number& Qt5Number::setMax(double max) {
	m_spinBox->setMaximum(max);
	return *this;
}

Number& Qt5Number::setDigits(int digits) {
	m_spinBox->setDecimals(digits);
	setStep(std::pow(0.1, digits));
	return *this;
}

Number& Qt5Number::setStep(double step) {
	m_spinBox->setSingleStep(step);
	return *this;
}

void Qt5Number::show() {
	m_spinBox->show();
	m_labelWidget->show();
}

void Qt5Number::hide() {
	m_spinBox->hide();
	m_labelWidget->hide();
}

bool Qt5Number::visible() const {
	return m_spinBox->isVisible();
}

void Qt5Number::enable() {
	m_spinBox->setEnabled(true);
	m_labelWidget->setEnabled(true);
}

void Qt5Number::disable() {
	m_spinBox->setEnabled(false);
	m_labelWidget->setEnabled(false);
}

bool Qt5Number::enabled() const {
	return m_spinBox->isEnabled();
}

double Qt5Number::value() const {
	return m_spinBox->value();
}

void Qt5Number::setValue(double value) {
	m_spinBox->setValue(value);
}

void Qt5Number::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5Number::widget() {
	return m_spinBox;
}

void Qt5Number::valueChanged(double value) {
	notify(value);
}


} // Property
} // GUI
