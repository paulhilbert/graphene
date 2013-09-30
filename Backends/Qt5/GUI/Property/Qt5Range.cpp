#include "Qt5Range.h"

namespace GUI {
namespace Property {

Qt5Range::Qt5Range(std::string label, QLabel* labelWidget) : Range(label), m_labelWidget(labelWidget), m_area(new QWidget()), m_box(new QHBoxLayout()), m_valueLabel(new QLabel()), m_slider(new QSlider(Qt::Horizontal)), m_minimum(0), m_maximum(1), m_digits(2) {
	m_area->setLayout(m_box);
	m_slider->setMinimum(0);
	m_slider->setTracking(true);
	m_slider->setSingleStep(1);
	m_slider->setValue(0);
	updateSlider();
	QObject::connect(m_slider, SIGNAL(valueChanged(int)), this, SLOT(valueChanged(int)));
	m_box->addWidget(m_slider, 1);
	m_box->addWidget(m_valueLabel, 0);
	updateLabel(0.0);
}

Qt5Range::~Qt5Range() {
}

void Qt5Range::setMin(double min) {
	m_minimum = min;
	if (value() < min) setValue(min);
	updateSlider();
	updateLabel(value());
}

void Qt5Range::setMax(double max) {
	m_maximum = max;
	if (value() > max) setValue(max);
	updateSlider();
	updateLabel(value());
}

void Qt5Range::setDigits(int digits) {
	m_digits = digits;
	updateSlider();
	updateLabel(value());
}

void Qt5Range::show() {
	m_area->show();
	m_labelWidget->show();
}

void Qt5Range::hide() {
	m_area->hide();
	m_labelWidget->hide();
}

bool Qt5Range::visible() const {
	return m_area->isVisible();
}

void Qt5Range::enable() {
	m_area->setEnabled(true);
	m_labelWidget->setEnabled(true);
}

void Qt5Range::disable() {
	m_area->setEnabled(false);
	m_labelWidget->setEnabled(false);
}

bool Qt5Range::enabled() const {
	return m_area->isEnabled();
}

double Qt5Range::value() const {
	return static_cast<double>(m_slider->value()) / std::pow(10.0, m_digits) + m_minimum;
}

void Qt5Range::setValue(double value) {
	if (value > m_maximum) return;
	double range = value - m_minimum;
	m_slider->setValue(std::floor((range * std::pow(10.0, m_digits))));
	updateLabel(value);
}

void Qt5Range::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5Range::widget() {
	return m_area;
}

void Qt5Range::updateSlider() {
	double range = m_maximum - m_minimum;
	m_slider->setMaximum(std::floor((range * std::pow(10.0, m_digits))));
}

void Qt5Range::updateLabel(double value) {
	m_valueLabel->setText(QString::number(value, 'f', m_digits));
}

void Qt5Range::valueChanged(int value) {
	double dValue = static_cast<double>(value) / std::pow(10.0, m_digits) + m_minimum;
	updateLabel(dValue);
	notify(dValue);
}


} // Property
} // GUI
