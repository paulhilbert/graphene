#include "Qt5ToggleButton.h"

namespace GUI {
namespace Property {

Qt5ToggleButton::Qt5ToggleButton(std::string label) : ToggleButton(label), m_button(new QPushButton()) {
	m_button->setText(QString::fromStdString(label));
	m_button->setCheckable(true);
	QObject::connect(m_button, SIGNAL(toggled(bool)), this, SLOT(buttonToggled(bool)));
}

Qt5ToggleButton::~Qt5ToggleButton() {
}

void Qt5ToggleButton::show() {
	m_button->show();
}

void Qt5ToggleButton::hide() {
	m_button->hide();
}

bool Qt5ToggleButton::visible() const {
	return m_button->isVisible();
}

void Qt5ToggleButton::enable() {
	m_button->setEnabled(true);
}

void Qt5ToggleButton::disable() {
	m_button->setEnabled(false);
}

bool Qt5ToggleButton::enabled() const {
	return m_button->isEnabled();
}

bool Qt5ToggleButton::value() const {
	return m_button->isChecked();
}

void Qt5ToggleButton::setValue(bool value) {
	m_button->setChecked(value);
}

void Qt5ToggleButton::setLabel(std::string label) {
	m_button->setText(QString::fromStdString(label));
}

QWidget* Qt5ToggleButton::widget() {
	return dynamic_cast<QWidget*>(m_button);
}

void Qt5ToggleButton::buttonToggled(bool state) {
	notify(state);
}

} // Property
} // GUI
