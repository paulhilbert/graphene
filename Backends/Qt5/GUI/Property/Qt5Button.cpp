#include "Qt5Button.h"

namespace GUI {
namespace Property {

Qt5Button::Qt5Button(std::string label) : Button(label), m_button(new QPushButton()) {
	m_button->setText(QString::fromStdString(label));
	QObject::connect(m_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
	//m_button->connect(m_button,&QAbstractButton::clicked,[=] (bool){ std::cout << "testing" << "\n";});
}

Qt5Button::~Qt5Button() {
}

void Qt5Button::show() {
	m_button->show();
}

void Qt5Button::hide() {
	m_button->hide();
}

bool Qt5Button::visible() const {
	return m_button->isVisible();
}

void Qt5Button::enable() {
	m_button->setEnabled(true);
}

void Qt5Button::disable() {
	m_button->setEnabled(false);
}

bool Qt5Button::enabled() const {
	return m_button->isEnabled();
}

void Qt5Button::setLabel(std::string label) {
	m_button->setText(QString::fromStdString(label));
}

QWidget* Qt5Button::widget() {
	return dynamic_cast<QWidget*>(m_button);
}

void Qt5Button::buttonClicked() {
	notify();
}

} // Property
} // GUI
