#include "Qt5String.h"

namespace GUI {
namespace Property {

Qt5String::Qt5String(std::string label, QLabel* labelWidget) : String(label), m_labelWidget(labelWidget), m_lineEdit(new QLineEdit()) {
	QObject::connect(m_lineEdit, SIGNAL(textChanged(QString)), this, SLOT(stringChanged(QString)));
}

Qt5String::~Qt5String() {
}

void Qt5String::show() {
	m_lineEdit->show();
	m_labelWidget->show();
}

void Qt5String::hide() {
	m_lineEdit->hide();
	m_labelWidget->hide();
}

bool Qt5String::visible() const {
	return m_lineEdit->isVisible();
}

void Qt5String::enable() {
	m_lineEdit->setEnabled(true);
	m_labelWidget->setEnabled(true);
}

void Qt5String::disable() {
	m_lineEdit->setEnabled(false);
	m_labelWidget->setEnabled(false);
}

bool Qt5String::enabled() const {
	return m_lineEdit->isEnabled();
}

std::string Qt5String::value() const {
	return m_lineEdit->text().toStdString();
}

void Qt5String::setValue(std::string value) {
	m_lineEdit->setText(QString::fromStdString(value));
}

void Qt5String::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5String::widget() {
	return m_lineEdit;
}

void Qt5String::stringChanged(QString text) {
	notify(text.toStdString());
}

} // Property
} // GUI
