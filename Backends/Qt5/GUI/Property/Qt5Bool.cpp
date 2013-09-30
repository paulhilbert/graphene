#include "Qt5Bool.h"

#include <QtCore/QString>

namespace GUI {
namespace Property {

Qt5Bool::Qt5Bool(std::string label, QLabel* labelWidget) : Bool(label), m_labelWidget(labelWidget), m_checkBox(new QCheckBox()) {
	//m_checkBox->setText(QString::fromStdString(label));
	m_checkBox->setChecked(false);
	QObject::connect(m_checkBox, SIGNAL(stateChanged(int)), this, SLOT(checkboxClicked(int)));
}

Qt5Bool::~Qt5Bool() {
}

void Qt5Bool::show() {
	m_checkBox->show();
	m_labelWidget->show();
}

void Qt5Bool::hide() {
	m_checkBox->hide();
	m_labelWidget->hide();
}

bool Qt5Bool::visible() const {
	return m_checkBox->isVisible();
}

void Qt5Bool::enable() {
	m_checkBox->setEnabled(true);
	m_labelWidget->setEnabled(true);
}

void Qt5Bool::disable() {
	m_checkBox->setEnabled(false);
	m_labelWidget->setEnabled(false);
}

bool Qt5Bool::enabled() const {
	return m_checkBox->isEnabled();
}

bool Qt5Bool::value() const {
	return m_checkBox->isChecked();
}

void Qt5Bool::setValue(bool value) {
	m_checkBox->setChecked(value);
}

void Qt5Bool::setLabel(std::string label) {
	m_labelWidget->setText(QString::fromStdString(label));
}

QWidget* Qt5Bool::widget() {
	return m_checkBox;
}

void Qt5Bool::checkboxClicked(int state) {
	notify(static_cast<bool>(state == Qt::Checked));
}

} // Property
} // GUI
