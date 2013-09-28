#include "Qt5Choice.h"

namespace GUI {
namespace Property {

Qt5Choice::Qt5Choice(std::string label) : Choice(label), m_group(new QGroupBox(QString::fromStdString(label))), m_box(new QVBoxLayout()), m_radioCount(0) {
	m_group->setLayout(m_box);
	m_group->setObjectName("ChoiceGroup");
	m_group->setStyleSheet("QGroupBox#ChoiceGroup {padding-left: 10px; padding-top: 15px}");
	m_box->addStretch(1);

	m_signalMapper = new QSignalMapper();
	connect(m_signalMapper, SIGNAL(mapped(int)), this, SLOT(optionChanged(int))) ;
}

Qt5Choice::~Qt5Choice() {
}

void Qt5Choice::show() {
	m_group->show();
}

void Qt5Choice::hide() {
	m_group->hide();
}

bool Qt5Choice::visible() const {
	return m_group->isVisible();
}

void Qt5Choice::enable() {
	m_group->setEnabled(true);
}

void Qt5Choice::disable() {
	m_group->setEnabled(false);
}

bool Qt5Choice::enabled() const {
	return m_group->isEnabled();
}

void Qt5Choice::setLabel(std::string label) {
	m_label = label;
	m_group->setTitle(QString::fromStdString(label));
}

QWidget* Qt5Choice::widget() {
	return m_group;
}

void Qt5Choice::optionChanged(int option) {
	notify(m_options[option]);
}

void Qt5Choice::addOption(std::string label) {
	QRadioButton* radioButton(new QRadioButton(QString::fromStdString(label)));
	if (!m_radios.size()) radioButton->setChecked(true);
	m_box->insertWidget(m_radioCount++, radioButton);
	//QObject::connect(radioButton, SIGNAL(toggled(bool)), this, SLOT(optionChanged(m_radios.size())));
	connect(radioButton, SIGNAL(clicked(bool)), m_signalMapper, SLOT(map())) ;
	m_signalMapper->setMapping(radioButton, m_radios.size()) ;
	m_radios.push_back(radioButton);
}

unsigned int Qt5Choice::getActiveOption() const {
	for (unsigned int r = 0; r < m_radios.size(); ++r) {
		if (m_radios[r]->isChecked()) return r;
	}
	return 0;
}

void Qt5Choice::setActiveOption(unsigned int option) {
	if (option >= m_radios.size()) return;
	m_radios[option]->setChecked(true);
}

} // Property
} // GUI
