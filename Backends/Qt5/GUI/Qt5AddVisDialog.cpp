#include "Qt5AddVisDialog.h"

#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QPushButton>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QListWidgetItem>
#include <GUI/Property/String.h>
using GUI::Property::String;

namespace GUI {

Qt5AddVisDialog::Qt5AddVisDialog(std::string title, bool singleMode) : m_singleMode(singleMode) {
	setWindowTitle(singleMode ? tr("Initial Settings") : QString::fromStdString(title));
	setMinimumSize(720, 420);

	//im_visSelection->setMovement(QListView::Static);
	//im_visSelection->setMaximumWidth(128);
	//im_visSelection->setSpacing(12);
	m_settingPages = new QStackedWidget;
	QPushButton *cancelButton = new QPushButton(singleMode ? tr("Exit") : tr("Cancel"));
	QPushButton *addButton = new QPushButton(singleMode ? tr("Start") : tr("Add"));

	QHBoxLayout *horizontalLayout = new QHBoxLayout;

	if (!singleMode) {
		m_visSelection = new QListWidget;
		horizontalLayout->addWidget(m_visSelection);
	}
	horizontalLayout->addWidget(m_settingPages, 2);

	QHBoxLayout *buttonsLayout = new QHBoxLayout;
	buttonsLayout->addStretch(1);
	buttonsLayout->addWidget(cancelButton);
	buttonsLayout->addWidget(addButton);

	QVBoxLayout *mainLayout = new QVBoxLayout;
	mainLayout->addLayout(horizontalLayout);
	mainLayout->addStretch(1);
	mainLayout->addSpacing(12);
	mainLayout->addLayout(buttonsLayout);
	setLayout(mainLayout);

	connect(cancelButton, SIGNAL(clicked()), this, SLOT(close()));
	connect(addButton, SIGNAL(clicked()), this, SLOT(accept()));
	if (!singleMode) connect(m_visSelection, SIGNAL(currentItemChanged(QListWidgetItem*,QListWidgetItem*)), this, SLOT(changeVis(QListWidgetItem*,QListWidgetItem*)));
}

Qt5AddVisDialog::~Qt5AddVisDialog() {
}

std::string Qt5AddVisDialog::getActiveFactory() const {
	int crt = m_singleMode ? 0 : m_visSelection->currentRow();
	return m_factoryNames[crt];
}

std::string Qt5AddVisDialog::getActiveVisName() const {
	if (m_singleMode) return std::string("Settings");
	int crt = m_visSelection->currentRow();
	return m_settings[crt]->get<String>(std::vector<std::string>(1, "__name__"))->value();
}

Qt5FactorySettings::Ptr Qt5AddVisDialog::addFactory(std::string name) {
	// add selection
	QListWidgetItem *factorySelection = new QListWidgetItem(m_visSelection);
	factorySelection->setText(QString::fromStdString(m_singleMode ? "Settings" : name));
	factorySelection->setTextAlignment(Qt::AlignLeft);
	factorySelection->setFlags(Qt::ItemIsSelectable | Qt::ItemIsEnabled);
	// add settings
	Qt5FactorySettings::Ptr settings(new Qt5FactorySettings(name));
	auto nameProp = settings->add<String>("Visualizer Name: ", "__name__");
	nameProp->setValue(m_singleMode ? "Settings" : name);
	if (m_singleMode) nameProp->hide();
	if (!m_singleMode) settings->addSeparator();
	m_settingPages->addWidget(settings->widget());
	m_settings.push_back(settings);
	// update member
	if (!m_singleMode) m_visSelection->setCurrentRow(0);
	m_factoryNames.push_back(name);

	return settings;
}

void Qt5AddVisDialog::changeVis(QListWidgetItem *current, QListWidgetItem *previous) {
	if (!m_factoryNames.size()) return;
	if (!current) current = previous;
	int idx = m_visSelection->row(current);
	m_settingPages->setCurrentIndex(idx);
}

} // GUI
