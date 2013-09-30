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
	m_button->setIcon(QIcon("Icons/fileopen.png"));
	QObject::connect(m_button, SIGNAL(clicked()), this, SLOT(buttonClicked()));
}

Qt5Files::~Qt5Files() {
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
	QStringList files = QFileDialog::getOpenFileNames(nullptr, "Open Files...", QString(), tr("All Files (*.*)"), nullptr, 0);
	QStringList::Iterator it = files.begin();
	m_value.clear();
	QString text;
	while (it != files.end()) {
		m_value.push_back(fs::path(it->toStdString()));
		if (it != files.begin()) text += ", ";
		text += *it;
		++it;
	}
	m_lineEdit->setText(text);
	notify(m_value);
}

} // Property
} // GUI
