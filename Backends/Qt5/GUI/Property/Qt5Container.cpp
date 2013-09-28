#include "Qt5Container.h"

#include <QtWidgets/QLabel>
#include <QtWidgets/QSpacerItem>

#include "Qt5Bool.h"
#include "Qt5Button.h"
#include "Qt5Choice.h"
#include "Qt5Color.h"
#include "Qt5File.h"
#include "Qt5Files.h"
#include "Qt5Folder.h"
#include "Qt5Group.h"
#include "Qt5Number.h"
#include "Qt5Range.h"
#include "Qt5Section.h"
#include "Qt5String.h"
#include "Qt5Separator.h"
#include "Qt5ToggleButton.h"
#include "Qt5Tree.h"

namespace GUI {
namespace Property {

Qt5Container::Qt5Container(bool scrollable) : Container(), m_scrollable(scrollable), m_widgetCount(0) {
	m_area->setLayout(m_box);
	if (scrollable) {
		m_outerArea = new QWidget();
		m_outerBox = new QVBoxLayout();
		m_outerBox->setSizeConstraint(QLayout::SetMinAndMaxSize);
		m_scroll = new QScrollArea();
		m_outerArea->setLayout(m_outerBox);
		m_scroll->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
		m_scroll->setHorizontalScrollBarPolicy(Qt::ScrollBarAlwaysOff);
		m_scroll->setWidgetResizable(true);
		m_scroll->setWidget(m_area);
		m_outerBox->addWidget(m_scroll);
	}
}

Qt5Container::~Qt5Container() {
}

void Qt5Container::setWidget(QWidget* widget) {
	m_widget = widget;
	m_layout->setColumnStretch(1, 1);
	m_widget->setLayout(m_layout);
	m_box->addWidget(m_widget, 0);
	m_box->addStretch(1);
}

void Qt5Container::show() {
	if (m_scrollable) m_outerArea->show();
	else              m_area->show();
}

void Qt5Container::hide() {
	if (m_scrollable) m_outerArea->hide();
	else              m_area->hide();
}

bool Qt5Container::visible() const {
	return m_scrollable ? m_outerArea->isVisible() : m_area->isVisible();
}

void Qt5Container::enable() {
	if (m_scrollable) m_outerArea->setEnabled(true);
	else              m_area->setEnabled(true);
}

void Qt5Container::disable() {
	if (m_scrollable) m_outerArea->setEnabled(false);
	else              m_area->setEnabled(false);
}

bool Qt5Container::enabled() const {
	return m_scrollable ? m_outerArea->isEnabled() : m_area->isEnabled();
}

QWidget* Qt5Container::widget() {
	return m_scrollable ? m_outerArea : m_area;
}

Bool::Ptr Qt5Container::createBool(std::string label) {
	QLabel* labelWidget = new QLabel(QString::fromStdString(label));
	Qt5Bool::Ptr result(new Qt5Bool(label, labelWidget));
	m_layout->addWidget(labelWidget,      m_widgetCount,   0, 1, 1, Qt::AlignLeft);
	m_layout->addWidget(result->widget(), m_widgetCount++, 1, 1, 1, Qt::AlignLeft);
	return std::dynamic_pointer_cast<Bool>(result);
}

Button::Ptr Qt5Container::createButton(std::string label) {
	Qt5Button::Ptr result(new Qt5Button(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 1, 1, 1, Qt::AlignLeft);
	return std::dynamic_pointer_cast<Button>(result);
}

Choice::Ptr Qt5Container::createChoice(std::string label) {
	Qt5Choice::Ptr result(new Qt5Choice(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2, Qt::AlignLeft);
	return std::dynamic_pointer_cast<Choice>(result);
}

Color::Ptr Qt5Container::createColor(std::string label) {
	QLabel* labelWidget = new QLabel(QString::fromStdString(label));
	Qt5Color::Ptr result(new Qt5Color(label, labelWidget));
	m_layout->addWidget(labelWidget,      m_widgetCount,   0, 1, 1, Qt::AlignLeft);
	m_layout->addWidget(result->widget(), m_widgetCount++, 1, 1, 1);
	return std::dynamic_pointer_cast<Color>(result);
}

File::Ptr Qt5Container::createFile(std::string label) {
	Qt5File::Ptr result(new Qt5File(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2);
	return std::dynamic_pointer_cast<File>(result);
}

Files::Ptr Qt5Container::createFiles(std::string label) {
	Qt5Files::Ptr result(new Qt5Files(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2);
	return std::dynamic_pointer_cast<Files>(result);
}

Folder::Ptr Qt5Container::createFolder(std::string label) {
	Qt5Folder::Ptr result(new Qt5Folder(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2);
	return std::dynamic_pointer_cast<Folder>(result);
}

Group::Ptr Qt5Container::createGroup(std::string label) {
	Qt5Group::Ptr result(new Qt5Group(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2);
	return std::dynamic_pointer_cast<Group>(result);
}

Number::Ptr Qt5Container::createNumber(std::string label) {
	QLabel* labelWidget = new QLabel(QString::fromStdString(label));
	Qt5Number::Ptr result(new Qt5Number(label, labelWidget));
	m_layout->addWidget(labelWidget,      m_widgetCount,   0, 1, 1, Qt::AlignLeft);
	m_layout->addWidget(result->widget(), m_widgetCount++, 1, 1, 1);
	return std::dynamic_pointer_cast<Number>(result);
}

Range::Ptr Qt5Container::createRange(std::string label) {
	QLabel* labelWidget = new QLabel(QString::fromStdString(label));
	Qt5Range::Ptr result(new Qt5Range(label, labelWidget));
	m_layout->addWidget(labelWidget,      m_widgetCount,   0, 1, 1, Qt::AlignLeft);
	m_layout->addWidget(result->widget(), m_widgetCount++, 1, 1, 1);
	return std::dynamic_pointer_cast<Range>(result);
}

Section::Ptr Qt5Container::createSection(std::string label) {
	Qt5Section::Ptr result(new Qt5Section(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2);
	return std::dynamic_pointer_cast<Section>(result);
}

String::Ptr Qt5Container::createString(std::string label) {
	QLabel* labelWidget = new QLabel(QString::fromStdString(label));
	Qt5String::Ptr result(new Qt5String(label, labelWidget));
	m_layout->addWidget(labelWidget,      m_widgetCount,   0, 1, 1, Qt::AlignLeft);
	m_layout->addWidget(result->widget(), m_widgetCount++, 1, 1, 1);
	return std::dynamic_pointer_cast<String>(result);
}

ToggleButton::Ptr Qt5Container::createToggleButton(std::string label) {
	Qt5ToggleButton::Ptr result(new Qt5ToggleButton(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 1, 1, 1, Qt::AlignLeft);
	return std::dynamic_pointer_cast<ToggleButton>(result);
}

Tree::Ptr Qt5Container::createTree(std::string label) {
	Qt5Tree::Ptr result(new Qt5Tree(label));
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2);
	return std::dynamic_pointer_cast<Tree>(result);
}

Separator::Ptr Qt5Container::createSeparator() {
	Qt5Separator::Ptr result(new Qt5Separator());
	m_layout->addWidget(result->widget(), m_widgetCount++, 0, 1, 2);
	return std::dynamic_pointer_cast<Separator>(result);
}


} // Property
} // GUI
