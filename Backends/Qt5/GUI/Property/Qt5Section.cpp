#include "Qt5Section.h"


namespace GUI {
namespace Property {

Qt5Section::Qt5Section(std::string label) : Section(label), m_group(new QGroupBox(QString::fromStdString(label))), m_area(new QWidget()), m_box(new QVBoxLayout()) {
	m_container.setWidget(m_area);
	m_group->setObjectName("SectionWidget");
	m_group->setStyleSheet("QGroupBox#SectionWidget {border:1px solid gray;border-radius:0px;margin-top: 1ex; padding-top: 10px; font-weight: bold} QGroupBox#SectionWidget::title{subcontrol-origin: margin;subcontrol-position:top center;padding:0 3px;}");
	m_group->setCheckable(true);
	m_group->setChecked(true);
	QObject::connect(m_group, SIGNAL(toggled(bool)), this, SLOT(toggled(bool)));
	m_group->setLayout(m_box);
	m_box->addWidget(m_container.widget());
}

Qt5Section::~Qt5Section() {
}

void Qt5Section::setCollapsed(bool collapsed) {
	m_group->setChecked(!collapsed);
}

void Qt5Section::show() {
	m_container.show();
}

void Qt5Section::hide() {
	m_container.hide();
}

bool Qt5Section::visible() const {
	return m_container.visible();
}

void Qt5Section::enable() {
	m_container.enable();
}

void Qt5Section::disable() {
	m_container.disable();
}

bool Qt5Section::enabled() const {
	return m_container.enabled();
}

void Qt5Section::setLabel(std::string label) {
	m_group->setTitle(QString::fromStdString(label));
}

QWidget* Qt5Section::widget() {
	return m_group;
}

void Qt5Section::toggled(bool state) {
	if (state) m_container.show();
	else       m_container.hide();
	//m_container.widget()->setVisible(state);
}

Bool::Ptr Qt5Section::createBool(std::string label) {
	return m_container.createBool(label);
}

Button::Ptr Qt5Section::createButton(std::string label) {
	return m_container.createButton(label);
}

Choice::Ptr Qt5Section::createChoice(std::string label) {
	return m_container.createChoice(label);
}

Color::Ptr Qt5Section::createColor(std::string label) {
	return m_container.createColor(label);
}

File::Ptr Qt5Section::createFile(std::string label) {
	return m_container.createFile(label);
}

Files::Ptr Qt5Section::createFiles(std::string label) {
	return m_container.createFiles(label);
}

Folder::Ptr Qt5Section::createFolder(std::string label) {
	return m_container.createFolder(label);
}

Group::Ptr Qt5Section::createGroup(std::string label) {
	return m_container.createGroup(label);
}

Number::Ptr Qt5Section::createNumber(std::string label) {
	return m_container.createNumber(label);
}

Range::Ptr Qt5Section::createRange(std::string label) {
	return m_container.createRange(label);
}

Section::Ptr Qt5Section::createSection(std::string label) {
	return m_container.createSection(label);
}

String::Ptr Qt5Section::createString(std::string label) {
	return m_container.createString(label);
}

Tree::Ptr Qt5Section::createTree(std::string label) {
	return m_container.createTree(label);
}

ToggleButton::Ptr Qt5Section::createToggleButton(std::string label) {
	return m_container.createToggleButton(label);
}

Separator::Ptr Qt5Section::createSeparator() {
	return m_container.createSeparator();
}

} // Property
} // GUI
