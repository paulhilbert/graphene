#include "Qt5PropGroup.h"

namespace GUI {
namespace Property {

Qt5Group::Qt5Group(std::string label) : Group(label), m_group(new QGroupBox(QString::fromStdString(label))) {
	m_group->setObjectName("GroupWidget");
	m_group->setStyleSheet("QGroupBox#GroupWidget {border:1px solid gray;border-radius:0px;margin-top: 1ex; padding-top: 10px; font-weight: bold} QGroupBox#GroupWidget::title{subcontrol-origin: margin;subcontrol-position:top center;padding:0 3px;}");
	m_container.setWidget(m_group);
}

Qt5Group::~Qt5Group() {
}

void Qt5Group::show() {
	m_container.show();
}

void Qt5Group::hide() {
	m_container.hide();
}

bool Qt5Group::visible() const {
	return m_container.visible();
}

void Qt5Group::enable() {
	m_container.enable();
}

void Qt5Group::disable() {
	m_container.disable();
}

bool Qt5Group::enabled() const {
	return m_container.enabled();
}

void Qt5Group::setLabel(std::string label) {
	m_group->setTitle(QString::fromStdString(label));
}

QWidget* Qt5Group::widget() {
	return m_container.widget();
}

Bool::Ptr Qt5Group::createBool(std::string label) {
	return m_container.createBool(label);
}

Button::Ptr Qt5Group::createButton(std::string label) {
	return m_container.createButton(label);
}

Choice::Ptr Qt5Group::createChoice(std::string label) {
	return m_container.createChoice(label);
}

Color::Ptr Qt5Group::createColor(std::string label) {
	return m_container.createColor(label);
}

File::Ptr Qt5Group::createFile(std::string label) {
	return m_container.createFile(label);
}

Files::Ptr Qt5Group::createFiles(std::string label) {
	return m_container.createFiles(label);
}

Folder::Ptr Qt5Group::createFolder(std::string label) {
	return m_container.createFolder(label);
}

Group::Ptr Qt5Group::createGroup(std::string label) {
	return m_container.createGroup(label);
}

Number::Ptr Qt5Group::createNumber(std::string label) {
	return m_container.createNumber(label);
}

Range::Ptr Qt5Group::createRange(std::string label) {
	return m_container.createRange(label);
}

Section::Ptr Qt5Group::createSection(std::string label) {
	return m_container.createSection(label);
}

String::Ptr Qt5Group::createString(std::string label) {
	return m_container.createString(label);
}

Tree::Ptr Qt5Group::createTree(std::string label) {
	return m_container.createTree(label);
}

ToggleButton::Ptr Qt5Group::createToggleButton(std::string label) {
	return m_container.createToggleButton(label);
}

Separator::Ptr Qt5Group::createSeparator() {
	return m_container.createSeparator();
}

} // Property
} // GUI
