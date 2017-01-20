/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "Qt5Settings.h"

#include <GUI/Property/PropBool.h>
using GUI::Property::Boolean;

namespace GUI {

Qt5Settings::Qt5Settings(Log::Ptr log) : m_log(log) {
	m_tabWidget = new QTabWidget();
	m_tabWidget->show();
	m_tabWidget->setTabsClosable(true);
}

Qt5Settings::~Qt5Settings() {
}

Container::Ptr Qt5Settings::add(std::string name, bool hasActiveCheckBox) {
	if (std::find(m_indexMap.begin(), m_indexMap.end() ,name) != m_indexMap.end()) {
		m_log->error("Tab name already exists");
		return Container::Ptr();
	}
	Qt5VisSettings::Ptr tab(new Qt5VisSettings());
	if (hasActiveCheckBox) {
		tab->add<Boolean>("Active: ", "__active__")->setValue(true);
		tab->addSeparator();
	}
	m_indexMap.push_back(name);
	m_tabs.push_back(tab);
	m_tabWidget->addTab(tab->widget(), QString::fromStdString(name));
	m_tabWidget->setCurrentIndex(m_tabWidget->count() - 1);
	return std::dynamic_pointer_cast<Container>(tab);
}

Container::Ptr Qt5Settings::get(std::string name) {
	auto findIt = std::find(m_indexMap.begin(), m_indexMap.end(), name);
	if (findIt == m_indexMap.end()) {
		m_log->error("Tab does not exist");
		return Container::Ptr();
	}
	unsigned int index = std::distance(m_indexMap.begin(), findIt);
	return std::dynamic_pointer_cast<Container>(m_tabs[index]);
}

std::vector<std::string> Qt5Settings::getActiveTabs() const {
	std::vector<std::string> result;
	for (unsigned int idx = 1; idx < m_tabs.size(); ++idx) {
		auto propActive = m_tabs[idx]->get<Boolean>(std::vector<std::string>(1, "__active__"));
		if (!propActive->value()) continue;
		result.push_back(m_indexMap[idx]);
	}
	return result;
}

std::string Qt5Settings::remove(int index) {
	if (!index) return "";
	std::string name = m_indexMap[index];
	m_tabWidget->setCurrentIndex(0);
	m_tabWidget->removeTab(index);
	auto itIdx = m_indexMap.begin();
	std::advance(itIdx, index);
	m_indexMap.erase(itIdx);
	auto itTab = m_tabs.begin();
	std::advance(itTab, index);
	m_tabs.erase(itTab);
	return name;
}

QWidget* Qt5Settings::widget() {
	return m_tabWidget;
}

} // GUI
