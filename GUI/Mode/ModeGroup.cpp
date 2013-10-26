/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "ModeGroup.h"

namespace GUI {
namespace Mode {

Group::Group(Log::Ptr log) : m_log(log) {
}

Group::~Group() {
}

Option::Ptr Group::addOption(std::string id, std::string label, fs::path icon) {
	if (m_modes.find(id) != m_modes.end()) {
		m_log->error("Option already exists");
		return Option::Ptr();
	}
	auto mode = createOption(id, label, icon);
	m_modes[id] = mode;
	m_enabledStates[id] = true;
	m_visibleStates[id] = true;
	return mode;
}

void Group::removeOption(std::string id) {
	auto findIt = m_modes.find(id);
	if (findIt == m_modes.end()) {
		m_log->error("Option does not exist");
		return;
	}
	m_modes.erase(findIt);
}

void Group::setCallback(Callback onChange) {
	m_onChange = std::move(onChange);
}

Option::Ptr Group::option(std::string id) {
	auto findIt = m_modes.find(id);
	if (findIt == m_modes.end()) {
		m_log->error("Option does not exist");
		return Option::Ptr();
	}
	return findIt->second;
}

std::string Group::getCurrentOption() const {
	for (const auto& m : m_modes ) {
		if (m.second->active()) return m.first;
	}
	return "";
}

void Group::enable() {
	for (auto& m : m_modes ) {
		if (m_enabledStates[m.first])	m.second->enable();
		else                          m.second->disable();
	}
}

void Group::disable() {
	for (auto& m : m_modes ) {
		m_enabledStates[m.first] = m.second->enabled();
		m.second->disable();
	}
}

void Group::show() {
	for (auto& m : m_modes ) {
		if (m_visibleStates[m.first])	m.second->show();
		else                          m.second->hide();
	}
}

void Group::hide() {
	for (auto& m : m_modes ) {
		m_visibleStates[m.first] = m.second->visible();
		m.second->hide();
	}
}

void Group::notify(std::string modeId) {
	if (m_modes[modeId]->active()) {
		for (const auto& m : m_modes) {
			if (m.first != modeId) m.second->setActive(false);
		}
	}
	if (m_onChange) m_onChange(modeId, m_modes[modeId]->active());
}

} // Mode
} // GUI
