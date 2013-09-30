#include "Handle.h"

namespace GUI {
namespace Mode {

Handle::Handle(Log::Ptr log) : m_log(log) {
}

Handle::~Handle() {
}

Group::Ptr Handle::addGroup(std::string id, const Group::Callback& onChange) {
	if (m_groups.find(id) != m_groups.end()) {
		m_log->error("Mode group already exists");
		return Group::Ptr();
	}
	auto group = createGroup(onChange);
	m_groups[id] = group;
	return group;
}

void Handle::removeGroup(std::string id) {
	auto findIt = m_groups.find(id);
	if (findIt == m_groups.end()) {
		m_log->error("Mode group does not exist");
		return;
	}
	m_groups.erase(findIt);
}

Group::Ptr Handle::group(std::string id) {
	auto findIt = m_groups.find(id);
	if (findIt == m_groups.end()) {
		m_log->error("Mode group does not exist");
		return Group::Ptr();
	}
	return findIt->second;
}

void Handle::enable() {
	for (auto it = m_groups.begin(); it != m_groups.end(); ++it) {
		it->second->enable();
	}
}

void Handle::disable() {
	for (auto it = m_groups.begin(); it != m_groups.end(); ++it) {
		it->second->disable();
	}
}

} // Mode
} // GUI
