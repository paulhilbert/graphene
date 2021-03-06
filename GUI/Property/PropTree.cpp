/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "PropTree.h"

#include <include/common.h>

namespace GUI {
namespace Property {

Tree::Tree(std::string label) : Base(), Labeled(label), Notify<void (std::string, bool)>() {
}

Tree::~Tree() {
}

void Tree::add(const std::string id, const std::vector<std::string>& path, bool checked) {
	std::string key = getKey(path);
	if (m_keyToIdMap.find(key) != m_keyToIdMap.end()) remove(id);
	m_keyToIdMap[key] = id;
	m_idToKeyMap[id] = key;
	add(path, checked);
}

void Tree::remove(const std::string& id) {
	if (m_idToKeyMap.find(id) == m_idToKeyMap.end()) throw std::runtime_error("ID does not exist");
	std::vector<std::string> path = Algorithm::split(m_idToKeyMap[id], " > ");
	removePath(path);
	m_keyToIdMap.erase(m_idToKeyMap[id]);
	m_idToKeyMap.erase(id);
}

bool Tree::has(const std::string id) const {
	return m_idToKeyMap.find(id) != m_idToKeyMap.end();
}

std::string Tree::getKey(const std::vector<std::string>& path) {
	return Algorithm::join(path, " > ");
}

std::string Tree::getId(const std::vector<std::string>& path) {
	std::string key = getKey(path);
	if (m_keyToIdMap.find(key) == m_keyToIdMap.end()) throw std::runtime_error("Path does not exist");
	return m_keyToIdMap[key];
}

} // Property
} // GUI
