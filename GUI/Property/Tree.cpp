#include "Tree.h"

#include <Testing/asserts.h>
#include <Algorithm/Strings.h>

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
	asserts(m_idToKeyMap.find(id) != m_idToKeyMap.end(), "ID does not exist");
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
	asserts(m_keyToIdMap.find(key) != m_keyToIdMap.end(), "Path does not exist");
	return m_keyToIdMap[key];
}

} // Property
} // GUI
