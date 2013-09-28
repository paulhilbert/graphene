#include "Choice.h"

#include <algorithm>
#include <Testing/asserts.h>

namespace GUI {
namespace Property {

Choice::Choice(std::string label) : Base(), Notify<std::string>(), Value<std::string>(), Labeled(label) {
}

Choice::~Choice() {
}

void Choice::add(std::string id, std::string label) {
	add({id, label});
}

void Choice::add(const Option& option) {
	asserts(std::find(m_options.begin(), m_options.end(), option.id) == m_options.end(), "Trying to add choice property option with duplicate id.");
	m_options.push_back(option.id);
	addOption(option.label);
}

void Choice::add(std::vector<Option>& options) {
	for (const auto& option : options) add(option);
}

std::string Choice::value() const {
	unsigned int idx = getActiveOption();
	asserts(idx < m_options.size(), "Choice property index out of bounds.");
	return m_options[idx];
}

void Choice::setValue(std::string id) {
	auto findIt = std::find(m_options.begin(), m_options.end(), id);
	asserts(findIt != m_options.end(), "Trying to set choice property value/id that does not exist.");
	setActiveOption(std::distance(m_options.begin(), findIt));
}

} // Property
} // GUI
