/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#include "PropChoice.h"

#include <include/common.h>

namespace GUI {
namespace Property {

Choice::Choice(std::string label) : Base(), Notify<void (std::string)>(), Value<std::string>(), Labeled(label) {
}

Choice::~Choice() {
}

void Choice::add(std::string id, std::string label) {
	Option opt;
	opt.id = id;
	opt.label = label;
	add(opt);
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
	setActiveOption(static_cast<unsigned int>(std::distance(m_options.begin(), findIt)));
}

} // Property
} // GUI
