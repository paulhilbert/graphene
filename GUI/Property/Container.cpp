#include "Container.h"

#include <Testing/asserts.h>

#include "Bool.h"
#include "Button.h"
#include "Choice.h"
#include "Color.h"
#include "File.h"
#include "Files.h"
#include "Folder.h"
#include "PropGroup.h"
#include "Number.h"
#include "Range.h"
#include "Section.h"
#include "Separator.h"
#include "String.h"
#include "ToggleButton.h"
#include "Tree.h"

namespace GUI {
namespace Property {

Container::Container() : Property::Base() {
}

Container::~Container() {
}

template <>
typename Bool::Ptr Container::add<Bool>(std::string label, std::string id) {
		Bool::Ptr child = createBool(label);
		add(child, id);
		return child;
}

template <>
typename Button::Ptr Container::add<Button>(std::string label, std::string id) {
		Button::Ptr child = createButton(label);
		add(child, id);
		return child;
}

template <>
typename Choice::Ptr Container::add<Choice>(std::string label, std::string id) {
		Choice::Ptr child = createChoice(label);
		add(child, id);
		return child;
}

template <>
typename Color::Ptr Container::add<Color>(std::string label, std::string id) {
		Color::Ptr child = createColor(label);
		add(child, id);
		return child;
}

template <>
typename File::Ptr Container::add<File>(std::string label, std::string id) {
		File::Ptr child = createFile(label);
		add(child, id);
		return child;
}

template <>
typename Files::Ptr Container::add<Files>(std::string label, std::string id) {
		Files::Ptr child = createFiles(label);
		add(child, id);
		return child;
}

template <>
typename Folder::Ptr Container::add<Folder>(std::string label, std::string id) {
		Folder::Ptr child = createFolder(label);
		add(child, id);
		return child;
}

template <>
typename Group::Ptr Container::add<Group>(std::string label, std::string id) {
		Group::Ptr child = createGroup(label);
		add(child, id);
		return child;
}

template <>
typename Number::Ptr Container::add<Number>(std::string label, std::string id) {
		Number::Ptr child = createNumber(label);
		add(child, id);
		return child;
}

template <>
typename Range::Ptr Container::add<Range>(std::string label, std::string id) {
		Range::Ptr child = createRange(label);
		add(child, id);
		return child;
}

template <>
typename Section::Ptr Container::add<Section>(std::string label, std::string id) {
		Section::Ptr child = createSection(label);
		add(child, id);
		return child;
}

template <>
typename String::Ptr Container::add<String>(std::string label, std::string id) {
		String::Ptr child = createString(label);
		add(child, id);
		return child;
}

template <>
typename ToggleButton::Ptr Container::add<ToggleButton>(std::string label, std::string id) {
		ToggleButton::Ptr child = createToggleButton(label);
		add(child, id);
		return child;
}

template <>
typename Tree::Ptr Container::add<Tree>(std::string label, std::string id) {
		Tree::Ptr child = createTree(label);
		add(child, id);
		return child;
}

std::shared_ptr<Separator> Container::addSeparator(std::string id) {
	Separator::Ptr child = createSeparator();
	add(child, id);
	return child;
}

template <class PropertyType>
typename PropertyType::Ptr Container::get(const std::vector<std::string>& path) {
	if (!path.size()) {
		std::cout << "Empty path" << "\n";
		return typename PropertyType::Ptr();
	}

	auto findIt = m_idMap.find(path[0]);
	asserts(findIt != m_idMap.end(), "Child does not exist");
	auto child = m_children[findIt->second];
	if (path.size() == 1) {
		return std::dynamic_pointer_cast<PropertyType>(child);
	}

	auto snd = path.begin(); ++snd;
	std::vector<std::string> rest(snd, path.end());
	asserts(child->isContainer(), "Invalid path");
	return std::dynamic_pointer_cast<Container>(child)->get<PropertyType>(rest);
}

template <class PropertyType>
typename PropertyType::Ptr Container::get(const std::string& first) {
	std::vector<std::string> path(1, first);
	return get<PropertyType>(path);
}

template <class PropertyType>
typename PropertyType::Ptr Container::get(const std::string& first, const std::string& second) {
	std::vector<std::string> path(1, first); path.push_back(second);
	return get<PropertyType>(path);
}

bool Container::isContainer() const {
	return true;
}

void Container::add(Base::Ptr child, std::string id) {
	asserts(m_idMap.find(id) == m_idMap.end(), "Child already exists");
	if (id != "") m_idMap[id] = static_cast<int>(m_children.size());
	m_children.push_back(child);
}

// instantiations
template Bool::Ptr           Container::get<Bool>(const std::vector<std::string>& path);
template Button::Ptr         Container::get<Button>(const std::vector<std::string>& path);
template Choice::Ptr         Container::get<Choice>(const std::vector<std::string>& path);
template Color::Ptr          Container::get<Color>(const std::vector<std::string>& path);
template File::Ptr           Container::get<File>(const std::vector<std::string>& path);
template Files::Ptr          Container::get<Files>(const std::vector<std::string>& path);
template Folder::Ptr         Container::get<Folder>(const std::vector<std::string>& path);
template Group::Ptr          Container::get<Group>(const std::vector<std::string>& path);
template Number::Ptr         Container::get<Number>(const std::vector<std::string>& path);
template Range::Ptr          Container::get<Range>(const std::vector<std::string>& path);
template Section::Ptr        Container::get<Section>(const std::vector<std::string>& path);
template Separator::Ptr      Container::get<Separator>(const std::vector<std::string>& path);
template String::Ptr         Container::get<String>(const std::vector<std::string>& path);
template ToggleButton::Ptr   Container::get<ToggleButton>(const std::vector<std::string>& path);
template Tree::Ptr           Container::get<Tree>(const std::vector<std::string>& path);

template Bool::Ptr           Container::get<Bool>(const std::string& first);
template Button::Ptr         Container::get<Button>(const std::string& first);
template Choice::Ptr         Container::get<Choice>(const std::string& first);
template Color::Ptr          Container::get<Color>(const std::string& first);
template File::Ptr           Container::get<File>(const std::string& first);
template Files::Ptr          Container::get<Files>(const std::string& first);
template Folder::Ptr         Container::get<Folder>(const std::string& first);
template Group::Ptr          Container::get<Group>(const std::string& first);
template Number::Ptr         Container::get<Number>(const std::string& first);
template Range::Ptr          Container::get<Range>(const std::string& first);
template Section::Ptr        Container::get<Section>(const std::string& first);
template Separator::Ptr      Container::get<Separator>(const std::string& first);
template String::Ptr         Container::get<String>(const std::string& first);
template ToggleButton::Ptr   Container::get<ToggleButton>(const std::string& first);
template Tree::Ptr           Container::get<Tree>(const std::string& first);

template Bool::Ptr           Container::get<Bool>(const std::string& first, const std::string& second);
template Button::Ptr         Container::get<Button>(const std::string& first, const std::string& second);
template Choice::Ptr         Container::get<Choice>(const std::string& first, const std::string& second);
template Color::Ptr          Container::get<Color>(const std::string& first, const std::string& second);
template File::Ptr           Container::get<File>(const std::string& first, const std::string& second);
template Files::Ptr          Container::get<Files>(const std::string& first, const std::string& second);
template Folder::Ptr         Container::get<Folder>(const std::string& first, const std::string& second);
template Group::Ptr          Container::get<Group>(const std::string& first, const std::string& second);
template Number::Ptr         Container::get<Number>(const std::string& first, const std::string& second);
template Range::Ptr          Container::get<Range>(const std::string& first, const std::string& second);
template Section::Ptr        Container::get<Section>(const std::string& first, const std::string& second);
template Separator::Ptr      Container::get<Separator>(const std::string& first, const std::string& second);
template String::Ptr         Container::get<String>(const std::string& first, const std::string& second);
template ToggleButton::Ptr   Container::get<ToggleButton>(const std::string& first, const std::string& second);
template Tree::Ptr           Container::get<Tree>(const std::string& first, const std::string& second);

} // Property
} // GUI
