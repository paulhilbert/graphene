#ifndef PROPERTYCONTAINER_H_
#define PROPERTYCONTAINER_H_

#include <include/common.h>

#include "Base.h"

namespace GUI {
namespace Property {

class Bool;
class Button;
class Choice;
class Color;
class File;
class Files;
class Folder;
class Group;
class Number;
class Range;
class Section;
class Separator;
class String;
class ToggleButton;
class Tree;

class Container : public Base {
	public:
		typedef std::shared_ptr<Container> Ptr;
		typedef std::weak_ptr<Container> WPtr;

	public:
		Container();
		virtual ~Container();

		template <class PropertyType>
		typename PropertyType::Ptr add(std::string label, std::string id = "");

		std::shared_ptr<Separator> addSeparator(std::string id = "");

		template <class PropertyType>
		typename PropertyType::Ptr get(const std::vector<std::string>& path);

		bool isContainer() const;

	protected:
		void add(Base::Ptr child, std::string id);
		virtual std::shared_ptr<Bool>          createBool(std::string label) = 0;
		virtual std::shared_ptr<Button>        createButton(std::string label) = 0;
		virtual std::shared_ptr<Choice>        createChoice(std::string label) = 0;
		virtual std::shared_ptr<Color>         createColor(std::string label) = 0;
		virtual std::shared_ptr<File>          createFile(std::string label) = 0;
		virtual std::shared_ptr<Files>         createFiles(std::string label) = 0;
		virtual std::shared_ptr<Folder>        createFolder(std::string label) = 0;
		virtual std::shared_ptr<Group>         createGroup(std::string label) = 0;
		virtual std::shared_ptr<Number>        createNumber(std::string label) = 0;
		virtual std::shared_ptr<Range>         createRange(std::string label) = 0;
		virtual std::shared_ptr<Section>       createSection(std::string label) = 0;
		virtual std::shared_ptr<String>        createString(std::string label) = 0;
		virtual std::shared_ptr<ToggleButton>  createToggleButton(std::string label) = 0;
		virtual std::shared_ptr<Tree>          createTree(std::string label) = 0;
		virtual std::shared_ptr<Separator>     createSeparator() = 0;


	protected:
		std::vector<Base::Ptr>              m_children;
		std::map<std::string, unsigned int> m_idMap;
};

} // Property
} // GUI

#endif /* PROPERTYCONTAINER_H_ */
