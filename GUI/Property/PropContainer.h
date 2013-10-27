/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYCONTAINER_H_
#define PROPERTYCONTAINER_H_

/**
 *  @file Container.h
 *
 *  Defines base container property type.
 */

#include <include/common.h>

#include "PropBase.h"

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

/**
 *  Property type that provides a base for container properties. 
 *
 *  This class provides the basic means to manage subproperties.
 *
 *  It should not be instantiated on its own.
 */
class Container : public Base {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Container> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Container> WPtr;

	public:
		/**
		 *  @internal Container()
		 *
		 *  @brief Constructor
		 */
		Container();

		/**
		 *  @internal ~Container()
		 *
		 *  @brief Destructor
		 */
		virtual ~Container();

		/**
		 *  Adds new subproperty with given parameters to this container.
		 *
		 *  This method is used to add properties to any container.
		 *  For example
		 *
		 *      auto boolProp = container.add<Bool>("Example Bool Property", "boolProp");
		 *
		 *  adds a new boolean property to the container with given label and id and
		 *  stores it in a variable.
		 *
		 *  @tparam PropertyType Type of property (i.e. the property types in GUI/Property/)
		 *  @param label Label to use for the property.
		 *  @param id Unique identifier of this property. Use this if you want to access this property from a different function later without storing the returned pointer in a member variable.
		 *  @return Shared pointer to the new property created.
		 */
		template <class PropertyType>
		typename PropertyType::Ptr add(std::string label, std::string id = "");

		/**
		 *  Adds a separator property to this container.
		 *
		 *  @param id Unique identifier of this property. Use this if you want to access this property from a different function later without storing the returned pointer in a member variable.
		 */
		std::shared_ptr<Separator> addSeparator(std::string id = "");

		/**
		 *  Returns subproperty with given path.
		 *
		 *  Consider following code:
		 *
		 *      auto container = gui()->properties()->add<Group>("Group label", "group")
		 *      auto subcontainer = container->add<Group>("Subgroup label", "subgroup")
		 *      auto boolProp = subcontainer->add<Bool>("Bool label", "boolProp");
		 *
		 *  Using this function to get the boolean property would then look like:
		 *
		 *      auto boolProp = gui()->properties()->get<Bool>({"group", "subgroup", "boolProp"});
		 *
		 *  So the path consists of those identifiers you supplied to the relevant add(label, id) calls.
		 *
		 *  @tparam PropertyType Type of property to return.
		 *  @param path Vector of strings representing the identifier path to the property.
		 *  @return Shared pointer to the property defined by path.
		 */
		template <class PropertyType>
		typename PropertyType::Ptr get(const std::vector<std::string>& path);

		/**
		 *  Overwritten implementation of Base::isContainer().
		 *
		 *  @return Equals true.
		 */
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
