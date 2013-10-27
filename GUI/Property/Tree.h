/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef TREE_H_
#define TREE_H_

/**
 *  @file Tree.h
 *
 *  Defines color property type.
 */

#include <include/common.h>

#include "Base.h"
#include "Labeled.h"
#include "Notify.h"

/**
 *  @file Tree.h
 *
 *  Defines tree property type.
 */

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to display tree structures where each element has a boolean state.
 *
 *  Backends usually implement this as a tree display widget with checkboxes next to each element.
 */
class Tree : public Base, public Labeled, public Notify<void (std::string, bool)> {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Tree> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Tree> WPtr;

		/** Typedef for a path representation */
		typedef std::vector<std::string> Path;

	public:
		/**
		 *  @internal Tree(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Tree(std::string label);

		/**
		 *  @internal ~Tree()
		 *
		 *  @brief Destructor
		 */
		virtual ~Tree();

		/**
		 *  Adds new element in the tree.
		 *
		 *  @param id Unique identifier of this element.
		 *  @param path Path to unique define the position of the new element. The path consists of the labels displayed.
		 *  @param checked Default boolean state of this element.
		 */
		void add(const std::string id, const std::vector<std::string>& path, bool checked = true);

		/**
		 *  Remove element with given identifier.
		 *
		 *  @param id Unique identifier of element to remove.
		 */
		void remove(const std::string& id);

		/**
		 *  Returns whether an element with the specified identifier exists.
		 *
		 *  @param id Identifier to search in the tree.
		 *  @return true iff the specified element exists.
		 */
		bool has(const std::string id) const;

		/**
		 *  Remove element at the specified path.
		 *
		 *  @param path Path to element to be removed.
		 */
		virtual void removePath(const std::vector<std::string>& path) = 0;

	protected:
		std::string  getKey(const std::vector<std::string>& path);
		std::string  getId(const std::vector<std::string>& path);
		virtual void add(const std::vector<std::string>& path, bool checked) = 0;

	protected:
		std::map<std::string, std::string>  m_keyToIdMap;
		std::map<std::string, std::string>  m_idToKeyMap;
};

} // Property
} // GUI

#endif /* TREE_H_ */
