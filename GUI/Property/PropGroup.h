/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYGROUP_H_
#define PROPERTYGROUP_H_

/**
 *  @file PropGroup.h
 *
 *  Defines property group type.
 */

#include "PropContainer.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to group subproperties.
 *
 *  Backends usually implement this as a framed, labeled box
 *  that groups all properties within.
 *
 *  @see Section for a collapsable version of Group.
 */
class Group : public Container, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Group> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Group>   WPtr;

	public:
		/**
		 *  @internal Group(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Group(std::string label);

		/**
		 *  @internal ~Group()
		 *
		 *  @brief Destructor
		 */
		virtual ~Group();
};

} // Property
} // GUI

#endif /* PROPERTYGROUP_H_ */
