/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYBASE_H_
#define PROPERTYBASE_H_

/**
 *  @file Base.h
 *
 *  Defines base class for all properties.
 */

#include <include/common.h>

#include "../GUIElement.h"

namespace GUI {
namespace Property {

class Container;

/**
 *  Base class for all properties.
 */
class Base : public GUIElement {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Base> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Base>   WPtr;

	public:
		/** 
		 *  @internal Base()
		 *
		 *  @brief Constructor
		 */
		Base();

		/** 
		 *  @internal ~Base()
		 *
		 *  @brief Destructor
		 */
		virtual ~Base();

		/**
		 *  Returns whether this property is a container property.
		 *
		 *  This method by default returns false and is overwritten by
		 *  container classes like Group or Section
		 *
		 *  @return true iff this property is of container type.
		 */
		virtual bool isContainer() const;
};

} // Property
} // GUI

#endif /* PROPERTYBASE_H_ */
