/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYBOOL_H_
#define PROPERTYBOOL_H_

/**
 *  @file Bool.h
 *
 *  Defines a property to choose a boolean state.
 */

#include "PropBase.h"
#include "PropValue.h"
#include "PropNotify.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

/**
 *  Boolean state property.
 *
 *  Backends usually implement this as a checkbox.
 *
 *  @see ToggleButton for another way to input a boolean state.
 */
class Boolean : public Base, public Value<bool>, public Notify<void (bool)>, public Labeled {
	public:
		/** Shared pointer to this */
		typedef std::shared_ptr<Boolean> Ptr;

		/** Weak pointer to this */
		typedef std::weak_ptr<Boolean>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (bool)>::Callback;

	public:
		/**
		 *  @internal Boolean(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Boolean(std::string label);

		/**
		 *  @internal ~Boolean()
		 *
		 *  @brief Destructor.
		 */
		virtual ~Boolean();
};

} // Property
} // GUI

#endif /* PROPERTYBOOL_H_ */
