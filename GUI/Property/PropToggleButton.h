/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYTOGGLEBUTTON_H_
#define PROPERTYTOGGLEBUTTON_H_

/**
 *  @file ToggleButton.h
 *
 *  Defines toggle button property type.
 */

#include "PropBase.h"
#include "PropNotify.h"
#include "PropValue.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to add a button with a static pressed state.
 *
 *  Backends usually implement this as a button that once pressed changes its state
 *  and keeps this state (i.e. stays down if enabled).
 *
 *  @see Bool For another way to input a boolean state.
 */
class ToggleButton : public Base, public Notify<void (bool)>, public Value<bool>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<ToggleButton> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<ToggleButton>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (bool)>::Callback;

	public:
		/**
		 *  @internal ToggleButton(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		ToggleButton(std::string label);

		/**
		 *  @internal ~ToggleButton()
		 *
		 *  @brief Destructor
		 */
		virtual ~ToggleButton();
};

} // Property
} // GUI

#endif /* PROPERTYTOGGLEBUTTON_H_ */
