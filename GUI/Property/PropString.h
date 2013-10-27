/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYSTRING_H_
#define PROPERTYSTRING_H_

/**
 *  @file String.h
 *
 *  Defines string property type.
 */

#include "PropBase.h"
#include "PropNotify.h"
#include "PropValue.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to enter/modify a string.
 *
 *  Backends usually implement this as a one-line editable text field.
 */
class String : public Base, public Notify<void (std::string)>, public Value<std::string>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<String> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<String>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (std::string)>::Callback;

	public:
		/**
		 *  @internal String(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		String(std::string label);

		/**
		 *  @internal ~String()
		 *
		 *  @brief Destructor
		 */
		virtual ~String();
};

} // Property
} // GUI

#endif /* PROPERTYSTRING_H_ */
