/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYCOLOR_H_
#define PROPERTYCOLOR_H_

/**
 *  @file Color.h
 *
 *  Defines color property type.
 */

#include <include/common.h>

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to choose a color.
 *
 *  Backends usually implement this as a colored button that shows a color choice dialog.
 *
 */
class Color : public Base, public Notify<void (Eigen::Vector4f)>, public Value<Eigen::Vector4f>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Color> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Color>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (Eigen::Vector4f)>::Callback;

	public:
		/**
		 *  @internal Color(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Color(std::string label);

		/**
		 *  @internal ~Color()
		 *
		 *  @brief Destructor
		 */
		virtual ~Color();
};


} // Property
} // GUI


#endif /* PROPERTYCOLOR_H_ */
