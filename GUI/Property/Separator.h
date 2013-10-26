/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYSEPARATOR_H_
#define PROPERTYSEPARATOR_H_

/**
 *  @file Separator.h
 *
 *  Defines separator property type.
 */

#include "Base.h"

namespace GUI {
namespace Property {

/**
 *  Property type that simply renders a horizontal separator.
 *
 *  Backends usually implement this as a simple horizontal line.
 *
 */
class Separator : public Base {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Separator> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Separator>   WPtr;

	public:
		/**
		 *  @internal Separator()
		 *
		 *  @brief Constructor
		 */
		Separator();

		/**
		 *  @internal ~Separator()
		 *
		 *  @brief Destructor
		 */
		virtual ~Separator();
};

} // Property
} // GUI

#endif /* PROPERTYSEPARATOR_H_ */
