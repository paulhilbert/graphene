/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYLABELED_H_
#define PROPERTYLABELED_H_

/**
 *  @file Labeled.h
 *
 *  Defines base type for labeled properties.
 */

#include <include/common.h>

namespace GUI {
namespace Property {

/**
 *  Property base type that defines labeled properties.
 *
 *  Properties inherited from this class (which are almost all types)
 *  store a string the GUI backend uses to label the corresponding widgets.
 */
class Labeled {
	public:
		/**
		 *  @internal Labeled(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Labeled(std::string label);

		/**
		 *  @internal ~Labeled()
		 *
		 *  @brief Destructor
		 */
		virtual ~Labeled();

		/**
		 *  Sets label string.
		 *
		 *  @param label Label for this property.
		 */
		virtual void setLabel(std::string label) = 0;

	protected:
		std::string m_label;
};

} // Property
} // GUI

#endif /* PROPERTYLABELED_H_ */
