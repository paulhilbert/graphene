/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYSECTION_H_
#define PROPERTYSECTION_H_

/**
 *  @file Section.h
 *
 *  Defines property section type.
 */

#include "PropContainer.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to group subproperties.
 *
 *  Backends usually implement this as a framed, labeled box
 *  that groups all properties within and is collapsable (i.e.
 *  the user can for example click a checkbox in order to hide
 *  its content).
 *
 *  @see Group for a non-collapsable version of Section.
 */
class Section : public Container, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Section> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Section>   WPtr;

	public:
		/**
		 *  @internal Section(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Section(std::string label);

		/**
		 *  @internal ~Section()
		 *
		 *  @brief Destructor
		 */
		virtual ~Section();

		/**
		 *  Hide content and reduce to minimum size.
		 *
		 *  Shorthand for setCollapsed(true).
		 */
		void collapse();

		/**
		 *  Unhide content and restore size.
		 *
		 *  Shorthand for setCollapsed(false).
		 */
		void expand();

		/**
		 *  Set collapsed state of section.
		 *
		 *  If parameter is true, the section is collapsed
		 *  to minimum size necessary to provide means for
		 *  expanding again and hides all properties within.
		 *
		 *  @param collapsed Whether the section should be collapsed.
		 */
		virtual void setCollapsed(bool collapsed) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYSECTION_H_ */
