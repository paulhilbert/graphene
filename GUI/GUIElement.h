/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef GUIELEMENT_H_
#define GUIELEMENT_H_

/**
 *  @file GUIElement.h
 *
 *  Defines base GUI element class.
 */ 

#include <include/common.h>

namespace GUI {

/**
 *  Base class for all user-modifiable GUI elements.
 *
 *  This is what in a widget toolkit would be called "widget".
 *  Its purpose is to define base functionality like for example visually hiding the element.
 */
class GUIElement {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<GUIElement> Ptr;
		/** Weak pointer to this class */
		typedef std::weak_ptr<GUIElement> WPtr;

	public:
		/** 
		 *  @internal GUIElement()
		 *
		 *  @brief Constructor
		 */
		GUIElement();

		/** 
		 *  @internal ~GUIElement()
		 *
		 *  @brief Destructor */
		virtual ~GUIElement();

		/** Show (hidden) element */
		virtual void show() = 0;

		/** Hide element */
		virtual void hide() = 0;

		/**
		 *  Return visibility state of element.
		 *
		 *  @return true iff element is visible in GUI.
		 */
		virtual bool visible() const = 0;

		/** 
		 *  Enable element
		 *
		 *  An "enabled" element is an element the user can modify.
		 */
		virtual void enable() = 0;

		/** 
		 *  Disable element
		 *
		 *  A "disabled" element is an element the user cannot modify.
		 *  Usually the backend renders disabled elements grayed-out.
		 */
		virtual void disable()= 0;

		/**
		 *  Return enabled state of element.
		 *
		 *  @return true iff element is enabled in GUI.
		 */
		virtual bool enabled() const = 0;
};

} // GUI

#endif /* GUIELEMENT_H_ */
