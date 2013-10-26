/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef GUIMODEOPTION_H_
#define GUIMODEOPTION_H_

/**
 *  @file GUI/Mode/Option.h
 *
 *  Defines single options in mode groups.
 */

#include <iostream>
#include <memory>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "../GUIElement.h"

namespace GUI {
namespace Mode {

class Group;

/**
 *  Mode option access class.
 */
class Option : public GUIElement {
	public:
		/** Shared pointer to this */
		typedef std::shared_ptr<Option> Ptr;
		/** Weak pointer to this */
		typedef std::weak_ptr<Option>   WPtr;

	public:
		/**
		 *  @internal Option(std::string id, std::string label, fs::path icon)
		 *
		 *  @brief Constructor
		 *
		 *  @param id Identifier of this option.
		 *  @param label Label to use for this option.
		 *  @param icon Filesystem path to icon for this option.
		 */
		Option(std::string id, std::string label, fs::path icon);

		/** @internal ~Option()
		 *
		 *  @brief Destructor.
		 */
		virtual ~Option();

		/**
		 *  Returns whether this option is currently active.
		 *
		 *  @return true iff this option is currently active in the GUI.
		 */
		virtual bool active() const = 0;

		/**
		 *  Sets this option active.
		 *
		 *  Usually options are (de-)activated in the GUI, but there are scenarios
		 *  where you might want to do this in code.
		 *  
		 *  @param active Active state to set for this option.
		 */
		virtual void setActive(bool active) = 0;

	protected:
		std::string m_id;
		std::string m_label;
		fs::path    m_icon;

};

} // Mode
} // GUI

#endif /* GUIMODEOPTION_H_ */
