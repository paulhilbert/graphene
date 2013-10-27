/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYFILE_H_
#define PROPERTYFILE_H_

/**
 *  @file File.h
 *
 *  Defines single file property type.
 */

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "PropBase.h"
#include "PropNotify.h"
#include "PropValue.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to choose a single file.
 *
 *  There are two modes for this property - one for opening and one
 *  for saving files.
 *
 *  Backends usually implement this as a button that triggers a file choice dialog.
 *
 */
class File : public Base, public Notify<void (fs::path)>, public Value<fs::path>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<File> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<File>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (fs::path)>::Callback;

		/** Mode that specifies how the property handles file choice options. */
		typedef enum {OPEN, SAVE} Mode;

	public:
		/**
		 *  @internal File(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		File(std::string label);

		/**
		 *  @internal ~File()
		 *
		 *  @brief Destructor
		 */
		virtual ~File();

		/**
		 *  Sets file choice mode
		 *
		 *  @param mode Mode for this property
		 */
		virtual void setMode(Mode mode) = 0;

		/**
		 *  Restrict choosable files to a subset specified by extensions.
		 *
		 *  @param extensions A vector of valid file extensions (without the dot!)
		 */
		virtual void setExtensions(std::vector<std::string> extensions) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYFILE_H_ */
