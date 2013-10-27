/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYFILES_H_
#define PROPERTYFILES_H_

/**
 *  @file Files.h
 *
 *  Defines multiple files property type.
 */

#include <include/common.h>

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "PropBase.h"
#include "PropNotify.h"
#include "PropValue.h"
#include "PropLabeled.h"

namespace GUI {
namespace Property {

typedef std::vector<fs::path> Paths;

/**
 *  Property type that provides means to choose multiple files.
 *
 *  Backends usually implement this as a button triggering a file choice dialog.
 */
class Files : public Base, public Notify<void (Paths)>, public Value<Paths>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Files> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Files>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (Paths)>::Callback;

	public:
		/**
		 *  @internal Files(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Files(std::string label);

		/**
		 *  @internal ~Files()
		 *
		 *  @brief Destructor
		 */
		virtual ~Files();

		/**
		 *  Restrict choosable files to a subset specified by extensions.
		 *
		 *  @param extensions A vector of valid file extensions (without the dot!)
		 */
		virtual void setExtensions(std::vector<std::string> extensions) = 0;
};

} // Property
} // GUI

#endif /* PROPERTYFILES_H_ */
