/* This program is free software. It comes without any warranty, to
 * the extent permitted by applicable law. You can redistribute it
 * and/or modify it under the terms of the Do What The Fuck You Want
 * To Public License, Version 2, as published by Sam Hocevar. See
 * the COPYING file for more details */


#ifndef PROPERTYFOLDER_H_
#define PROPERTYFOLDER_H_

/**
 *  @file Folder.h
 *
 *  Defines folder choice property type.
 */

#include <boost/filesystem.hpp>
namespace fs = boost::filesystem;

#include "Base.h"
#include "Notify.h"
#include "Value.h"
#include "Labeled.h"

namespace GUI {
namespace Property {

/**
 *  Property type that provides means to choose a filesystem folder.
 *
 *  Backends usually implement this as a button triggering a folder choice dialog.
 */
class Folder : public Base, public Notify<void (fs::path)>, public Value<fs::path>, public Labeled {
	public:
		/** Shared pointer to this class */
		typedef std::shared_ptr<Folder> Ptr;

		/** Weak pointer to this class */
		typedef std::weak_ptr<Folder>   WPtr;

		/** Specific callback type for this property */
		using Notify<void (fs::path)>::Callback;

	public:
		/**
		 *  @internal Folder(std::string label)
		 *
		 *  @brief Constructor
		 *
		 *  @param label Label for this property.
		 */
		Folder(std::string label);

		/**
		 *  @internal ~Folder()
		 *
		 *  @brief Destructor
		 */
		virtual ~Folder();
};

} // Property
} // GUI

#endif /* PROPERTYFOLDER_H_ */
